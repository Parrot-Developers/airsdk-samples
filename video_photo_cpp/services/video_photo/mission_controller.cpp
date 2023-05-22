/**
 * Copyright (c) 2023 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the Parrot Company nor the names
 *   of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written
 *   permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#define ULOG_TAG video_photo_ctrl
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "mission_controller.hpp"

/**
 * Custom state machine one can imagine for an example mission whose goal is to
 * record a video and take a photo
 */
enum VideoPhotoStateMachine {
	WAITING_FOR_RECORDING_CONFIG = 0,
	RECORDING_CONFIG_DONE,
	RECORDING_STARTED,
	RECORDING_STOPPED,
	PHOTO_CONFIG_DONE,
	PHOTO_SHOOT_DONE,
};

/**
 * Called once the mission controller interface gets connected
 */
static void onConnected(bool success, void *userdata)
{
	ULOGN("MissionController is connected : %s",
	      success ? "succeeded" : "failed");

	MissionController *ctrlitf = (MissionController *)userdata;
	/* At the beginning of the mission, we start by sending the recording
	 * configuration - mandatory before any recording. The current state is
	 * updated consequently */
	ctrlitf->setVideoPhotoCurrentState(RECORDING_CONFIG_DONE);
	ctrlitf->cmdFcamSetConfigRecording();
}

/**
 * Called once the mission controller interface gets disconnected
 */
static void onDisconnected(bool success, void *userdata)
{
	ULOGN("MissionController is Disconnected : %s",
	      success ? "succeeded" : "failed");
}

/**
 * Called each time the mission controller interface sends a command to the
 * drone, such as the commands related to the front camera actions, land or
 * takeoff and so on
 */
static void onSent(airsdk::control::ControlInterface *control_interface,
		   const arsdk_cmd *cmd,
		   bool success,
		   void *userdata)
{
	char buf[128];
	// Format the commands the mission sends to ease their printing
	arsdk_cmd_fmt(cmd, buf, sizeof(buf));
	ULOGW("MissionController cmd %s has been sent", buf);
}

/**
 * Called each time the mission controller interface receives an event from the
 * drone, such as the response related to the front camera actions, land or
 * takeoff and so on
 */
static void onReceived(airsdk::control::ControlInterface *control_interface,
		       const arsdk_cmd *cmd,
		       void *userdata)
{
	// Transfer those commands to the mission controller interface switch
	// case, that will react to the events accordingly and perform the
	// appropriate moves or next actions (depending on what one have planned
	// with the custom state machine)
	auto self = static_cast<MissionController *>(userdata);
	self->onCmdReceived(cmd);
}

MissionController::MissionController(pomp::Loop &loop)
	: mControlItf(loop),
	  mVideoPhotoCurrentState(WAITING_FOR_RECORDING_CONFIG),
	  hasAlreadyHovered(false)
{
}

int MissionController::start()
{
	// Set up a listener to trigger commands sending and events receiving
	const airsdk::control::Listener<airsdk::control::ControlInterface>
		listener_cb = {
			.connected_cb = onConnected,
			.disconnected_cb = onDisconnected,
			.sent_cb = onSent,
			.received_cb = onReceived,
			.userdata = this,
		};

	// Control connection
	return mControlItf.connect(listener_cb);
}

// Example of how to implement a C++ protobuf message, whose role is to take a
// photo. And then send it as an arsdk command to the drone
int MissionController::cmdFcamStartPhoto()
{
	arsdk_binary payload;
	arsdk_cmd pkt;
	uint16_t service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// Create a message and serialize it
	arsdk::camera::Command cmd;
	// Start_Photo command is a nested oneof of the message Command. So it
	// has to be retrieved as an extension of arsdk::camera::Command to be
	// sent within its packet
	arsdk::camera::Command_StartPhoto *cmd_start_photo =
		cmd.mutable_start_photo();
	if (!cmd.has_start_photo()) {
		ULOGE("Error while start photo command creation");
		return -1;
	}

	// The camera id involved in photo is the front camera, and has to be
	// precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_start_photo->set_camera_id(cam_id);
	if (cmd_start_photo->camera_id() != cam_id) {
		ULOGE("set_camera_id bad allocation in start photo");
		return -1;
	}
	uint16_t msg_num = cmd.id_case();

	/* Allocate buffer for serialization, use internal one if possible */
	size_t len = cmd.ByteSizeLong();
	std::vector<uint8_t> v_data(len);

	/* Serialize and send the message */
	int res =
		cmd.SerializeToArray(static_cast<void *>(v_data.data()), len) ?
			0 :
			-EIO;
	if (res != 0) {
		return res;
	}

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = static_cast<void *>(v_data.data());
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	ULOGN("ControlItf send start photo");
	mControlItf.send(&pkt);

	arsdk_cmd_clear(&pkt);
	return res;
}

// Example of how to implement a C++ protobuf message, whose role is to start
// recording. And then send it as an arsdk command to the drone
int MissionController::cmdFcamStartRecording()
{
	arsdk_binary payload;
	arsdk_cmd pkt;
	uint16_t service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// Create a message and serialize it
	arsdk::camera::Command cmd;
	// Start_Recording command is a nested oneof of the message Command. So
	// it has to be retrieved as an extension of arsdk::camera::Command to
	// be sent within its packet
	arsdk::camera::Command_StartRecording *cmd_start_recording =
		cmd.mutable_start_recording();
	if (!cmd.has_start_recording()) {
		ULOGE("Error while start recording command creation");
		return -1;
	}

	// The camera id involved in recording is the front camera, and has to
	// be precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_start_recording->set_camera_id(cam_id);
	if (cmd_start_recording->camera_id() != cam_id) {
		ULOGE("set_camera_id bad allocation in start recording");
		return -1;
	}
	uint16_t msg_num = cmd.id_case();

	/* Allocate buffer for serialization, use internal one if possible */
	size_t len = cmd.ByteSizeLong();
	std::vector<uint8_t> v_data(len);

	/* Serialize and send the message */
	int res =
		cmd.SerializeToArray(static_cast<void *>(v_data.data()), len) ?
			0 :
			-EIO;
	if (res != 0) {
		return res;
	}

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = static_cast<void *>(v_data.data());
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	ULOGN("ControlItf send start recording");
	mControlItf.send(&pkt);

	arsdk_cmd_clear(&pkt);
	return res;
}

// Example of how to implement a C++ protobuf message, whose role is to stop
// recording. And then send it as an arsdk command to the drone
int MissionController::cmdFcamStopRecording()
{
	arsdk_binary payload;
	arsdk_cmd pkt;
	uint16_t service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// Create a message and serialize it
	arsdk::camera::Command cmd;
	// Stop_Recording command is a nested oneof of the message Command. So
	// it has to be retrieved as an extension of arsdk::camera::Command to
	// be sent within its packet
	arsdk::camera::Command_StopRecording *cmd_stop_recording =
		cmd.mutable_stop_recording();
	if (!cmd.has_stop_recording()) {
		ULOGE("Error while stop recording command creation");
		return -1;
	}

	// The camera id involved in recording is the front camera, and has to
	// be precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_stop_recording->set_camera_id(cam_id);
	if (cmd_stop_recording->camera_id() != cam_id) {
		ULOGE("set_camera_id bad allocation in stop recording");
		return -1;
	}
	uint16_t msg_num = cmd.id_case();

	/* Allocate buffer for serialization, use internal one if possible */
	size_t len = cmd.ByteSizeLong();
	std::vector<uint8_t> v_data(len);

	/* Serialize and send the message */
	int res =
		cmd.SerializeToArray(static_cast<void *>(v_data.data()), len) ?
			0 :
			-EIO;
	if (res != 0) {
		return res;
	}

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = static_cast<void *>(v_data.data());
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	ULOGN("ControlItf send stop recording");
	mControlItf.send(&pkt);

	arsdk_cmd_clear(&pkt);
	return res;
}

// Example of how to implement a C++ protobuf message, whose role is to set
// config to photo. This command is mandatory: it is not possible to take a
// photo if the mode is not set to `photo`. Then send it as an arsdk command
// to the drone
int MissionController::cmdFcamSetConfigPhoto()
{
	arsdk_binary payload;
	arsdk_cmd pkt;
	uint16_t service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// Create a message and serialize it
	arsdk::camera::Command cmd;
	// Command_Configure command is a nested oneof of the message Command.
	// So it has to be retrieved as an extension of arsdk::camera::Command
	// to be sent within its packet
	arsdk::camera::Command_Configure *cmd_configure =
		cmd.mutable_configure();
	if (!cmd.has_configure()) {
		ULOGE("Error while setting up recording command configure");
		return -1;
	}

	// The camera id involved in photo is the front camera, and has to be
	// precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_configure->set_camera_id(cam_id);
	if (cmd_configure->camera_id() != cam_id) {
		ULOGE("set_camera_id bad allocation in set config recording");
		return -1;
	}
	uint16_t msg_num = cmd.id_case();

	// Config command is a nested oneof of the message Command_Configure. So
	// it has to be retrieved as an extension of
	// arsdk::camera::Command_Configure to be sent within its packet
	arsdk::camera::Config *config = cmd_configure->mutable_config();
	if (!cmd_configure->has_config()) {
		ULOGE("Error while setting up recording config");
		return -1;
	}
	// Config command is in charge of the camera mode. In that case, the
	// camera mode is to take a photo
	config->set_camera_mode(arsdk::camera::CameraMode::CAMERA_MODE_PHOTO);
	if (config->camera_mode()
	    != arsdk::camera::CameraMode::CAMERA_MODE_PHOTO) {
		ULOGE("set_camera_mode failed in set config photo");
		return -1;
	}
	// Notify that this field has been changed on purpose with a new value.
	// Otherwise, it won't be taken into account. Setting a new value to a
	// field, without marking it as a selected field, will be ignored
	(*config->mutable_selected_fields())
		[arsdk::camera::Config::kCameraModeFieldNumber] = {};
	// Config command is in charge of the camera photo mode
	config->set_photo_mode(arsdk::camera::PhotoMode::PHOTO_MODE_SINGLE);
	if (config->photo_mode()
	    != arsdk::camera::PhotoMode::PHOTO_MODE_SINGLE) {
		ULOGE("set_photo_mode failed in set config photo");
		return -1;
	}
	// Notify that this field has been changed on purpose with a new value.
	// Otherwise, it won't be taken into account. Setting a new value to a
	// field, without marking it as a selected field, will be ignored
	(*config->mutable_selected_fields())
		[arsdk::camera::Config::kPhotoModeFieldNumber] = {};

	/* Allocate buffer for serialization, use internal one if possible */
	size_t len = cmd.ByteSizeLong();
	std::vector<uint8_t> v_data(len);

	/* Serialize and send the message */
	int res =
		cmd.SerializeToArray(static_cast<void *>(v_data.data()), len) ?
			0 :
			-EIO;
	if (res != 0) {
		return res;
	}

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = static_cast<void *>(v_data.data());
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	ULOGN("ControlItf send set photo config");
	mControlItf.send(&pkt);

	arsdk_cmd_clear(&pkt);
	return res;
}

// Example of how to implement a C++ protobuf message, whose role is to set
// config to recording. This command is mandatory: it is not possible to start a
// recording if the mode is not set to `recording`. Then send it as an arsdk
// command to the drone
int MissionController::cmdFcamSetConfigRecording()
{
	arsdk_binary payload;
	arsdk_cmd pkt;
	uint16_t service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// Create a message and serialize it
	arsdk::camera::Command cmd;
	// Command_Configure command is a nested oneof of the message Command.
	// So it has to be retrieved as an extension of arsdk::camera::Command
	// to be sent within its packet
	arsdk::camera::Command_Configure *cmd_configure =
		cmd.mutable_configure();
	if (!cmd.has_configure()) {
		ULOGE("Error while setting up recording command configure");
		return -1;
	}

	// The camera id involved in recording is the front camera, and has to
	// be precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_configure->set_camera_id(cam_id);
	if (cmd_configure->camera_id() != cam_id) {
		ULOGE("set_camera_id bad allocation in set config recording");
		return -1;
	}
	uint16_t msg_num = cmd.id_case();

	// Config command is a nested oneof of the message Command_Configure. So
	// it has to be retrieved as an extension of
	// arsdk::camera::Command_Configure to be sent within its packet
	arsdk::camera::Config *config = cmd_configure->mutable_config();
	if (!cmd_configure->has_config()) {
		ULOGE("Error while setting up recording config");
		return -1;
	}
	// Config command is in charge of the camera mode. In that case, the
	// camera mode is to record
	config->set_camera_mode(
		arsdk::camera::CameraMode::CAMERA_MODE_RECORDING);
	if (config->camera_mode()
	    != arsdk::camera::CameraMode::CAMERA_MODE_RECORDING) {
		ULOGE("set_camera_mode failed in set config recording");
		return -1;
	}
	// Notify that this field has been changed on purpose with a new value.
	// Otherwise, it won't be taken into account. Setting a new value to a
	// field, without marking it as a selected field, will be ignored
	(*config->mutable_selected_fields())
		[arsdk::camera::Config::kCameraModeFieldNumber] = {};

	/* Allocate buffer for serialization, use internal one if possible */
	size_t len = cmd.ByteSizeLong();
	std::vector<uint8_t> v_data(len);

	/* Serialize and send the message */
	int res =
		cmd.SerializeToArray(static_cast<void *>(v_data.data()), len) ?
			0 :
			-EIO;
	if (res != 0) {
		return res;
	}

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = static_cast<void *>(v_data.data());
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	ULOGN("ControlItf send set recording config");
	mControlItf.send(&pkt);

	arsdk_cmd_clear(&pkt);
	return res;
}

int MissionController::onCmdReceived(const arsdk_cmd *cmd)
{
	int res = 0;
	uint16_t service_id = 0;
	uint16_t msg_num = 0;
	arsdk_binary payload;
	arsdk::camera::Event evt;

	// The first state is supposed to be over. Otherwise, it means the
	// controller is not connected since the config has not been set
	if (this->mVideoPhotoCurrentState == WAITING_FOR_RECORDING_CONFIG) {
		return -1;
	}

	if (cmd->id == ARSDK_ID_GENERIC_CUSTOM_EVT) {
		// Decode the generic custom event
		res = arsdk_cmd_dec_Generic_Custom_evt(
			cmd, &service_id, &msg_num, &payload);
		if (res < 0) {
			ULOGE("Generic custom decoded failed");
			return res;
		}
		// Make sure it is a protobuf message
		if (service_id
		    != msghub_utils_get_service_id(
			    arsdk__camera__event__descriptor.name)) {
			return -1;
		}

		// Decode the protobuf message
		if (!evt.ParseFromArray((const uint8_t *)payload.cdata,
					payload.len)) {
			ULOGE("Generic custom arsdk event unpack failed");
			return -1;
		}

		switch (evt.id_case()) {
		case arsdk::camera::Event::IdCase::kState:
			this->reactToEventState(&evt);
			break;
		case arsdk::camera::Event::IdCase::kPhoto:
			this->reactInSmToEventPhoto(&evt);
			break;
		case arsdk::camera::Event::IdCase::kRecording:
			this->reactInSmToEventRecording(&evt);
			break;
		default:
			break;
		}
	}

	return res;
}

void MissionController::reactToEventState(arsdk::camera::Event *evt)
{
	// scroll over the selected fields (meaning the fields whose values have
	// changed AND have been notified about it) to check if the Config Field
	// - of Event_State message - is new to the drone. That would mean the
	// new config set has succeeded
	for (auto const &entry : evt->state().selected_fields()) {
		switch (entry.first) {
		case arsdk::camera::Event_State::kConfigFieldNumber:
			this->reactToEventStateConfigFieldNumber(evt);
			break;
		default:
			break;
		}
	}
}

void MissionController::reactToEventStateConfigFieldNumber(
	arsdk::camera::Event *evt)
{
	// scroll over the selected fields (meaning the fields whose values have
	// changed AND have been notified about it) to check if the CameraMode
	// Field
	// - of Config message - is new to the drone. That would mean the new
	// camera mode set has succeeded
	for (auto const &entry : evt->state().config().selected_fields()) {
		switch (entry.first) {
		case arsdk::camera::Config::kCameraModeFieldNumber:
			this->reactInSmToCameraMode(
				evt->state().config().camera_mode());
			break;
		default:
			break;
		}
	}
}

void MissionController::reactInSmToCameraMode(arsdk::camera::CameraMode mode)
{
	if ((mode == arsdk::camera::CameraMode::CAMERA_MODE_PHOTO)
	    && (this->mVideoPhotoCurrentState == PHOTO_CONFIG_DONE)) {
		// if one has received from the drone that a photo config set up
		// command has been sent + the current state noticed that one
		// has sent a photo config set up, then one asks to shoot a
		// photo
		this->mVideoPhotoCurrentState = PHOTO_SHOOT_DONE;
		this->cmdFcamStartPhoto();
	} else if ((mode == arsdk::camera::CameraMode::CAMERA_MODE_RECORDING)
		   && (this->mVideoPhotoCurrentState
		       == RECORDING_CONFIG_DONE)) {
		// if one has received from the drone that a recording config
		// set up command has been sent + the current state noticed that
		// one has sent a recording config set up, then one asks to
		// start recording
		this->mVideoPhotoCurrentState = RECORDING_STARTED;
		this->cmdFcamStartRecording();
	}
}

void MissionController::reactInSmToEventPhoto(arsdk::camera::Event *evt)
{
	if ((evt->photo().type() == arsdk::camera::PhotoEvent::PHOTO_EVENT_STOP)
	    && (this->mVideoPhotoCurrentState == PHOTO_SHOOT_DONE)) {
		// if one has received from the drone that a photo shoot command
		// has been sent + the current state noticed that a photo has
		// been shot, then the mission is over
	}
}

void MissionController::reactInSmToEventRecording(arsdk::camera::Event *evt)
{
	if ((evt->recording().type()
	     == arsdk::camera::RecordingEvent::RECORDING_EVENT_START)
	    && (this->mVideoPhotoCurrentState == RECORDING_STARTED)) {
		// if one has received from the drone that a video recording
		// start command has been sent + the current state noticed that
		// one has started a recording, then one asks to stop recording
		this->mVideoPhotoCurrentState = RECORDING_STOPPED;
		this->cmdFcamStopRecording();
	} else if ((evt->recording().type()
		    == arsdk::camera::RecordingEvent::RECORDING_EVENT_STOP)
		   && (this->mVideoPhotoCurrentState == RECORDING_STOPPED)) {
		// if one has received from the drone that a video recording
		// stop command has been sent + the current state noticed that
		// one has stopped the video recording, then one asks to set up
		// photo config to switch to photo mode
		this->mVideoPhotoCurrentState = PHOTO_CONFIG_DONE;
		this->cmdFcamSetConfigPhoto();
	}
}
