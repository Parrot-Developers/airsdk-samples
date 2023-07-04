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
#include "mission_controller.h"

#define ULOG_TAG_mission_controller video_photo_ctrl
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG_mission_controller);

struct mission_controller {
	/**
	 * a control interface is a Parrot object, responsible for connecting to
	 * the autopilot, and triggers the 'Send command' and 'Received event'
	 */
	struct airsdk_control_itf *control_itf;
	/**
	 * a listener spooks on the commands sent and events received, and
	 * transmit them to the control interface, to whom it is connected
	 */
	struct airsdk_control_itf_listener listener;

	/*
	 * current state of the custom state machine we are at. The state
	 machine is shaped as an enum, and enables to follow this sequence of
	 events :
	 * - send recording configuration - mandatory before recording anything
	 * - start recording
	 * - stop recording
	 * - send photo configuration - mandatory before taking any photo
	 * - take photo
	 */
	enum video_photo_state_machine video_photo_current_state;
};

struct mission_controller *mission_controller_new(struct pomp_loop *ptr_loop)
{
	struct mission_controller *mctrl;

	/* allocate memory for mission controller */
	mctrl = (struct mission_controller *)calloc(1, sizeof(*mctrl));
	if (!mctrl) {
		ULOGE("Error while allocating memory to mission_controller");
		mission_controller_destroy(mctrl);
		return NULL;
	}

	/* allocate a control interface object, link it to the control interface
	 * member of the mission controller and creates an internal pomp loop */
	if (airsdk_control_itf_new_with_loop(ptr_loop, &mctrl->control_itf)
	    != 0) {
		ULOGE("Error while creating a control interface");
		mission_controller_destroy(mctrl);
		return NULL;
	}

	return mctrl;
}

/**
 * free memory
 */
void mission_controller_destroy(struct mission_controller *mctrl)
{
	if (!mctrl)
		return;
	free(mctrl->control_itf);
	free(mctrl);
}

int mission_controller_init(struct mission_controller *mctrl)
{
	if (!mctrl)
		return -EINVAL;

	mctrl->video_photo_current_state = WAITING_FOR_RECORDING_CONFIG;

	// Set up a listener to trigger commands sending and events receiving
	mctrl->listener.connected_cb = on_connected;
	mctrl->listener.disconnected_cb = on_disconnected;
	mctrl->listener.sent_cb = on_sent;
	mctrl->listener.received_cb = on_received;
	mctrl->listener.userdata = mctrl;

	return 0;
}

int mission_controller_start(struct mission_controller *mctrl)
{
	int res = start(mctrl);
	if (res != 0) {
		ULOGE("Error while starting mission_controller");
		ULOG_ERRNO("mission_controller::start", -res);
	} else
		ULOGI("mission_controller has started successfully");

	return res;
}

/**
 * Called once the mission controller interface gets connected
 */
static void on_connected(bool success, void *userdata)
{
	ULOGN("mission_controller is connected : %s",
	      success ? "succeeded" : "failed");

	struct mission_controller *mctrl =
		(struct mission_controller *)userdata;
	/* At the beginning of the mission, we start by sending the recording
	 configuration - mandatory before any recording. The current state
	 is updated consequently */
	mctrl->video_photo_current_state = RECORDING_CONFIG_DONE;
	cmd_fcam_set_config_recording(mctrl);
}

/**
 * Called once the mission controller interface gets disconnected
 */
static void on_disconnected(bool success, void *userdata)
{
	ULOGN("mission_controller is Disconnected : %s",
	      success ? "succeeded" : "failed");
}

/**
 * Called each time the mission controller interface sends a command to the
 * drone, such as the commands related to the front camera actions, land or
 * takeoff and so on
 */
static void on_sent(struct airsdk_control_itf *ctrlitf,
		    const struct arsdk_cmd *cmd,
		    bool success,
		    void *userdata)
{
	char buf[128];
	// Format the commands the mission sends to ease their printing
	arsdk_cmd_fmt(cmd, buf, sizeof(buf));
	ULOGW("mission_controller cmd %s has been sent", buf);
}

/**
 * Called each time the mission controller interface receives an event from the
 * drone, such as the response related to the front camera actions, land or
 * takeoff and so on
 */
static void on_received(struct airsdk_control_itf *ctrlitf,
			const struct arsdk_cmd *cmd,
			void *userdata)
{
	struct mission_controller *mctrl =
		(struct mission_controller *)userdata;
	// Transfer those commands to the mission controller
	// interface switch case, that will react to the events accordingly and
	// perform the appropriate moves or next actions (depending on what one
	// have planned with the custom state machine)
	on_cmd_received(cmd, mctrl);
}

static int start(struct mission_controller *mctrl)
{
	// Set up the listener by attributing its callback, and connect it to
	// the control interface, that will deal as the messages hub
	int res = airsdk_control_itf_connect(mctrl->control_itf,
					     &mctrl->listener);
	ULOGW("connect res %d", res);
	if (res != 0)
		ULOGE("Control Interface connection failed: %d", res);
	if (!airsdk_control_itf_is_connected(mctrl->control_itf)) {
		ULOGE("Control Interface is not connected");
	}
	return res;
}

// Example of how to implement a C protobuf message, whose role is to take a
// photo. And then send it as an arsdk command to the drone
static int cmd_fcam_start_photo(struct mission_controller *mctrl)
{
	struct arsdk_binary payload = {NULL, 0};
	struct arsdk_cmd pkt;
	uint16_t service_id;
	uint16_t msg_num;
	size_t len;
	uint8_t *data;
	int res;

	// Create a message and serialize it
	Arsdk__Camera__Command cmd = ARSDK__CAMERA__COMMAND__INIT;
	// StartPhoto is a nested oneof of the message Command. So it has
	// to be retrieved as an extension of arsdk::camera::Command to be sent
	// within its packet
	Arsdk__Camera__Command__StartPhoto cmd_start_photo =
		ARSDK__CAMERA__COMMAND__START_PHOTO__INIT;
	service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// The camera id involved in photo is the front camera, and has to be
	// precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_start_photo.camera_id = cam_id;

	cmd.id_case = ARSDK__CAMERA__COMMAND__ID_START_PHOTO;
	cmd.start_photo = &cmd_start_photo;
	msg_num = cmd.id_case;

	/* Allocate buffer for serialization, use internal one if possible */
	len = protobuf_c_message_get_packed_size(&cmd.base);
	data = (uint8_t *)malloc(len);
	if (data == NULL) {
		ULOGE("malloc failed in cmd_fcam_start_photo");
		res = -EPERM;
		return res;
	}

	/* Pack the message */
	protobuf_c_message_pack(&cmd.base, data);

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = data;
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	if (!airsdk_control_itf_send(mctrl->control_itf, &pkt, NULL, NULL)) {
		res = -1;
		ULOGN("ControlItf send start photo failed");
	} else
		ULOGN("ControlItf send start photo");

	arsdk_cmd_clear(&pkt);
	return res;
}

// Example of how to implement a C protobuf message, whose role is to start
// recording. And then send it as an arsdk command to the drone
static int cmd_fcam_start_recording(struct mission_controller *mctrl)
{
	struct arsdk_binary payload = {NULL, 0};
	struct arsdk_cmd pkt;
	uint16_t service_id;
	uint16_t msg_num;
	size_t len;
	uint8_t *data;
	int res;

	// Create a message and serialize it
	Arsdk__Camera__Command cmd = ARSDK__CAMERA__COMMAND__INIT;
	// StartRecording command is a nested oneof of the message Command. So
	// it has to be retrieved as an extension of arsdk::camera::Command to
	// be sent within its packet
	Arsdk__Camera__Command__StartRecording cmd_start_recording =
		ARSDK__CAMERA__COMMAND__START_RECORDING__INIT;
	service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// The camera id involved in recording is the front camera, and has to
	// be precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_start_recording.camera_id = cam_id;

	cmd.id_case = ARSDK__CAMERA__COMMAND__ID_START_RECORDING;
	cmd.start_recording = &cmd_start_recording;
	msg_num = cmd.id_case;

	/* Allocate buffer for serialization, use internal one if possible */
	len = protobuf_c_message_get_packed_size(&cmd.base);
	data = (uint8_t *)malloc(len);
	if (data == NULL) {
		ULOGE("malloc failed in cmd_fcam_start_recording");
		res = -EPERM;
		return res;
	}

	/* Pack the message */
	protobuf_c_message_pack(&cmd.base, data);

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = data;
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	if (!airsdk_control_itf_send(mctrl->control_itf, &pkt, NULL, NULL)) {
		res = -1;
		ULOGN("ControlItf send start recording failed");
	} else
		ULOGN("ControlItf send start recording");

	arsdk_cmd_clear(&pkt);
	return res;
}

// Example of how to implement a C protobuf message, whose role is to stop
// recording. And then send it as an arsdk command to the drone
static int cmd_fcam_stop_recording(struct mission_controller *mctrl)
{
	struct arsdk_binary payload = {NULL, 0};
	struct arsdk_cmd pkt;
	uint16_t service_id;
	uint16_t msg_num;
	size_t len;
	uint8_t *data;
	int res;

	// Create a message and serialize it
	Arsdk__Camera__Command cmd = ARSDK__CAMERA__COMMAND__INIT;
	// StopRecording command is a nested oneof of the message Command. So it
	// has to be retrieved as an extension of arsdk::camera::Command to be
	// sent within its packet
	Arsdk__Camera__Command__StopRecording cmd_stop_recording =
		ARSDK__CAMERA__COMMAND__STOP_RECORDING__INIT;
	service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// The camera id involved in recording is the front camera, and has to
	// be precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_stop_recording.camera_id = cam_id;

	cmd.id_case = ARSDK__CAMERA__COMMAND__ID_STOP_RECORDING;
	cmd.stop_recording = &cmd_stop_recording;
	msg_num = cmd.id_case;

	/* Allocate buffer for serialization, use internal one if possible */
	len = protobuf_c_message_get_packed_size(&cmd.base);
	data = (uint8_t *)malloc(len);
	if (data == NULL) {
		ULOGE("malloc failed in cmd_fcam_stop_recording");
		res = -EPERM;
	}

	/* Pack the message */
	protobuf_c_message_pack(&cmd.base, data);

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = data;
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	if (!airsdk_control_itf_send(mctrl->control_itf, &pkt, NULL, NULL)) {
		res = -1;
		ULOGN("ControlItf send stop recording failed");
	} else
		ULOGN("ControlItf send stop recording");

	arsdk_cmd_clear(&pkt);
	return res;
}

// Example of how to implement a C protobuf message, whose role is to set
// config to photo. This command is mandatory: it is not possible to take a
// photo if the mode is not set to `photo`. Then send it as an arsdk command
// to the drone
static int cmd_fcam_set_config_photo(struct mission_controller *mctrl)
{
	struct arsdk_binary payload = {NULL, 0};
	struct arsdk_cmd pkt;
	uint16_t service_id;
	uint16_t msg_num;
	size_t len;
	uint8_t *data;
	int res;

	// Create a message and serialize it
	Arsdk__Camera__Command cmd = ARSDK__CAMERA__COMMAND__INIT;
	// Configure command is a nested oneof of the message Command. So it
	// has to be retrieved as an extension of arsdk::camera::Command to be
	// sent within its packet
	Arsdk__Camera__Command__Configure cmd_configure =
		ARSDK__CAMERA__COMMAND__CONFIGURE__INIT;
	// Config command is a nested oneof of the message Command_Configure. So
	// it has to be retrieved as an extension of
	// arsdk::camera::Command_Configure to be sent within its packet
	Arsdk__Camera__Config config = ARSDK__CAMERA__CONFIG__INIT;
	// It is necessary to stress the fact 2 fields of the Config command are
	// getting changed, in order to take them into account. If it is not
	// explicitly marked down, these changes won't have effect
	// The first value of the field is its new value, the second remains an
	// empty google protobuf type
	static Google__Protobuf__Empty empty = GOOGLE__PROTOBUF__EMPTY__INIT;
	Arsdk__Camera__Config__SelectedFieldsEntry sfields[2] = {
		ARSDK__CAMERA__CONFIG__SELECTED_FIELDS_ENTRY__INIT,
		ARSDK__CAMERA__CONFIG__SELECTED_FIELDS_ENTRY__INIT};
	Arsdk__Camera__Config__SelectedFieldsEntry *psfields[2] = {
		&sfields[0],
		&sfields[1],
	};
	service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// Config command is in charge of the camera mode. In that case, the
	// camera mode is to take a photo
	config.camera_mode = ARSDK__CAMERA__CAMERA_MODE__CAMERA_MODE_PHOTO;
	sfields[0].key = ARSDK__CAMERA__CONFIG__CAMERA_MODE__FIELD_NUMBER;
	sfields[0].value = &empty;
	// Config command is in charge of the camera photo mode. In that case,
	// the camera photo mode is single (for a single photo)
	config.photo_mode = ARSDK__CAMERA__PHOTO_MODE__PHOTO_MODE_SINGLE;
	sfields[1].key = ARSDK__CAMERA__CONFIG__PHOTO_MODE__FIELD_NUMBER;
	sfields[1].value = &empty;

	// Notify that this field has been changed on purpose with a new value.
	// Otherwise, it won't be taken into account. Setting a new value to a
	// field, without marking it as a selected field, will be ignored
	config.selected_fields = psfields;
	config.n_selected_fields = 2;

	// The camera id involved in photo is the front camera, and has to be
	// precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_configure.camera_id = cam_id;

	cmd_configure.config = &config;
	cmd.id_case = ARSDK__CAMERA__COMMAND__ID_CONFIGURE;
	cmd.configure = &cmd_configure;
	msg_num = cmd.id_case;

	/* Allocate buffer for serialization, use internal one if possible */
	len = protobuf_c_message_get_packed_size(&cmd.base);
	data = (uint8_t *)malloc(len);
	if (data == NULL) {
		ULOGE("malloc failed in cmd_fcam_set_config_photo");
		res = -EPERM;
		return res;
	}

	/* Pack the message */
	protobuf_c_message_pack(&cmd.base, data);

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = data;
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	if (!airsdk_control_itf_send(mctrl->control_itf, &pkt, NULL, NULL)) {
		res = -1;
		ULOGE("ControlItf send set config failed (mode photo)");
	} else
		ULOGN("ControlItf send set config (mode photo)");

	arsdk_cmd_clear(&pkt);
	return res;
}

// Example of how to implement a C++ protobuf message, whose role is to set
// config to recording. This command is mandatory: it is not possible to start a
// recording if the mode is not set to `recording`. Then send it as an arsdk
// command to the drone
static int cmd_fcam_set_config_recording(struct mission_controller *mctrl)
{
	struct arsdk_binary payload = {NULL, 0};
	struct arsdk_cmd pkt;
	uint16_t service_id;
	uint16_t msg_num;
	size_t len;
	uint8_t *data;
	int res;

	// Create a message and serialize it
	Arsdk__Camera__Command cmd = ARSDK__CAMERA__COMMAND__INIT;
	// Command_Configure command is a nested oneof of the message Command.
	// So it has to be retrieved as an extension of arsdk::camera::Command
	// to be sent within its packet
	Arsdk__Camera__Command__Configure cmd_configure =
		ARSDK__CAMERA__COMMAND__CONFIGURE__INIT;
	// Config command is a nested oneof of the message Command_Configure. So
	// it has to be retrieved as an extension of
	// arsdk::camera::Command_Configure to be sent within its packet
	Arsdk__Camera__Config config = ARSDK__CAMERA__CONFIG__INIT;
	// It is necessary to stress the fact 1 field of the Config command is
	// getting changed, in order to take it into account. If it is not
	// explicitly marked down, that change won't have effect
	// The first value of the field is its new value, the second remains an
	// empty google protobuf type
	static Google__Protobuf__Empty empty = GOOGLE__PROTOBUF__EMPTY__INIT;
	Arsdk__Camera__Config__SelectedFieldsEntry sfields[1] = {
		ARSDK__CAMERA__CONFIG__SELECTED_FIELDS_ENTRY__INIT};
	Arsdk__Camera__Config__SelectedFieldsEntry *psfields[1] = {&sfields[0]};
	service_id = msghub_utils_get_service_id(
		arsdk__camera__command__descriptor.name);

	// Config command is in charge of the camera mode. In that case, the
	// camera mode is to record
	config.camera_mode = ARSDK__CAMERA__CAMERA_MODE__CAMERA_MODE_RECORDING;
	sfields[0].key = ARSDK__CAMERA__CONFIG__CAMERA_MODE__FIELD_NUMBER;
	sfields[0].value = &empty;

	// Notify that this field has been changed on purpose with a new value.
	// Otherwise, it won't be taken into account. Setting a new value to a
	// field, without marking it as a selected field, will be ignored
	config.selected_fields = psfields;
	config.n_selected_fields = 1;

	// The camera id involved in recording is the front camera, and has to
	// be precised when sending the command
	uint8_t cam_id = 0; // FCAM
	cmd_configure.camera_id = cam_id;

	cmd_configure.config = &config;
	cmd.id_case = ARSDK__CAMERA__COMMAND__ID_CONFIGURE;
	cmd.configure = &cmd_configure;
	msg_num = cmd.id_case;

	/* Allocate buffer for serialization, use internal one if possible */
	len = protobuf_c_message_get_packed_size(&cmd.base);
	data = (uint8_t *)malloc(len);
	if (data == NULL) {
		ULOGE("malloc failed in cmd_fcam_set_config_recording");
		res = -EPERM;
		return res;
	}

	/* Pack the message */
	protobuf_c_message_pack(&cmd.base, data);

	/* Prepare payload */
	arsdk_cmd_init(&pkt);
	payload.cdata = data;
	payload.len = len;
	res = arsdk_cmd_enc_Generic_Custom_cmd(
		&pkt, service_id, msg_num, &payload);

	/* Failed to prepare event */
	if (res != 0) {
		arsdk_cmd_clear(&pkt);
		return res;
	}

	if (!airsdk_control_itf_send(mctrl->control_itf, &pkt, NULL, NULL)) {
		res = -1;
		ULOGE("ControlItf send set config failed (mode recording)");
	} else
		ULOGN("ControlItf send set config (mode recording)");

	arsdk_cmd_clear(&pkt);
	return res;
}

static int on_cmd_received(const struct arsdk_cmd *cmd,
			   struct mission_controller *mctrl)
{
	int res = 0;
	uint16_t service_id = 0;
	uint16_t msg_num = 0;
	struct arsdk_binary payload;

	// The first state is supposed to be over. Otherwise, it means the
	// controller is not connected since the config has not been set
	if (mctrl->video_photo_current_state == WAITING_FOR_RECORDING_CONFIG) {
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

		Arsdk__Camera__Event *evt;
		// Decode the protobuf message
		evt = arsdk__camera__event__unpack(
			NULL, payload.len, (const uint8_t *)payload.cdata);
		if (evt == NULL) {
			ULOGE("Generic custom arsdk event unpack failed");
			return -1;
		}

		switch (evt->id_case) {
		case ARSDK__CAMERA__EVENT__ID_STATE:
			react_to_event_state(evt, mctrl);
			break;
		case ARSDK__CAMERA__EVENT__ID_PHOTO:
			react_in_sm_to_event_photo(evt, mctrl);
			break;
		case ARSDK__CAMERA__EVENT__ID_RECORDING:
			react_in_sm_to_event_recording(evt, mctrl);
			break;
		default:
			break;
		}
		arsdk__camera__event__free_unpacked(evt, NULL);
	}

	return res;
}

static void react_to_event_state(Arsdk__Camera__Event *evt,
				 struct mission_controller *mctrl)
{
	// scroll over the selected fields (meaning the fields whose values have
	// changed AND have been notified about it) to check if the Config Field
	// - of Event_State message - is new to the drone. That would mean the
	// new config set has succeeded
	for (size_t i = 0; i < evt->state->n_selected_fields; i++) {
		Arsdk__Camera__Event__State__SelectedFieldsEntry *entry =
			evt->state->selected_fields[i];

		switch (entry->key) {
		case ARSDK__CAMERA__EVENT__STATE__CONFIG__FIELD_NUMBER:
			react_to_event_state_config_field_number(evt, mctrl);
			break;
		default:
			break;
		}
	}
}

static void react_to_event_state_config_field_number(
	Arsdk__Camera__Event *evt,
	struct mission_controller *mctrl)
{
	// scroll over the selected fields (meaning the fields whose values have
	// changed AND have been notified about it) to check if the CameraMode
	// Field - of Config message - is new to the drone. That would mean the
	// new camera mode set has succeeded
	for (size_t i = 0; i < evt->state->config->n_selected_fields; i++) {
		Arsdk__Camera__Config__SelectedFieldsEntry *entry =
			evt->state->config->selected_fields[i];

		switch (entry->key) {
		case ARSDK__CAMERA__CONFIG__CAMERA_MODE__FIELD_NUMBER:
			react_in_sm_to_camera_mode(
				evt->state->config->camera_mode, mctrl);
			break;
		default:
			break;
		}
	}
}

static void react_in_sm_to_camera_mode(Arsdk__Camera__CameraMode mode,
				       struct mission_controller *mctrl)
{
	if ((mode == ARSDK__CAMERA__CAMERA_MODE__CAMERA_MODE_PHOTO)
	    && (mctrl->video_photo_current_state == PHOTO_CONFIG_DONE)) {
		// if one has received from the drone that a photo config set up
		// command has been sent + the current state noticed that one
		// has sent a photo config set up, then one asks to shoot a
		// photo
		mctrl->video_photo_current_state = PHOTO_SHOOT_DONE;
		cmd_fcam_start_photo(mctrl);
	} else if ((mode == ARSDK__CAMERA__CAMERA_MODE__CAMERA_MODE_RECORDING)
		   && (mctrl->video_photo_current_state
		       == RECORDING_CONFIG_DONE)) {
		// if one has received from the drone that a recording config
		// set up command has been sent + the current state noticed that
		// one has sent a recording config set up, then one asks to
		// start recording
		mctrl->video_photo_current_state = RECORDING_STARTED;
		cmd_fcam_start_recording(mctrl);
	}
}

static void react_in_sm_to_event_photo(Arsdk__Camera__Event *evt,
				       struct mission_controller *mctrl)
{
	if ((evt->photo->type == ARSDK__CAMERA__PHOTO_EVENT__PHOTO_EVENT_STOP)
	    && (mctrl->video_photo_current_state == PHOTO_SHOOT_DONE)) {
		// if one has received from the drone that a photo shoot command
		// has been sent + the current state noticed that a photo has
		// been shot, then the mission is over
	}
}

static void react_in_sm_to_event_recording(Arsdk__Camera__Event *evt,
					   struct mission_controller *mctrl)
{
	if ((evt->recording->type
	     == ARSDK__CAMERA__RECORDING_EVENT__RECORDING_EVENT_START)
	    && (mctrl->video_photo_current_state == RECORDING_STARTED)) {
		// if one has received from the drone that a video recording
		// start command has been sent + the current state noticed that
		// one has started a recording, then one asks to stop recording
		mctrl->video_photo_current_state = RECORDING_STOPPED;
		cmd_fcam_stop_recording(mctrl);
	} else if ((evt->recording->type
		    == ARSDK__CAMERA__RECORDING_EVENT__RECORDING_EVENT_STOP)
		   && (mctrl->video_photo_current_state == RECORDING_STOPPED)) {
		// if one has received from the drone that a video recording
		// stop command has been sent + the current state noticed that
		// one has stopped the video recording, then one asks to set up
		// photo config to switch to photo mode
		mctrl->video_photo_current_state = PHOTO_CONFIG_DONE;
		cmd_fcam_set_config_photo(mctrl);
	}
}
