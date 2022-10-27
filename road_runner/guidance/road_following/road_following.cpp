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

#define ULOG_TAG gdnc_road_following
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG)

#include "road_following.hpp"

static const std::string ROAD_FOLLOWING_MODE_NAME =
	"com.parrot.missions.samples.road_runner.road_following";

static const std::string ROAD_FOLLOWING_CONFIG_PATH =
	"/etc/guidance/road_following/mode.cfg";

RoadFollowing::RoadFollowing(guidance::Guidance *guidance) : Mode(guidance)
{
	int res;

	/* read configuration */
	res = mConfiguration.read(
		guidance->getConfigFile(ROAD_FOLLOWING_CONFIG_PATH));
	if (res < 0) {
		ULOG_ERRNO("Guidance::getConfigFile", res);
		goto out;
	}

	// https://developer.parrot.com/docs/airsdk/telemetry/api_telemetry.html
	mTelemetryServiceConsumer = telemetry::Consumer::create();
	if (mTelemetryServiceConsumer == nullptr) {
		ULOGE("Could not create telemetry service consumer");
		res = -1;
		goto out;
	}
	mTelemetryDroneConsumer = telemetry::Consumer::create();
	if (mTelemetryDroneConsumer == nullptr) {
		ULOGE("Could not create telemetry drone consumer");
		res = -1;
		goto out;
	}
	// clang-format off
	mTelemetryDroneConsumer->reg(mDroneYaw,
		"drone_controller.attitude_euler_angles.yaw");

	/* road estimation telemetry */
	mTelemetryServiceConsumer->reg(mVelocityEst.x(),
		"road_estimation.x_velocity", &mTsServiceCons);
	mTelemetryServiceConsumer->reg(mVelocityEst.y(),
		"road_estimation.y_velocity");
	mTelemetryServiceConsumer->reg(mVelocityEst.z(),
		"road_estimation.z_velocity");
	mTelemetryServiceConsumer->reg(mYawVelocityEst,
		"road_estimation.yaw_velocity");
	// clang-format on

	mTelemetryDroneConsumer->regComplete();
	mTelemetryServiceConsumer->regComplete();

	/* Init drone estimated telemetry */
	mDroneYaw = 0.f;

	/* Init service estimated telemetry */
	mHorizontalVelocityEst = Eigen::Vector3f::Zero();
	mVelocityEst = Eigen::Vector3f::Zero();
	mYawVelocityEst = 0.f;


out:
	if (res < 0)
		mIsCreated = false;
	else
		mIsCreated = true;
}

RoadFollowing::~RoadFollowing()
{
	telemetry::Consumer::release(mTelemetryServiceConsumer);
	telemetry::Consumer::release(mTelemetryDroneConsumer);
}

const std::string &RoadFollowing::getName() const
{
	return ROAD_FOLLOWING_MODE_NAME;
}

bool RoadFollowing::hasObstacleAvoidance()
{
	return false;
}

void RoadFollowing::getTriggers(uint32_t *triggers,
				uint32_t *timeout,
				uint32_t *period)
{
	*triggers = guidance::TRIGGER_TICK;
	*timeout = 0;
	*period = mConfiguration.tickPeriod;
}

void RoadFollowing::configure(const ::google::protobuf::Any &config,
			      bool disableObstacleAvoidance,
			      bool overrideFrontCamera,
			      bool overrideStereoCamera)
{
	guidance::Output *output = getOutput();

	// https://developer.parrot.com/docs/airsdk/general/guidance_api.html#_CPPv4N8guidance6Output23FrontCamReferenceConfigE
	output->mHasFrontCamReferenceConfig = true;

	auto frontPitchConfig =
		output->mFrontCamReferenceConfig.mutable_pitch();
	frontPitchConfig->set_locked(true);
	frontPitchConfig->set_filtered(true);

	auto frontRollConfig = output->mFrontCamReferenceConfig.mutable_roll();
	frontRollConfig->set_locked(false);

	auto frontYawConfig = output->mFrontCamReferenceConfig.mutable_yaw();
	frontYawConfig->set_locked(true);
	frontYawConfig->set_filtered(true);
}

void RoadFollowing::enter()
{
	::msghub::Channel *channel =
		mGuidance->getChannel(guidance::CHANNEL_KIND_GUIDANCE);
	mGuidance->getMessageHub()->attachMessageSender(&mEventSender, channel);

	/* Send message to enable cv_service */
	const ::google::protobuf::Empty message;
	mEventSender.roadFollowingEnabled(message);
}

void RoadFollowing::beginStep()
{
	timespec now;
	timespec diffTime;

	/* Update telemetry data */
	time_get_monotonic(&now);
	mTelemetryServiceConsumer->getSample(
		&now, telemetry::Method::TLM_FIRST_BEFORE);
	mTelemetryDroneConsumer->getSample(nullptr,
					   telemetry::Method::TLM_LATEST);

	time_timespec_diff(&mTsServiceCons, &now, &diffTime);
	if (!mTsServiceCons.tv_sec ||
	    diffTime.tv_sec > mConfiguration.missingTelemetryValuesLimit) {
		const ::google::protobuf::Empty message;
		mEventSender.telemetryMissedTooLong(message);
	}
}

void RoadFollowing::generateDroneReference()
{

	guidance::Output *output = getOutput();

	output->mHasHorizontalReference = true;

	mHorizontalVelocityEst = physics::horizontalToNed3(
		Eigen::Vector3f(mVelocityEst.x(), mVelocityEst.y(), 0.f),
		mDroneYaw);

	// https://developer.parrot.com/docs/airsdk/messages/messages_list.html#_CPPv4N15DroneController8Messages19HorizontalReferenceE
	auto horizontalRef = output->mHorizontalReference.mutable_velocity();
	horizontalRef->mutable_ref()->mutable_x()->set_x(
		mHorizontalVelocityEst.x());
	horizontalRef->mutable_ref()->mutable_y()->set_x(
		mHorizontalVelocityEst.y());
	horizontalRef->set_config(
		::ColibryLite::Messages::HorizontalControlConfig::DEFAULT);
	horizontalRef->set_controller_reactivity(
		::ColibryLite::Messages::HorizontalControllerReactivity::
			DEFAULT);

	// https://developer.parrot.com/docs/airsdk/messages/messages_list.html#_CPPv4N15DroneController8Messages17VerticalReferenceE
	output->mHasVerticalReference = true;
	auto verticalRef = output->mVerticalReference.mutable_velocity();
	verticalRef->set_ref(mVelocityEst.z());
	verticalRef->set_config(
		::ColibryLite::Messages::VerticalControlConfig::DEFAULT);
	verticalRef->set_controller_setting(
		::ColibryLite::Messages::VerticalControllerSetting::DEFAULT);
	verticalRef->set_ground_constrained(true);
}

void RoadFollowing::generateAttitudeReferences()
{
	using AxisRef_t = CamController::Messages::AxisReference;
	guidance::Output *output = getOutput();

	// https://developer.parrot.com/docs/airsdk/messages/messages_list.html#_CPPv4N15DroneController8Messages12YawReferenceE
	output->mHasYawReference = true;
	auto yawRate = output->mYawReference.mutable_rate();
	yawRate->set_config(ColibryLite::Messages::YawControlConfig::DEFAULT);
	yawRate->set_ref(mYawVelocityEst);

	output->mHasStereoCamReference = false;

	// https://developer.parrot.com/docs/airsdk/general/guidance_api.html#_CPPv4N8guidance6Output17FrontCamReferenceE
	output->mHasFrontCamReference = true;
	AxisRef_t *fcamPitchRef = output->mFrontCamReference.mutable_pitch();
	fcamPitchRef->set_ctrl_mode(
		CamController::Messages::ControlMode::POSITION);
	fcamPitchRef->set_frame_of_ref(
		CamController::Messages::FrameOfReference::NED_START);
	fcamPitchRef->set_position(mConfiguration.cameraPitchPosition * M_PI /
				   180.f);

	AxisRef_t *fcamYawRef = output->mFrontCamReference.mutable_yaw();
	fcamYawRef->set_ctrl_mode(
		CamController::Messages::ControlMode::POSITION);
	fcamYawRef->set_frame_of_ref(
		CamController::Messages::FrameOfReference::NED_START);
	fcamYawRef->set_position(mDroneYaw);
}

void RoadFollowing::endStep()
{
	/* Unused in this mode. Not mandatory. */
}

void RoadFollowing::exit()
{
	/* Send message to disable cv_service */
	const ::google::protobuf::Empty message;
	mEventSender.roadFollowingDisabled(message);

	mGuidance->getMessageHub()->detachMessageSender(&mEventSender);
}