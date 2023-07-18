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

#include <futils/mbox.h>
#include <futils/timetools.h>

#include "tracking_object_class.hpp"
#include "tracking_telemetry.hpp"

#define ULOG_TAG telemetry_tracking
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

using namespace trackingExample::mbox;

static std::string objectClassToString(ObjectClass id)
{
	std::string objectName;

	switch (id) {
	case OC_ANIMAL:
		objectName = "Animal";
		break;
	case OC_BICYCLE:
		objectName = "Bicycle";
		break;
	case OC_BOAT:
		objectName = "Boat";
		break;
	case OC_CAR:
		objectName = "Car";
		break;
	case OC_HORSE:
		objectName = "Horse";
		break;
	case OC_MOTORBIKE:
		objectName = "MotorBike";
		break;
	case OC_PERSON:
		objectName = "Person";
		break;
	case OC_UNDEFINED:
		objectName = "Undefined";
		break;
	default:
		objectName = "ERROR";
		ULOGI("class id value=%d", id);
		break;
	}

	return objectName;
}

int TrackingTelemetry::registerCvTrackingProposalsConsumer()
{
	int i;
	int res = 0;
	std::string telemetrySectionName;
	std::string telemetryValueName;

	if (!mCvTrackingProposalsConsumer)
		return EINVAL;

	telemetrySectionName =
		std::string(CV_TRACKING_PROPOSALS_TELEMETRY_SECTION) + ".";

	telemetryValueName = "count";
	res = mCvTrackingProposalsConsumer->reg(
		mCvTrackingProposalsData.count,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingProposalsConsumer register count failed");
		return res;
	}

	for (i = 0; i < TRACKING_BOX_SIZE; i++) {
		telemetryValueName = std::string("box[") + std::to_string(i) +
				     std::string("].x");
		res = mCvTrackingProposalsConsumer->reg(
			mCvTrackingProposalsData.box[i].x,
			(telemetrySectionName + telemetryValueName).c_str(),
			&mCvTrackingProposalsData.ts);
		if (res < 0) {
			ULOGE("CvTrackingProposalsConsumer register %s failed",
			      telemetryValueName.c_str());
			return res;
		}

		telemetryValueName = std::string("box[") + std::to_string(i) +
				     std::string("].y");
		res = mCvTrackingProposalsConsumer->reg(
			mCvTrackingProposalsData.box[i].y,
			(telemetrySectionName + telemetryValueName).c_str());
		if (res < 0) {
			ULOGE("CvTrackingProposalsConsumer register %s failed",
			      telemetryValueName.c_str());
			return res;
		}

		telemetryValueName = std::string("box[") + std::to_string(i) +
				     std::string("].width");
		res = mCvTrackingProposalsConsumer->reg(
			mCvTrackingProposalsData.box[i].width,
			(telemetrySectionName + telemetryValueName).c_str());
		if (res < 0) {
			ULOGE("CvTrackingProposalsConsumer register %s failed",
			      telemetryValueName.c_str());
			return res;
		}

		telemetryValueName = std::string("box[") + std::to_string(i) +
				     std::string("].height");
		res = mCvTrackingProposalsConsumer->reg(
			mCvTrackingProposalsData.box[i].height,
			(telemetrySectionName + telemetryValueName).c_str());
		if (res < 0) {
			ULOGE("CvTrackingProposalsConsumer register %s failed",
			      telemetryValueName.c_str());
			return res;
		}

		telemetryValueName = std::string("box[") + std::to_string(i) +
				     std::string("].class_id");
		res = mCvTrackingProposalsConsumer->reg(
			mCvTrackingProposalsData.box[i].classId,
			(telemetrySectionName + telemetryValueName).c_str());
		if (res < 0) {
			ULOGE("CvTrackingProposalsConsumer register %s failed",
			      telemetryValueName.c_str());
			return res;
		}

		telemetryValueName = std::string("box[") + std::to_string(i) +
				     std::string("].confidence");
		res = mCvTrackingProposalsConsumer->reg(
			mCvTrackingProposalsData.box[i].condidence,
			(telemetrySectionName + telemetryValueName).c_str());
		if (res < 0) {
			ULOGE("CvTrackingProposalsConsumer register %s failed",
			      telemetryValueName.c_str());
			return res;
		}

		telemetryValueName = std::string("box[") + std::to_string(i) +
				     std::string("].track_id");
		res = mCvTrackingProposalsConsumer->reg(
			mCvTrackingProposalsData.box[i].trackId,
			(telemetrySectionName + telemetryValueName).c_str());
		if (res < 0) {
			ULOGE("CvTrackingProposalsConsumer register %s failed",
			      telemetryValueName.c_str());
			return res;
		}
	}

	res = mCvTrackingProposalsConsumer->regComplete();
	if (res < 0) {
		ULOGE("CvTrackingProposalsConsumer register complete failed");
		return res;
	}

	return res;
}

void TrackingTelemetry::logCvTrackingProposalsData()
{
	uint32_t i;

	ULOGI("> Cv Tracking Proposals Data");
	ULOGI("> count : %d", mCvTrackingProposalsData.count);
	for (i = 0; i < mCvTrackingProposalsData.count; i++) {
		ULOGI("> Box[%d]: x=%f  y=%f  width=%f  height=%f  class_id=(%d)[%s]  confidence=%f  track_id=%d",
		      i,
		      mCvTrackingProposalsData.box[i].x,
		      mCvTrackingProposalsData.box[i].y,
		      mCvTrackingProposalsData.box[i].width,
		      mCvTrackingProposalsData.box[i].height,
		      mCvTrackingProposalsData.box[i].classId,
		      objectClassToString(
			      static_cast<ObjectClass>(
				      mCvTrackingProposalsData.box[i].classId))
			      .c_str(),
		      mCvTrackingProposalsData.box[i].condidence,
		      mCvTrackingProposalsData.box[i].trackId);
	}
}

int TrackingTelemetry::registerCvTrackingBoxConsumer()
{
	int res = 0;
	std::string telemetrySectionName;
	std::string telemetryValueName;

	if (!mCvTrackingBoxConsumer)
		return EINVAL;

	telemetrySectionName =
		std::string(CV_TRACKING_BOX_TELEMETRY_SECTION) + ".";

	telemetryValueName = std::string("status");
	mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.status,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	telemetryValueName = std::string("box.x");
	res = mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.box.x,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	telemetryValueName = std::string("box.y");
	res = mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.box.y,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	telemetryValueName = std::string("box.width");
	res = mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.box.width,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	telemetryValueName = std::string("box.height");
	res = mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.box.height,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	telemetryValueName = std::string("box.class_id");
	res = mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.box.classId,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	telemetryValueName = std::string("box.confidence");
	res = mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.box.condidence,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	telemetryValueName = std::string("box.track_id");
	res = mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.box.trackId,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	telemetryValueName = std::string("tag_id");
	mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.tagId,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	telemetryValueName = std::string("quality");
	mCvTrackingBoxConsumer->reg(
		mCvTrackingBoxData.quality,
		(telemetrySectionName + telemetryValueName).c_str());
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register %s failed",
		      telemetryValueName.c_str());
		return res;
	}

	mCvTrackingBoxConsumer->regComplete();
	if (res < 0) {
		ULOGE("CvTrackingBoxConsumer register complete failed");
		return res;
	}

	return res;
}

void TrackingTelemetry::logCvTrackingBoxData()
{
	ULOGI("> Cv Tracking Box Data");
	ULOGI("> status : %d", mCvTrackingBoxData.status);

	ULOGI("> Box: x=%f  y=%f  width=%f  height=%f  class_id=(%d)[%s]  confidence=%f  track_id=%d",
	      mCvTrackingBoxData.box.x,
	      mCvTrackingBoxData.box.y,
	      mCvTrackingBoxData.box.width,
	      mCvTrackingBoxData.box.height,
	      mCvTrackingBoxData.box.classId,
	      objectClassToString(
		      static_cast<ObjectClass>(mCvTrackingBoxData.box.classId))
		      .c_str(),
	      mCvTrackingBoxData.box.condidence,
	      mCvTrackingBoxData.box.trackId);

	ULOGI("> tag_id : %d", mCvTrackingBoxData.tagId);
	ULOGI("> quality : %d", mCvTrackingBoxData.quality);
}

int TrackingTelemetry::sendTrackId()
{
	uint64_t tsUs;
	stateMachine::msg sendMsg;
	int res = 0;

	if (mCvTrackingProposalsData.count > 0) {
		time_timespec_to_us(&mCvTrackingProposalsData.ts, &tsUs);

		sendMsg.type = stateMachine::MsgType::TELEMETRY_SEND_TRACK_ID;
		sendMsg.telemetrySetTargetMsg.track_id =
			mCvTrackingProposalsData.box[0].trackId;
		sendMsg.telemetrySetTargetMsg.timespec_us = tsUs;

		res = mbox_push(mMboxStateMachine, &sendMsg);
		if (res < 0)
			ULOG_ERRNO("mbox_push", -res);
	} else {
		res = -1;
	}

	return res;
}

TrackingTelemetry::TrackingTelemetry(pomp::Loop &loop, mbox *mboxStateMachine) :
		mLoop(loop), mMboxStateMachine(mboxStateMachine),
		mTimer(&mLoop, this)
{
	mCvTrackingProposalsConsumer = telemetry::Consumer::create();
	mCvTrackingBoxConsumer = telemetry::Consumer::create();

	mTargetTracked = false;
}

TrackingTelemetry::~TrackingTelemetry()
{
	if (mCvTrackingProposalsConsumer) {
		telemetry::Consumer::release(mCvTrackingProposalsConsumer);
		mCvTrackingProposalsConsumer = nullptr;
	}

	if (mCvTrackingBoxConsumer) {
		telemetry::Consumer::release(mCvTrackingBoxConsumer);
		mCvTrackingBoxConsumer = nullptr;
	}
}

int TrackingTelemetry::initConsumer()
{
	int res;

	res = registerCvTrackingProposalsConsumer();
	if (res < 0) {
		ULOG_ERRNO("registerCvTrackingProposalsConsumer", -res);
		return res;
	}

	res = registerCvTrackingBoxConsumer();
	if (res < 0) {
		ULOG_ERRNO("registerCvTrackingBoxConsumer", -res);
		return res;
	}

	return res;
}

void TrackingTelemetry::startt()
{
	mTimer.setPeriodic(1000, 1000);
}

void TrackingTelemetry::stopt()
{
	mTimer.clear();
}

void TrackingTelemetry::resetTargetTracked()
{
	mTargetTracked = false;
}

void TrackingTelemetry::processTimer()
{
	if (!mCvTrackingProposalsConsumer->getSample(
		    nullptr, telemetry::Method::TLM_LATEST)) {
		ULOGI("Unable to read cv tracking proposals telemetry sample.");
	} else {
		logCvTrackingProposalsData();
	}

	if (!mCvTrackingBoxConsumer->getSample(nullptr,
					       telemetry::Method::TLM_LATEST)) {
		ULOGI("Unable to read cv tracking box telemetry sample.");
	} else {
		logCvTrackingBoxData();
	}

	if (!mTargetTracked && sendTrackId() == 0) {
		mTargetTracked = true;
	}
}