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

#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include "processing.hpp"

#define ULOG_TAG processing
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

#define CFG_CHECK(E) ULOG_ERRNO_RETURN_ERR_IF(E < 0, EINVAL)

namespace cfgreader {
template <>
int SettingReader<struct roadFollowingCfg>::read(const libconfig::Setting &set,
						 T &v)
{
	const char *str = "droneAltitude";
	CFG_CHECK(ConfigReader::getField(set, str, v.droneAltitude));

	str = "xVelocity";
	CFG_CHECK(ConfigReader::getField(set, str, v.xVelocity));

	str = "xVelocityRoadLost";
	CFG_CHECK(ConfigReader::getField(set, str, v.xVelocityRoadLost));

	str = "yVelocityCoefficient";
	CFG_CHECK(ConfigReader::getField(set, str, v.yVelocityCoefficient));

	str = "yawVelocityCoefficient";
	CFG_CHECK(ConfigReader::getField(set, str, v.yawVelocityCoefficient));

	str = "lostRoadTimeLimit";
	CFG_CHECK(ConfigReader::getField(set, str, v.lostRoadTimeLimit));

	str = "telemetryProducerSection";
	CFG_CHECK(ConfigReader::getField(set, str, v.telemetryProducerSection));

	str = "telemetryProducerSectionRate";
	CFG_CHECK(ConfigReader::getField(
		set, str, v.telemetryProducerSectionRate));

	str = "telemetryProducerSectionCount";
	CFG_CHECK(ConfigReader::getField(
		set, str, v.telemetryProducerSectionCount));

	return 0;
}
} // namespace cfgreader

static void do_step(const struct vipc_frame *frame,
		    struct roadData *road_data,
		    bool &is_road_detected)
{
	cv::Mat frame_ref;
	cv::Mat frame_grey;
	cv::Mat frame_hsv;
	cv::Mat frame_mask_road_line;
	cv::Mat frame_mask_final;
	cv::Mat frame_blur;
	cv::Mat frame_cany;

	cv::Point ini;
	cv::Point fini;
	std::vector<cv::Point> line_pts;
	cv::Point line_p;
	double line_m; // y = m*x + p

	int middle_y;

	/* Vector of lines. Each line is represented by a 4-element vector
	(x_1, y_1, x_2, y_2) , where (x_1,y_1) and (x_2, y_2) are the ending
	points of each detected line segment. */
	std::vector<cv::Vec4i> lines;

	/* line parameters. vector of 4 elements (like Vec4f) - (vx, vy, x0,
	y0), where (vx, vy) is a normalized vector collinear to the line and
	(x0, y0) is a point on the line. */
	cv::Vec4f line;

	cv::cvtColor(cv::Mat(frame->height * 3 / 2,
			     frame->width,
			     CV_8UC1,
			     (void *)frame->planes[0].virt_addr),
		     frame_ref,
		     cv::COLOR_YUV2BGR_NV21,
		     3);

	cv::cvtColor(frame_ref, frame_grey, cv::COLOR_RGB2GRAY);
	cv::cvtColor(frame_ref, frame_hsv, cv::COLOR_BGR2HSV);

	cv::inRange(frame_hsv,
		    cv::Scalar(18, 46, 233),
		    cv::Scalar(26, 91, 255),
		    frame_mask_road_line);

	cv::bitwise_and(frame_grey, frame_mask_road_line, frame_mask_final);
	cv::GaussianBlur(frame_mask_final, frame_blur, cv::Size(3, 3), 0);
	cv::Canny(frame_blur, frame_cany, 190, 200);
	cv::HoughLinesP(frame_cany, lines, 2, CV_PI / 180, 100, 40, 5);

	if (lines.size() > 0) {

		for (auto i : lines) {
			ini = cv::Point(i[0], i[1]);
			fini = cv::Point(i[2], i[3]);

			line_pts.push_back(ini);
			line_pts.push_back(fini);
		}

		cv::fitLine(line_pts, line, CV_DIST_L2, 0, 0.01, 0.01);

		line_m = line[1] / line[0];
		line_p = cv::Point(line[2], line[3]);

		middle_y = frame->height / 2;

		road_data->line_center_diff =
			frame->width / 2 -
			(((middle_y - line_p.y) / line_m) + line_p.x);
		road_data->line_leading_coeff = line_m;

		is_road_detected = true;
		line_pts.clear();
	} else {
		is_road_detected = false;
	}
}

ProcessingListener::ProcessingListener(void *userdata) : Listener(userdata) {}

int ProcessingListener::processingStep(void *userdata,
				       const struct vipc_frame *new_frame)
{
	Processing *processing = (Processing *)userdata;
	return processing->processingStep(new_frame);
}

void Processing::threadEntry()
{
	std::unique_lock<std::mutex> lk(mMutex);

	struct vipc_frame frame;

	while (!mStopRequested) {
		/* Atomically unlock the mutex, wait for condition and then
		  re-lock the mutex when condition is signaled */
		mCond.wait(lk);

		if (mFirstTime) {
			/* Save time */
			time_get_monotonic(&mSaveTime);
			mFirstTime = false;
		}

		if (mStopRequested) {
			mMutex.unlock();
			break;
		}
		if (!mFrameAvailable)
			continue;

		/* Copy locally input data */
		memcpy(&frame, mFrame, sizeof(frame));
		mFrameAvailable = false;

		/* Do the heavy computation outside lock */
		mMutex.unlock();
		do_step(mFrame, &mRoadData, mIsRoadDetected);
		mMutex.lock();

		mTelemetryConsumer->getSample(nullptr,
					      telemetry::Method::TLM_LATEST);

		computeAltitude();

		if (mIsRoadDetected) {
			computeTrajectory();
			mIsRoadDetected = false;
			time_get_monotonic(&mSaveTime);
		} else {
			computeTrajectoryRoadLost();
			time_timespec_diff_now(&mSaveTime, &mDiffTime);
			if ((uint64_t)mDiffTime.tv_sec >
			    (uint64_t)mRoadFollowingCfg.lostRoadTimeLimit) {
				/* Send road lost message */
				const ::google::protobuf::Empty message;
				this->roadLost(message);

				mFirstTime = true;
			}
		}

		/* Done with the input frame */
		vipcc_release_safe(mFrame);
	}
}

int Processing::loadRoadFollowingConfiguration(const std::string &configPath)
{
	cfgreader::FileConfigReader reader(configPath);
	int res = reader.load();
	if (res < 0) {
		ULOG_ERRNO("cannot load %s", -res, configPath.c_str());
		return res;
	}

	res = reader.get("road_following", mRoadFollowingCfg);
	if (res < 0) {
		ULOG_ERRNO("cannot read blockage detector config", -res);
		return res;
	}

	return 0;
}

Processing::Processing(pomp::Loop *loop) :
		mProcessingListener(this), mChannel(nullptr),
		mMessageHub(loop, this), mVideo(loop)
{
	int res;

	/* pomp loop object */
	mLoop = loop;

	/* Service context */
	mStopRequested = 0;
	mStarted = false;
	mIsRoadDetected = false;

	/* timer */
	mTimerHandler.set(std::bind(&Processing::produceTelemetry, this));

	/* Thread context */
	mThread = nullptr;

	/* Vipc context */
	mFrame = nullptr;
	mFrameAvailable = false;

	/* Timespec context */
	mFirstTime = true;
	mSaveTime = {0, 0};
	mDiffTime = {0, 0};

	res = loadRoadFollowingConfiguration(
		cfgreader::ConfigReader::insertMissionRootDir(
			ROAD_FOLLOWING_SERVICE_CONFIG_PATH));
	if (res < 0) {
		ULOG_ERRNO("loadRoadFollowingConfiguration", -res);
		std::bad_alloc ex;
		throw ex;
	}

	/* Telemetry start consumer */
	mTelemetryConsumer = telemetry::Consumer::create();
	res = mTelemetryConsumer->reg(mTlmAltitudeAgl,
				      "drone_controller.altitude_agl");
	if (res < 0) {
		ULOG_ERRNO("drone_controller.altitude_agl registration", -res);
		std::bad_alloc ex;
		throw ex;
	}

	res = mTelemetryConsumer->regComplete();
	if (res < 0) {
		ULOG_ERRNO("telemetry::Consumer::regComplete", -res);
		std::bad_alloc ex;
		throw ex;
	}

	/* Telemetry start producer */

	mTelemetryProducer = telemetry::Producer::create(
		mRoadFollowingCfg.telemetryProducerSection,
		mRoadFollowingCfg.telemetryProducerSectionCount,
		mRoadFollowingCfg.telemetryProducerSectionRate,
		nullptr,
		false);
	if (mTelemetryProducer == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("new Producer object", -res);
		std::bad_alloc ex;
		throw ex;
	}

	res = mTelemetryProducer->reg(mTlmXVelocity, "x_velocity");
	if (res != 0) {
		ULOG_ERRNO("failed to register x_velocity", -res);
		std::bad_alloc ex;
		throw ex;
	}

	res = mTelemetryProducer->reg(mTlmYVelocity, "y_velocity");
	if (res != 0) {
		ULOG_ERRNO("failed to register y_velocity", -res);
		std::bad_alloc ex;
		throw ex;
	}

	res = mTelemetryProducer->reg(mTlmZVelocity, "z_velocity");
	if (res != 0) {
		ULOG_ERRNO("failed to register z_velocity", -res);
		std::bad_alloc ex;
		throw ex;
	}

	res = mTelemetryProducer->reg(mTlmYawVelocity, "yaw_velocity");
	if (res != 0) {
		ULOG_ERRNO("failed to register yaw_velocity", -res);
		std::bad_alloc ex;
		throw ex;
	}

	res = mTelemetryProducer->regComplete();
	if (res < 0) {
		ULOG_ERRNO("telemetry::Producer::regComplete", -res);
		std::bad_alloc ex;
		throw ex;
	}
}

Processing::~Processing()
{
	stop();

	telemetry::Consumer::release(mTelemetryConsumer);
	telemetry::Producer::release(mTelemetryProducer);

	delete mThread;

	mThread = nullptr;
	mFrame = nullptr;
}

int Processing::start(void)
{
	int res = 0;

	/* Start Message Handler */
	mChannel = mMessageHub.startServerChannel(MSGHUB_ADDR, 0666);
	if (!mChannel) {
		ULOGC("failed to create server channel");
		res = -EPERM;
		goto out;
	}
	mMessageHub.enableDump();
	mMessageHub.attachMessageHandler(this);
	mMessageHub.attachMessageSender(this, mChannel);

	/* Create background thread */
	mMutex.lock();
	mStopRequested = 0;
	mMutex.unlock();

	mThread = new std::thread(&Processing::threadEntry, this);
	if (mThread == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("new Thread object", -res);
		goto out;
	}

	/* Set first telemetry values to send to RoadFollowing guidance mode */
	mTlmXVelocity = 0.f;
	mTlmYVelocity = 0.f;
	mTlmZVelocity = 0.f;
	mTlmYawVelocity = 0.f;
	mTelemetryProducer->putSample(nullptr);

	/* Init road data values */
	mRoadData.line_center_diff = 0;
	mRoadData.line_leading_coeff = 0;

	mTimer = new pomp::Timer(mLoop, &mTimerHandler);
	mTimer->setPeriodic(mRoadFollowingCfg.telemetryProducerSectionRate,
			    mRoadFollowingCfg.telemetryProducerSectionRate);

	mStarted = true;

out:
	return res;
}

void Processing::stop()
{
	/* Ask thread to stop */
	mMutex.lock();
	mStopRequested = 1;
	mCond.notify_one();
	mMutex.unlock();

	/* Wait for thread and release resources */
	mThread->join();
	mStarted = false;

	/* Cleanup remaining input data if any */
	mMutex.lock();
	if (mFrameAvailable) {
		vipcc_release_safe(mFrame);
		mFrameAvailable = false;
	}
	mMutex.unlock();

	/* Stop timer */
	if (mTimer) {
		delete mTimer;
		mTimer = nullptr;
	}

	/* Stop Message Handler */
	mMessageHub.stop();
	mMessageHub.detachMessageHandler(this);
	mMessageHub.detachMessageSender(this);
	mChannel = nullptr;
}

int Processing::processingStep(const struct vipc_frame *new_frame)
{
	ULOG_ERRNO_RETURN_ERR_IF(new_frame == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPERM);

	mMutex.lock();

	/* If an input is already pending, release it before overwrite */
	if (mFrameAvailable) {
		vipcc_release_safe(mFrame);
		mFrameAvailable = false;
	}

	/* Copy input data and take ownership of frame */
	mFrame = new_frame;
	mFrameAvailable = true;

	/* Wakeup background thread */
	mCond.notify_one();

	mMutex.unlock();

	return 0;
}

void Processing::computeAltitude()
{
	mTlmZVelocity = -(mRoadFollowingCfg.droneAltitude - mTlmAltitudeAgl);
}

void Processing::computeTrajectory()
{
	mTlmXVelocity = mRoadFollowingCfg.xVelocity;
	mTlmYVelocity = -mRoadData.line_center_diff *
			mRoadFollowingCfg.yVelocityCoefficient;

	if (mRoadData.line_leading_coeff != 0.f)
		mTlmYawVelocity = -(1 / mRoadData.line_leading_coeff) /
				  mRoadFollowingCfg.yawVelocityCoefficient;
	else
		mTlmYawVelocity = 0;
}

void Processing::computeTrajectoryRoadLost()
{
	mTlmXVelocity = mRoadFollowingCfg.xVelocityRoadLost;
	mTlmYVelocity = 0.0;
	mTlmYawVelocity = 0.0;
}

void Processing::produceTelemetry(void)
{
	mTelemetryProducer->putSample(nullptr);
}

void Processing::onConnected(::msghub::Channel *channel, pomp::Connection *conn)
{
	ULOGN("connected to %s", MSGHUB_ADDR);
};

void Processing::onDisconnected(::msghub::Channel *channel,
				pomp::Connection *conn)
{
	ULOGN("disconnected to %s", MSGHUB_ADDR);
};

void Processing::enableCv(const bool msg)
{
	if (msg)
		mVideo.vipcStart(&mProcessingListener);
	else
		mVideo.vipcStop();
};
