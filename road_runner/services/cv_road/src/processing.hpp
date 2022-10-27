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

#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>
#include <csignal>
#include <unistd.h>

#include <cfgreader/cfgreader.hpp>
#include <futils/timetools.h>
#include <libpomp.hpp>
#include <libtelemetry.hpp>
#include <video-ipc/vipc_client.h>

#include "listener.hpp"
#include "video.hpp"

/* Messages exchanged with Flight Supervisor */
#include <road_runner/cv_road/messages.msghub.h>
#include <road_runner/cv_road/messages.pb.h>

#define MSGHUB_ADDR "unix:/tmp/road-runner-cv-road-service"

static const std::string ROAD_FOLLOWING_SERVICE_CONFIG_PATH =
	"/etc/services/road_following.cfg";

/* Configuration values */
struct roadFollowingCfg {
	float droneAltitude;
	float xVelocity;
	float xVelocityRoadLost;
	float yVelocityCoefficient;
	float yawVelocityCoefficient;
	int lostRoadTimeLimit;
	std::string telemetryProducerSection;
	int telemetryProducerSectionRate;
	int telemetryProducerSectionCount;
};

/* Values to send to RoadFollowing guidance mode */
struct roadData {
	int line_center_diff;
	double line_leading_coeff;
};

class ProcessingListener : public Listener {
public:
	/* Constructor */
	ProcessingListener(void *userdata);
	/* Destructor */
	~ProcessingListener() = default;
	/* processingStep callback */
	int processingStep(void *userdata,
			   const struct vipc_frame *new_frame) override;
};

class Processing : public ::road_runner::service::cv_road::messages::msghub::
			   CommandHandler,
		   public ::road_runner::service::cv_road::messages::msghub::
			   EventSender,
		   public ::msghub::MessageHub::ConnectionHandler {
private:
	/* pomp loop object */
	pomp::Loop *mLoop;

	/* Service context */
	sig_atomic_t mStopRequested;
	bool mStarted;
	bool mIsRoadDetected;

	/* Road Following configuration Object */
	struct roadFollowingCfg mRoadFollowingCfg;

	/* Values to send to RoadFollowing guidance mode */
	struct roadData mRoadData;

	/* timer */
	pomp::Timer::HandlerFunc mTimerHandler;
	pomp::Timer *mTimer;

	/* Thread context */
	std::thread *mThread;
	std::mutex mMutex;
	std::condition_variable mCond;

	/* Vipc context */
	const struct vipc_frame *mFrame;
	bool mFrameAvailable;

	/* Timespec context */
	bool mFirstTime;
	struct timespec mSaveTime;
	struct timespec mDiffTime;

	/* Listener Object */
	ProcessingListener mProcessingListener;

	/* Msghub objects */
	msghub::Channel *mChannel;
	msghub::MessageHub mMessageHub;

	/* Video Object */
	Video mVideo;

	/* Telemetry */
	telemetry::Consumer *mTelemetryConsumer;
	telemetry::Producer *mTelemetryProducer;

	float mTlmAltitudeAgl;

	float mTlmXVelocity;
	float mTlmYVelocity;
	float mTlmZVelocity;
	float mTlmYawVelocity;

private:
	/* Thread function */
	void threadEntry();

	/**
	 * Load the configuration of the service
	 *
	 * @param configPath path of the config file.
	 * @return 0 in case of success, negative errno in case of error.
	 */
	int loadRoadFollowingConfiguration(const std::string &configPath);

public:
	/**
	 * Constructor
	 * @param loop pomp loop object
	 */
	Processing(pomp::Loop *loop);

	/**
	 * Destructor
	 */
	~Processing();

	/**
	 * Start processing.
	 * @return  0 in case of success, negative errno in case of error.
	 */
	int start(void);

	/**
	 * Stop processing.
	 */
	void stop(void);

	/**
	 * Processing video step
	 * @param new_frame vipc_frame object.
	 */
	int processingStep(const struct vipc_frame *new_frame);

	/**
	 * Set Z axes values
	 */
	void computeAltitude(void);

	/**
	 * Set trajectory value when the road is detected
	 *
	 * X, Y and Yaw axes
	 */
	void computeTrajectory(void);

	/**
	 * Set trajectory value when the road is not detected
	 *
	 * X, Y and Yaw axes
	 */
	void computeTrajectoryRoadLost(void);

	/* --- Telemetry --- */

	/**
	 * produce telemetry. Works with a timer.
	 */
	void produceTelemetry(void);

	/* --- Msghub --- */

	/* ConnectionHandler overridden functions */
	virtual void onConnected(::msghub::Channel *channel,
				 pomp::Connection *conn) override;
	virtual void onDisconnected(::msghub::Channel *channel,
				    pomp::Connection *conn) override;

	/**
	 * Override enableCv function (Handler).
	 *
	 * True received  : Road video processing need to be activated
	 * False received : Road video processing need to be disabled
	 *
	 * @param msg bool
	 */
	virtual void enableCv(const bool msg) override;
};
