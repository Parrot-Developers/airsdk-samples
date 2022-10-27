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

#include "road_following_configuration.hpp"
#include "road_following_plugin.hpp"

using RoadFollowingEventSender =
	::road_runner::guidance::road_following::messages::msghub::EventSender;

class RoadFollowing : public guidance::Mode {
private:
	/* Message Sender */
	RoadFollowingEventSender mEventSender;

	/* Road Following guidance mode configuration Object */
	RoadFollowingConfiguration mConfiguration;

	/* Telemetry consumer */
	telemetry::Consumer *mTelemetryDroneConsumer;
	telemetry::Consumer *mTelemetryServiceConsumer;

	/* Drone estimated telemetry */
	float mDroneYaw;

	/* Service estimated telemetry */
	Eigen::Vector3f mHorizontalVelocityEst;
	Eigen::Vector3f mVelocityEst;
	float mYawVelocityEst;

	/* Watchdog service */
	timespec mTsServiceCons;

public:
	/**
	 * Constructor
	 * @param guidance Guidance object
	 */
	RoadFollowing(guidance::Guidance *guidance);

	/**
	 * Destructor
	 */
	~RoadFollowing();

	/* Overriden guidance mode functions */
	virtual bool hasObstacleAvoidance() override;
	virtual void getTriggers(uint32_t *triggers,
				 uint32_t *timeout,
				 uint32_t *period) override;
	void configure(const ::google::protobuf::Any &config,
		       bool disableObstacleAvoidance,
		       bool overrideFrontCamera,
		       bool overrideStereoCamera) override;
	virtual const std::string &getName() const override;
	virtual void enter() override;
	virtual void beginStep() override;
	virtual void generateDroneReference() override;
	virtual void generateAttitudeReferences() override;
	virtual void endStep() override;
	virtual void exit() override;
};
