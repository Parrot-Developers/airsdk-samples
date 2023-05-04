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

#include <airsdk/airsdk.hpp>

#include "configuration.hpp"

enum orientation_mode {
	NONE = 0,  // The drone won’t change its orientation
	TO_TARGET, // The drone will make a rotation to look in direction of the
		   // given location
	HEADING_START, // The drone will orientate itself to the given heading
		       // before moving to the location
	HEADING_DURING // The drone will orientate itself to the given heading
		       // while moving to the location
};

struct absoluteMove {
	double latitude;
	double longitude;
	double altitude;
};

struct relativeMove {
	float dx;
	float dy;
	float dz;
};

struct trajectory {
	absoluteMove absTarget;
	relativeMove relTarget;
	orientation_mode orientMode;
	float heading;
};

class ControlInterface {
private:
	airsdk::control::ControlInterface mControlItf;

	// To handle default velocities, that may be seen as config variables
	MissionConfiguration *mMissionConfiguration;

	// Relative and absolute flighplan can be built the same way. We will
	// only detail relative flightplan then
	std::vector<trajectory> mRelativeTrajectory;
	uint8_t mMoveIndex;

	bool mFirstTimeHovering;

public:
	ControlInterface(pomp::Loop &loop, MissionConfiguration *config);
	~ControlInterface() = default;

	int start();

	int cmdMoveTo(absoluteMove target,
		      orientation_mode orientMode,
		      float heading,
		      float maxHorSpeed,
		      float maxVertSpeed,
		      float maxYawSpeed);
	int cmdMoveTo(absoluteMove target,
		      orientation_mode orientMode,
		      float heading)
	{
		return cmdMoveTo(
			target,
			orientMode,
			heading,
			this->mMissionConfiguration->mVelocityCfg
				.horizontalSpeed,
			this->mMissionConfiguration->mVelocityCfg.verticalSpeed,
			this->mMissionConfiguration->mVelocityCfg.yawSpeed);
	}

	int cmdMoveBy(relativeMove target,
		      float headingRotation,
		      float maxHorSpeed,
		      float maxVertSpeed,
		      float maxYawSpeed);
	int cmdMoveBy(relativeMove target, float headingRotation)
	{
		return cmdMoveBy(
			target,
			headingRotation,
			this->mMissionConfiguration->mVelocityCfg
				.horizontalSpeed,
			this->mMissionConfiguration->mVelocityCfg.verticalSpeed,
			this->mMissionConfiguration->mVelocityCfg.yawSpeed);
	}

	int cmdRTH();

	int cmdLand();

	void generateRelativeTrajectory();

	void eventInfo(uint32_t missingInputs);

	int onCmdReceived(const arsdk_cmd *cmd);
};