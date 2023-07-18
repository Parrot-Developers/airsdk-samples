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

#include <libpomp.hpp>
#include <libtelemetry.hpp>

#include "mbox_message.hpp"

#define CV_TRACKING_PROPOSALS_TELEMETRY_SECTION "cv@tracking@proposals"
#define CV_TRACKING_BOX_TELEMETRY_SECTION "cv@tracking@box"

#define TRACKING_BOX_SIZE 10

class TrackingTelemetry : public pomp::Timer::Handler {
private:
	pomp::Loop &mLoop;

	/* mailbox */
	mbox *mMboxStateMachine;

	telemetry::Consumer *mCvTrackingProposalsConsumer;
	telemetry::Consumer *mCvTrackingBoxConsumer;

	pomp::Timer mTimer;

	struct Box {
		float x;
		float y;
		float width;
		float height;
		uint32_t classId;
		float condidence;
		uint32_t trackId;
	};

	struct {
		uint32_t count;
		Box box[TRACKING_BOX_SIZE];
		timespec ts;
	} mCvTrackingProposalsData;

	struct {
		uint32_t status;
		Box box;
		uint32_t tagId;
		uint8_t quality;
	} mCvTrackingBoxData;

	bool mTargetTracked;

private:
	/**
	 * Register values in the 'cv tracking proposals' section. This section
	 * includes the box proposal data (between 0 and 10 boxes). Only if the
	 * Computer vision tracking is enabled with 'with_boxprop' option.
	 *
	 * @return 0 in case of success, negative errno in case of error.
	 */
	int registerCvTrackingProposalsConsumer();

	/**
	 * Display the values of mCvTrackingProposalsConsumer in the drone
	 * terminal.
	 */
	void logCvTrackingProposalsData();

	/**
	 * Register values in the 'cv tracking box' section.This section
	 * includes the tracking box data. The tracking box is the box selected
	 * by the user
	 *
	 * @return 0 in case of success, negative errno in case of error.
	 */
	int registerCvTrackingBoxConsumer();

	/**
	 * Display the values of mCvTrackingBoxConsumer in the drone
	 * terminal.
	 */
	void logCvTrackingBoxData();

	/**
	 * Send trackId of the first box proposal to the state machine if there
	 * is no tracking target enabled.
	 *
	 * @return 0 in case of success, -1 if there are no proposal box
	 * available, negative errno in case of error.
	 */
	int sendTrackId();

public:
	TrackingTelemetry(pomp::Loop &loop, mbox *mboxStateMachine);
	~TrackingTelemetry();

	/**
	 * initialize the telemetry consumers:
	 *  - mCvTrackingProposalsConsumer
	 *  - mCvTrackingBoxConsumer
	 *
	 * @return 0 in case of success, negative errno in case of error.
	 */
	int initConsumer();

	/**
	 * Start a timer to obtain telemetry samples every second.
	 */
	void startt();

	/**
	 * Stop the timer.
	 */
	void stopt();

	/**
	 * Reset the TargetTracked value to select a new box proposal.
	 */
	void resetTargetTracked();

	/**
	 * Override Timer function.
	 */
	virtual void processTimer() override;
};
