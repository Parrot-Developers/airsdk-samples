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

#include <libpomp.hpp>

#include "mbox_message.hpp"

class TrackingMsghub;
class TrackingTelemetry;

class StateMachine : public pomp::Timer::Handler {
private:
	/* Pomp loop object */
	pomp::Loop &mLoop;

	/* mBox */
	struct mbox *mMboxStateMachine;
	pomp::Loop::HandlerFunc mMboxStateMachineHandlerFunc;

	/* objects */
	TrackingMsghub &mTrackingMsghub;
	TrackingTelemetry &mTrackingTelemetry;

	/* timer */
	pomp::Timer mTimer;

	/* Static members */
	enum State {
		STATE_IDLE,
		STATE_MANUAL_MODE,
		STATE_MANUAL_MODE_SET_RECT,
		STATE_AUTO_MODE,
		STATE_TARGET_TRACKED,
		STATE_SEARCHING_TARGET,
	} mState;

	enum TrackingMode{
		TRACKING_MODE_MANUAL,
		TRACKING_MODE_AUTO,
	} mTrackingMode;

	static const char *mStateName[];

private:
	/**
	 * Reset the state to the idle state.
	 */
	void resetState();

	/**
	 * Start the timer.
	 *
	 * @param delayMs timer delay.
	 */
	void startTimer(int delayMs);

	/**
	 * Stop the timer.
	 */
	void stopTimer();

	/**
	 * The function is executed once when a state is entered.
	 */
	void processOnEnterState();

	/**
	 * The function is executed once when a state is exited
	 */
	void processOnExitState();

	/**
	 * Process all the event from the mailbox.
	 *
	 * @param msg message from the mailbox.
	 */
	void processStateMachine(trackingExample::mbox::stateMachine::msg *msg);

public:
	StateMachine(pomp::Loop &loop,
		     struct mbox *mboxStateMachine,
		     TrackingMsghub &trackingMsghub,
		     TrackingTelemetry &trackingTelemetry) :
			mLoop(loop),
			mMboxStateMachine(mboxStateMachine),
			mTrackingMsghub(trackingMsghub),
			mTrackingTelemetry(trackingTelemetry),
			mTimer(&mLoop, this), mTrackingMode(TRACKING_MODE_AUTO){};

	/**
	 * Start state machine.
	 *
	 * @return  0 in case of success, negative errno in case of error.
	 */
	int start();

	/**
	 * Stop state machine.
	 *
	 * @return  0 in case of success, negative errno in case of error.
	 */
	int stop();

	/**
	 * Mailbox handler.
	 */
	void mboxStateMachineCb();

	/**
	 * Reset the state machine.
	 */
	void reset();

	/**
	 * Change the state of the state machine.
	 *
	 * @param new_state the next state of the state machine.
	 */
	void changeState(State new_state);

	/**
	 * Override Timer function.
	 */
	virtual void processTimer() override;
};