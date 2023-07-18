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

#define ULOG_TAG state_machine
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

#include <futils/mbox.h>

#include "state_machine.hpp"
#include "tracking_msghub.hpp"
#include "tracking_telemetry.hpp"

using namespace trackingExample::mbox;

/* - - - PRIVATE FUNCTIONS - - - */

void StateMachine::resetState()
{
	mState = STATE_IDLE;
	ULOGN("#SM State Machine reinitialized: [%s]", mStateName[mState]);
	processOnEnterState();
}

void StateMachine::startTimer(int delayMs)
{
	mTimer.set(delayMs);
}

void StateMachine::stopTimer()
{
	mTimer.clear();
}

const char *StateMachine::mStateName[6] = {
	"IDLE",
	"MANUAL_MODE",
	"MANUAL_MODE_SET_RECT",
	"AUTO_MODE",
	"TARGET_TRACKED",
	"SEARCHING_TARGET",
};

/* - - - PUBLIC FUNCTIONS - - - */

void StateMachine::mboxStateMachineCb()
{
	int res;
	stateMachine::msg msg;

	res = mbox_peek(mMboxStateMachine, &msg);
	if (res < 0) {
		ULOG_ERRNO("can't retrieve msg from mbox_state_machine", -res);
		return;
	}

	ULOGN("StateMachine received mbox message [%s]",
	      stateMachine::MsgTypeName[msg.type].c_str());

	processStateMachine(&msg);
}

void StateMachine::reset()
{
	resetState();
}

void StateMachine::changeState(State new_state)
{
	processOnExitState();
	ULOGN("#SM State Machine updated: [%s] -> [%s]",
	      mStateName[mState],
	      mStateName[new_state]);
	mState = new_state;
	processOnEnterState();
}

void StateMachine::processOnEnterState()
{
	ULOGN("Process on enter state [%s]", mStateName[mState]);
	switch (mState) {
	case STATE_IDLE:
		break;

	case STATE_MANUAL_MODE:
		mTrackingMsghub.stopAllTracking();
		break;

	case STATE_MANUAL_MODE_SET_RECT:
		mTrackingMsghub.setRect(0.0f, 0.0f, 0.5f, 0.5f);
		break;

	case STATE_AUTO_MODE:
		mTrackingMsghub.stopAllTracking();
		mTrackingTelemetry.resetTargetTracked();
		break;

	case STATE_TARGET_TRACKED:
		break;

	case STATE_SEARCHING_TARGET:
		startTimer(5000);
		break;

	default:
		ULOGN("State unknow : %d -> return to [%s]",
		      mState,
		      mStateName[STATE_IDLE]);
		changeState(STATE_IDLE);
		break;
	}
}

void StateMachine::processStateMachine(stateMachine::msg *msg)
{
	switch (mState) {
	case STATE_IDLE:
		/* If the message MSGHUB_AVAILABILITY_NOT_AVAILABLE is received,
		this means that the computer vision service has been started and
		is disable. */
		if (msg->type ==
		    stateMachine::MSGHUB_AVAILABILITY_NOT_AVAILABLE) {
			if (mTrackingMode == TRACKING_MODE_MANUAL) {
				/* The box proposals are disabled in
				 * MANUAL_MODE. */
				mTrackingMsghub.enable(false);
				changeState(STATE_MANUAL_MODE);
			} else {
				/* The box proposals are enabled in AUTO_MODE.
				 */
				mTrackingMsghub.enable(true);
				changeState(STATE_AUTO_MODE);
			}
		}
		break;

	case STATE_MANUAL_MODE:
		if (msg->type == stateMachine::MSGHUB_ANSWER_PROCESSED) {
			/* This one-second timer is here to give the service
			time to process the command (sent in
			processOnEnterState() function). At the end of the
			timer, a SM_TIMER_REACHED mbox message is sent and
			catched by the conditional statement just below. */
			startTimer(1000);
		}
		if (msg->type == stateMachine::SM_TIMER_REACHED) {
			changeState(STATE_MANUAL_MODE_SET_RECT);
		}
		break;

	case STATE_MANUAL_MODE_SET_RECT:
		if (msg->type == stateMachine::MSGHUB_ANSWER_PROCESSED) {
			changeState(STATE_TARGET_TRACKED);
		} else if (msg->type ==
			   stateMachine::MSGHUB_ANSWER_TARGET_LIMIT_REACHED) {
			changeState(STATE_MANUAL_MODE);
		}
		break;


	case STATE_AUTO_MODE:
		if (msg->type == stateMachine::TELEMETRY_SEND_TRACK_ID) {
			mTrackingMsghub.setId(
				msg->telemetrySetTargetMsg.track_id,
				msg->telemetrySetTargetMsg.timespec_us);
		}

		if (msg->type == stateMachine::MSGHUB_STATES_TRACKING) {
			changeState(STATE_TARGET_TRACKED);
		}
		break;

	case STATE_TARGET_TRACKED:
		if (msg->type == stateMachine::MSGHUB_STATES_SEARCHING) {
			changeState(STATE_SEARCHING_TARGET);
		}
		break;

	case STATE_SEARCHING_TARGET:
		if (msg->type == stateMachine::MSGHUB_STATES_TRACKING) {
			changeState(STATE_TARGET_TRACKED);
		}

		if (msg->type == stateMachine::SM_TIMER_REACHED) {
			if (mTrackingMode == TRACKING_MODE_MANUAL) {
				changeState(STATE_MANUAL_MODE);
			} else {
				changeState(STATE_AUTO_MODE);
			}
		}
		break;


	default:
		ULOGN("State unknow : %d -> return to [%s]",
		      mState,
		      mStateName[STATE_IDLE]);
		changeState(STATE_IDLE);
		break;
	}
}

void StateMachine::processOnExitState()
{
	ULOGN("Process on exit state [%s]", mStateName[mState]);
	switch (mState) {
	case STATE_IDLE:
		break;

	case STATE_MANUAL_MODE:
		break;

	case STATE_MANUAL_MODE_SET_RECT:
		break;

	case STATE_AUTO_MODE:
		break;

	case STATE_TARGET_TRACKED:
		break;

	case STATE_SEARCHING_TARGET:
		stopTimer();
		break;

	default:
		ULOGN("State unknow : %d -> return to [%s]",
		      mState,
		      mStateName[STATE_IDLE]);
		changeState(STATE_IDLE);
		break;
	}
}


int StateMachine::start()
{
	int res;

	reset();

	mMboxStateMachineHandlerFunc.set(
		[this](int fd, uint32_t revents) { mboxStateMachineCb(); });
	res = mLoop.add(mbox_get_read_fd(mMboxStateMachine),
			POMP_FD_EVENT_IN,
			&mMboxStateMachineHandlerFunc);
	if (res < 0) {
		ULOG_ERRNO("can't add mMboxStateMachine to the pomp loop",
			   -res);
	}

	return res;
}

int StateMachine::stop()
{
	int res = 0;

	res = mLoop.remove(mbox_get_read_fd(mMboxStateMachine));
	if (res < 0) {
		ULOG_ERRNO("can't remove mMboxStateMachine to the pomp loop",
			   -res);
	}

	return res;
}

void StateMachine::processTimer()
{
	int res;
	stateMachine::msg sendMsg;

	sendMsg.type = stateMachine::MsgType::SM_TIMER_REACHED;

	res = mbox_push(mMboxStateMachine, &sendMsg);
	if (res < 0)
		ULOG_ERRNO("mbox_push", -res);
}