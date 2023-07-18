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

#include "mbox_message.hpp"
#include "tracking_msghub.hpp"

#define ULOG_TAG tracking_example
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

using namespace trackingExample::mbox;

void TrackingConnection::onConnected(msghub::Channel *channel,
				     pomp::Connection *conn)
{
	ULOGN("Connected to the server: %s",
	      COMPUTER_VISION_TRACKING_MSGHUB_ADD);
}

void TrackingConnection::onDisconnected(msghub::Channel *channel,
					pomp::Connection *conn)
{
	ULOGN("Disconnected to the server: %s",
	      COMPUTER_VISION_TRACKING_MSGHUB_ADD);
}


void TrackingEventHandler::states(
	const ::CvTrackingService::Messages::TrackingStates &msg)
{
	int i;
	int res;
	stateMachine::msg sendMsg;

	for (i = 0; i < msg.states_size(); i++) {
		if (msg.states(i).status() ==
		    ::CvTrackingService::Messages::Status::TRACKING)
			sendMsg.type =
				stateMachine::MsgType::MSGHUB_STATES_TRACKING;
		else if (msg.states(i).status() ==
			 ::CvTrackingService::Messages::Status::SEARCHING)
			sendMsg.type =
				stateMachine::MsgType::MSGHUB_STATES_SEARCHING;
		else if (msg.states(i).status() ==
			 ::CvTrackingService::Messages::Status::ABANDON)
			sendMsg.type =
				stateMachine::MsgType::MSGHUB_STATES_ABANDON;
		else
			sendMsg.type =
				stateMachine::MsgType::MSGHUB_STATES_ERROR;
		res = mbox_push(mMboxStateMachine, &sendMsg);
		if (res < 0)
			ULOG_ERRNO("mbox_push", -res);
	}
}

void TrackingEventHandler::answer(::CvTrackingService::Messages::Answer msg)
{
	int res;
	stateMachine::msg sendMsg;

	if (msg == ::CvTrackingService::Messages::Answer::PROCESSED)
		sendMsg.type = stateMachine::MsgType::MSGHUB_ANSWER_PROCESSED;
	else if (msg ==
		 ::CvTrackingService::Messages::Answer::TARGET_LIMIT_REACHED)
		sendMsg.type = stateMachine::MsgType::
			MSGHUB_ANSWER_TARGET_LIMIT_REACHED;
	else if (msg == ::CvTrackingService::Messages::Answer::NOT_FOUND)
		sendMsg.type = stateMachine::MsgType::MSGHUB_ANSWER_NOT_FOUND;
	else if (msg == ::CvTrackingService::Messages::Answer::INVALID)
		sendMsg.type = stateMachine::MsgType::MSGHUB_ANSWER_INVALID;
	else
		sendMsg.type = stateMachine::MsgType::MSGHUB_ANSWER_ERROR;

	res = mbox_push(mMboxStateMachine, &sendMsg);
	if (res < 0)
		ULOG_ERRNO("mbox_push", -res);
}

void TrackingEventHandler::availability(
	::CvTrackingService::Messages::Availability msg)
{
	int res;
	stateMachine::msg sendMsg;

	if (msg == ::CvTrackingService::Messages::Availability::AVAILABLE)
		sendMsg.type =
			stateMachine::MsgType::MSGHUB_AVAILABILITY_AVAILABLE;
	else if (msg ==
		 ::CvTrackingService::Messages::Availability::NOT_AVAILABLE)
		sendMsg.type = stateMachine::MsgType::
			MSGHUB_AVAILABILITY_NOT_AVAILABLE;
	else
		sendMsg.type = stateMachine::MsgType::MSGHUB_AVAILABILITY_ERROR;

	res = mbox_push(mMboxStateMachine, &sendMsg);
	if (res < 0)
		ULOG_ERRNO("mbox_push", -res);
}

int TrackingMsghub::start()
{
	mChannel =
		mMsghub.startClientChannel(COMPUTER_VISION_TRACKING_MSGHUB_ADD);
	if (!mChannel)
		return -1;

	mMsghub.attachMessageHandler(&mHandler);
	mMsghub.attachMessageSender(&mSender, mChannel);
	mMsghub.enableDump();

	return 0;
}

void TrackingMsghub::stop()
{
	mMsghub.stop();
	mMsghub.detachMessageHandler(&mHandler);
	mMsghub.detachMessageSender(&mSender);
}

void TrackingMsghub::enable(bool start_with_box_proposal)
{
	::CvTrackingService::Messages::Enable msg;
	msg.set_with_boxprop(start_with_box_proposal);

	mSender.enable(msg);
}

void TrackingMsghub::stopAllTracking()
{
	mSender.stopAllTracking({});
}

void TrackingMsghub::setId(uint32_t id, uint64_t tsUs)
{
	::CvTrackingService::Messages::SetId msg;
	msg.set_id(id);
	msg.set_mode(CvTrackingService::Messages::SetMode::ADD);
	msg.set_ts_us(tsUs);
	msg.set_cookie(0);

	mSender.setId(msg);
}

void TrackingMsghub::setRect(float pos_x, float pos_y, float width, float height)
{
	::CvTrackingService::Messages::SetRect msg;
	::CvTrackingService::Messages::Rect *rect = msg.mutable_rect();

	rect->set_left_x(pos_x);
	rect->set_top_y(pos_y);
	rect->set_width(width);
	rect->set_height(height);

	mSender.setRect(msg);
}
