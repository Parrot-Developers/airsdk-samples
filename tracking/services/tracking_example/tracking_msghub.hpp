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
#include <msghub.hpp>

#include <cv-tracking-service/cv-tracking-service.msghub.h>
#include <cv-tracking-service/cv-tracking-service.pb.h>

#include "mbox_message.hpp"

#define COMPUTER_VISION_TRACKING_MSGHUB_ADD "unix:/tmp/selkie"

class TrackingTelemetry;
class TrackingMsghub;

class TrackingConnection : public msghub::MessageHub::ConnectionHandler {
public:
	void onConnected(msghub::Channel *channel,
			 pomp::Connection *conn) override;

	void onDisconnected(msghub::Channel *channel,
			    pomp::Connection *conn) override;
};

class TrackingEventHandler
		: public CvTrackingService::Messages::msghub::EventHandler {
private:
	mbox *mMboxStateMachine;

public:
	TrackingEventHandler(mbox *mboxStateMachine) :
			mMboxStateMachine(mboxStateMachine){};

	void states(const ::CvTrackingService::Messages::TrackingStates &msg)
		override;
	void answer(::CvTrackingService::Messages::Answer msg) override;
	void
	availability(::CvTrackingService::Messages::Availability msg) override;
};


class TrackingMsghub {
private:
	::pomp::Loop &mLoop;

	/* mailbox */
	mbox *mMboxStateMachine;

	TrackingConnection mMsghubConn;
	TrackingEventHandler mHandler;
	::CvTrackingService::Messages::msghub::CommandSender mSender;

	::msghub::MessageHub mMsghub;
	::msghub::Channel *mChannel;

public:
	TrackingMsghub(pomp::Loop &loop, mbox *mboxStateMachine) :
			mLoop(loop), mMboxStateMachine(mboxStateMachine),
			mMsghubConn(), mHandler(mboxStateMachine),
			mMsghub(&mLoop, &mMsghubConn)
	{
	}

	/**
	 * Start msghub
	 *
	 * @return  0 in case of success, negative errno in case of error.
	 */
	int start();

	/**
	 * Stop msghub
	 */
	void stop();

	/**
	 * Send enable message
	 *
	 * Enable tracking with or without box proposal.
	 *
	 * @param start_with_box_proposal start the computer vision service with
	 * or withour box proposal.
	 */
	void enable(bool start_with_box_proposal);


	/**
	 * Send stopAllTracking message.
	 *
	 * Stop target tracking.
	 */
	void stopAllTracking();

	/**
	 * Send setId message.
	 *
	 * Set a new target using an id from the proposed targets.
	 *
	 * @param id Identification of one of the proposed boxes.
	 * @param tsUs Timestamp that was associated to the frame on which the
	 * box was selected.
	 */
	void setId(uint32_t id, uint64_t tsUs);

	/**
	 * Send setRect message.
	 *
	 * Set a new target using a rectangle.
	 *
	 * @param pos_x position on the horizontal axis.
	 * @param pos_y position on the vertical axis.
	 * @param width % of the total width to the right.
	 * @param height % of the total height downwards.
	 */
	void setRect(float pos_x, float pos_y, float width, float height);
};
