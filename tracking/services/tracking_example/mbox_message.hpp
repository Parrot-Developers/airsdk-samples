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

#include <string>
#include <vector>

namespace trackingExample {
namespace mbox {
namespace stateMachine {

enum MsgType {
	SM_TIMER_REACHED,

	/* Tracking states for all the targets  */
	MSGHUB_STATES_TRACKING,
	MSGHUB_STATES_SEARCHING,
	MSGHUB_STATES_ABANDON,
	MSGHUB_STATES_ERROR,

	/* Answer to a tracking request. */
	MSGHUB_ANSWER_PROCESSED,
	MSGHUB_ANSWER_TARGET_LIMIT_REACHED,
	MSGHUB_ANSWER_NOT_FOUND,
	MSGHUB_ANSWER_INVALID,
	MSGHUB_ANSWER_ERROR,

	/* Availability of the visual tracking feature. */
	MSGHUB_AVAILABILITY_AVAILABLE,
	MSGHUB_AVAILABILITY_NOT_AVAILABLE,
	MSGHUB_AVAILABILITY_ERROR,

	TELEMETRY_SEND_TRACK_ID
};

const std::vector<std::string> MsgTypeName{"SM_TIMER_REACHED",
					   "MSGHUB_STATES_TRACKING",
					   "MSGHUB_STATES_SEARCHING",
					   "MSGHUB_STATES_ABANDON",
					   "MSGHUB_STATES_ERROR",
					   "MSGHUB_ANSWER_PROCESSED",
					   "MSGHUB_ANSWER_TARGET_LIMIT_REACHED",
					   "MSGHUB_ANSWER_NOT_FOUND",
					   "MSGHUB_ANSWER_INVALID",
					   "MSGHUB_ANSWER_ERROR",
					   "MSGHUB_AVAILABILITY_AVAILABLE",
					   "MSGHUB_AVAILABILITY_NOT_AVAILABLE",
					   "MSGHUB_AVAILABILITY_ERROR",
					   "TELEMETRY_SEND_TRACK_ID"};

struct msg {
	enum MsgType type;
	union {
		struct {
			uint32_t track_id;
			uint64_t timespec_us;
		} telemetrySetTargetMsg;
	};
};

} // namespace stateMachine
} // namespace mbox
} // namespace trackingExample