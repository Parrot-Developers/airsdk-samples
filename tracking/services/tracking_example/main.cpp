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

#include <csignal>
#include <unistd.h>

#include <futils/mbox.h>
#include <libpomp.hpp>
#include <msghub.hpp>

#include "mbox_message.hpp"
#include "state_machine.hpp"
#include "tracking_msghub.hpp"
#include "tracking_telemetry.hpp"

#define ULOG_TAG example_main
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

using namespace trackingExample::mbox;

sig_atomic_t run = 1;

extern "C" void sig_handler(int sig)
{
	run = 0;
}

int main(int argc, char *argv[])
{
	/* Initialisation code
	 *
	 * The service is automatically started by the drone when the mission is
	 * loaded.
	 */
	ULOGI("Hello from tracking example");
	signal(SIGTERM, sig_handler);

	pomp::Loop loop;
	struct mbox *mboxStateMachine = mbox_new(sizeof(stateMachine::msg));

	TrackingTelemetry trackingTelemetry(loop, mboxStateMachine);
	TrackingMsghub trackingMsghub(loop, mboxStateMachine);
	StateMachine stateMachine(
		loop, mboxStateMachine, trackingMsghub, trackingTelemetry);

	trackingTelemetry.initConsumer();

	stateMachine.start();
	trackingTelemetry.startt();
	trackingMsghub.start();

	/* Loop code
	 *
	 * The service is assumed to run an infinite loop, and termination
	 * requests are handled via a SIGTERM signal.
	 * If your service exits before this SIGTERM is sent, it will be
	 * considered as a crash, and the system will relaunch the service.
	 * If this happens too many times, the system will no longer start the
	 * service.
	 */
	while (run) {
		loop.waitAndProcess(-1);
	}

	/* Cleanup code
	 *
	 * When stopped by a SIGTERM, a service can use a short amount of time
	 * for cleanup (typically closing opened files and ensuring that the
	 * written data is coherent).
	 */
	ULOGI("Cleaning up from tracking_example");

	trackingMsghub.stop();
	trackingTelemetry.stopt();
	stateMachine.stop();

	mbox_destroy(mboxStateMachine);

	return 0;
}