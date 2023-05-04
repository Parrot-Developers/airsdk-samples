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

#include "control_interface.hpp"
#include <libpomp.hpp>

#define ULOG_TAG move_along
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

/****************************************************************/

class Context {
	/* Main loop of the program */
	pomp::Loop mLoop;

	ControlInterface controlInterface;
	MissionConfiguration missionConfiguration;

public:
	Context() : controlInterface(this->mLoop, &this->missionConfiguration)
	{
	}

	inline void wakeup()
	{
		this->mLoop.wakeup();
	}
	inline void waitAndProcess(int timeout)
	{
		this->mLoop.waitAndProcess(timeout);
	}
	inline int start()
	{
		int res = 0;
		res = this->controlInterface.start();
		if (res != 0) {
			ULOGE("Error while starting ControlInterface");
			ULOG_ERRNO("ControlInterface::start", -res);
			return res;
		} else
			ULOGI("ControlInterface has started successfully");

		res = this->missionConfiguration.start();
		if (res != 0) {
			ULOGE("Error while starting MissionController");
			ULOG_ERRNO("MissionController::start", -res);
			return res;
		} else
			ULOGI("MissionController has started successfully");
		return res;
	}
};

/*
 * Global context of the mission. It retrieves all the objects and
 * controllers the mission needs to work up
 */
static Context s_ctx;

/****************************************************************/

/* Stop flag, set to 1 by signal handler to exit cleanly */
static sig_atomic_t stop;

static void sighandler(int signum)
{
	/* Set stopped flag and wakeup loop */
	ULOGI("Signal %d (%s) received", signum, strsignal(signum));
	stop = 1;
	s_ctx.wakeup();
}

/****************************************************************/

int main(int argc, char *argv[])
{
	int res;

	/* Initialisation code
	 *
	 * The service is automatically started by the drone when the mission is
	 * loaded.
	 */
	ULOGI("Hello from moveby mission");
	/* Setup signal handler */
	signal(SIGINT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGPIPE, SIG_IGN);

	/* Initialize and start context */
	res = s_ctx.start();
	if (res != 0) {
		ULOGE("Error while starting MissionController");
		ULOG_ERRNO("MissionController::start", -res);
		return res;
	} else
		ULOGI("MissionController has started successfully");

	/* Loop code
	 *
	 * The service is assumed to run an infinite loop, and termination
	 * requests are handled via a SIGTERM signal.
	 * If your serivce exists before this SIGTERM is sent, it will be
	 * considered as a crash, and the system will relaunch the service.
	 * If this happens too many times, the system will no longer start the
	 * service.
	 */
	while (!stop)
		s_ctx.waitAndProcess(-1);

	/* Cleanup code
	 *
	 * When stopped by a SIGTERM, a service can use a short amount of time
	 * for cleanup (typically closing opened files and ensuring that the
	 * written data is coherent).
	 */
	ULOGI("Cleaning up from moveby mission");

	return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}