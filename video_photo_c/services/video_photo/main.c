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
#include <signal.h>
#include <stdbool.h>
#include <unistd.h>
#include <libpomp.h>

#include "mission_controller.h"

#define ULOG_TAG_main video_photo
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG_main);

struct pomp_loop *loop;

/*
 * Signal handling towards the mission loop process
 */
static sig_atomic_t stop = 0;
static void sig_handler(int sig)
{
	ULOGI("signal %d (%s) received", sig, strsignal(sig));
	stop = 1;
	pomp_loop_wakeup(loop);
}

int main(int argc, char *argv[])
{
	/* Mission controller interface, responsible for commands and
	 * events handling
	 */
	struct mission_controller *msn_controller;

	int res = 0;

	/* Initialisation code
	 *
	 * The service is automatically started by the drone when the mission is
	 * loaded.
	 */
	ULOGI("Hello from video_photo mission");
	signal(SIGTERM, sig_handler);

	loop = pomp_loop_new();
	if (loop == NULL) {
		ULOGE("Error while creating loop");
		return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
	}

	/**
	 * Create a new mission controller
	 */
	msn_controller = mission_controller_new(loop);
	if (!msn_controller) {
		ULOGE("Error while defining a new msn_controller");
		mission_controller_destroy(msn_controller);
		pomp_loop_destroy(loop);
		return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
	}

	/**
	 * Initialize the mission controller
	 */
	res = mission_controller_init(msn_controller);
	if (res < 0) {
		ULOGE("Error while initializing a msn_controller");
		ULOG_ERRNO("mission_controller_init", -res);
		mission_controller_destroy(msn_controller);
		pomp_loop_destroy(loop);
		return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
	}

	/* Start the mission controller. That launches the custom 'state
	 * machine' built to start a video recording and photo */
	res = mission_controller_start(msn_controller);
	if (res != 0) {
		ULOGE("Error while starting a msn_controller");
		ULOG_ERRNO("mission_controller_start", -res);
		mission_controller_destroy(msn_controller);
		pomp_loop_destroy(loop);
		return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
	}

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
		pomp_loop_wait_and_process(loop, -1);

	/* Cleanup code
	 *
	 * When stopped by a SIGTERM, a service can use a short amount of time
	 * for cleanup (typically closing opened files and ensuring that the
	 * written data is coherent).
	 */
	ULOGI("Cleaning up from video_photo mission");

	/**
	 * Destroy the mission controller object to free its resources
	 */
	mission_controller_destroy(msn_controller);
	pomp_loop_destroy(loop);
	return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}