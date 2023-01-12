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
#include <stdatomic.h>
#include <stdbool.h>
#include <unistd.h>

#define ULOG_TAG ex_tlm_c_main
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "example_telemetry.h"

atomic_bool run;

static void sig_handler(int sig)
{
	atomic_store(&run, false);
}

int main(int argc, char *argv[])
{
	/* Initialisation code
	 *
	 * The service is automatically started by the drone when the mission is
	 * loaded.
	 */
	ULOGI("Hello from example_telemetry");
	atomic_init(&run, true);
	signal(SIGTERM, sig_handler);
	struct example_telemetry_context *example_telemetry_ctx;
	int res;

	res = 0;

	example_telemetry_ctx = example_telemetry_new();
	if (!example_telemetry_ctx) {
		ULOGE("can't create example_telemetry_context");
		res = -1;
		goto out;
	}

	res = example_telemetry_init(example_telemetry_ctx);
	if (res < 0) {
		ULOG_ERRNO("example_telemetry_init", -res);
		goto out;
	}

	/* Loop code
	 *
	 * The service is assumed to run an infinite loop, and termination
	 * requests are handled via a SIGTERM signal.
	 * If your service exits before this SIGTERM is sent, it will be
	 * considered as a crash, and the system will relaunch the service.
	 * If this happens too many times, the system will no longer start the
	 * service.
	 */
	while (atomic_load(&run)) {
		example_telemetry_put_samples(example_telemetry_ctx);
		sleep(5);
		example_telemetry_get_samples(example_telemetry_ctx);
		example_telemetry_log_values(example_telemetry_ctx);
	}

out:
	/* Cleanup code
	 *
	 * When stopped by a SIGTERM, a service can use a short amount of time
	 * for cleanup (typically closing opened files and ensuring that the
	 * written data is coherent).
	 */
	ULOGI("Cleaning up from example_telemetry");
	example_telemetry_destroy(example_telemetry_ctx);

	if (res < 0)
		return EXIT_FAILURE;
	else
		return EXIT_SUCCESS;
}