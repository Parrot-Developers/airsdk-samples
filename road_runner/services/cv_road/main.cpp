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

#include <libpomp.hpp>

#define ULOG_TAG service_main
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "src/processing.hpp"

struct context {
	/* Main loop of the program */
	pomp::Loop loop;

	/* Processing object */
	Processing *processing;
};

/* Global context (so the signal handler can access it) */
static struct context s_ctx;
/* Stop flag, set to 1 by signal handler to exit cleanly */
static sig_atomic_t stop;

static void context_clean(struct context *ctx)
{
	if (ctx->processing != nullptr) {
		delete ctx->processing;
		ctx->processing = nullptr;
	}
}

static int context_init(struct context *ctx)
{
	try {
		ctx->processing = new Processing(&ctx->loop);
	} catch (std::exception &ex) {
		ULOGE("Processing object creation failed (%s)", ex.what());
		goto error;
	}

	return 0;
error:
	context_clean(ctx);
	return -1;
}

static int context_start(struct context *ctx)
{
	int res = 0;

	if (!ctx->processing) {
		res = -EINVAL;
		ULOG_ERRNO("context_start", -res);
		goto out;
	}

	res = ctx->processing->start();
	if (res != 0) {
		ULOG_ERRNO("Processing::start", -res);
		goto out;
	}

out:
	return res;
}

static void context_stop(struct context *ctx)
{
	if (ctx->processing) {
		ctx->processing->stop();
	}
}

static void sighandler(int signum)
{
	/* Set stopped flag and wakeup loop */
	ULOGI("signal %d (%s) received", signum, strsignal(signum));
	stop = 1;
	pomp_loop_wakeup(s_ctx.loop.get());
}

int main(int argc, char *arhv[])
{
	int res = 0;

	/* Initialize context */
	res = context_init(&s_ctx);
	if (res != 0)
		goto out;

	/* Setup signal handler */
	signal(SIGINT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGPIPE, SIG_IGN);

	res = context_start(&s_ctx);
	if (res != 0)
		goto out;

	/* Run loop until stop is requested */
	while (!stop)
		pomp_loop_wait_and_process(s_ctx.loop.get(), -1);

out:
	/* Stop and cleanup */
	context_stop(&s_ctx);

	signal(SIGINT, SIG_DFL);
	signal(SIGTERM, SIG_DFL);
	signal(SIGPIPE, SIG_DFL);
	context_clean(&s_ctx);

	return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}