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

#define ULOG_TAG video
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "video.hpp"

static void frame_cb(struct vipcc_ctx *ctx,
		     const struct vipc_frame *frame,
		     void *be_frame,
		     void *userdata)
{
	int res = 0;

	Listener *listener = (Listener *)userdata;

	res = listener->processingStep(listener->getUserdata(), frame);
	if (res < 0) {
		ULOG_ERRNO("processing_step", -res);
		goto out;
	}

	/* The frame has been given to the processing object */
	frame = NULL;

	/* In case of error, need to release the input frame */
out:
	if (frame != NULL)
		vipcc_release(ctx, frame);
}

static void
conn_status_cb(struct vipcc_ctx *ctx, bool connected, void *userdata)
{
	ULOGN("connected: %d", connected);
}

static void
status_cb(struct vipcc_ctx *ctx, const struct vipc_status *st, void *userdata)
{
	int res = 0;

	res = vipcc_start(ctx);
	if (res < 0)
		ULOG_ERRNO("vipcc_start", -res);
}

static const struct vipcc_cb s_vipc_client_cbs = {.status_cb = status_cb,
						  .configure_cb = NULL,
						  .frame_cb = frame_cb,
						  .connection_status_cb =
							  conn_status_cb,
						  .eos_cb = NULL};

Video::~Video()
{
	mLoop = nullptr;
	mVipcc = nullptr;

	if (mVipcc != NULL) {
		vipcc_stop(mVipcc);
		vipcc_destroy(mVipcc);
		mVipcc = NULL;
	}

	if (mFrameDim != NULL) {
		mFrameDim = NULL;
	}
}

int Video::vipcStart(Listener *listener)
{
	int res = 0;

	/* VIDEO */
	struct vipcc_cfg_info vipc_info;
	memset(&vipc_info, 0, sizeof(vipc_info));

	/* Get vipc cfg info */
	res = vipcc_cfg_get_info(VIPC_FRONT_CAM_STREAM, &vipc_info);
	if (res != 0) {
		ULOG_ERRNO("vipcc_cfg_get_info('%s')",
			   -res,
			   VIPC_FRONT_CAM_STREAM);
		goto out;
	}

	/* Create vipc client */
	mVipcc = vipcc_new(mLoop->get(),
			   &s_vipc_client_cbs,
			   vipc_info.be_cbs,
			   vipc_info.address,
			   listener,
			   1,
			   true);
	if (mVipcc == NULL) {
		res = -EPERM;
		ULOG_ERRNO("vipcc_new", -res);
		goto out;
	}

out:
	vipcc_cfg_release_info(&vipc_info);
	return res;
}

void Video::vipcStop()
{
	/* destroy video */
	if (mVipcc != NULL) {
		vipcc_stop(mVipcc);
		vipcc_destroy(mVipcc);
		mVipcc = NULL;
	}
}