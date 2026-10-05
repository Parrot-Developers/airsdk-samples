/**
 * Copyright (c) 2026 Parrot Drones SAS
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
 *
 * object_detector viewer — minimal pdraw-backend + SDL2/GLES2 app that plays
 * the drone's live stream and overlays the mission's detection boxes.
 *
 * Modeled on the public pdraw reference app (apps/pdraw_desktop). Kept minimal
 * on purpose: single stream, single video renderer, no on-screen UI. The
 * detection-box drawing lives in overlay.c and is invoked from render_overlay.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <GLES2/gl2.h>
#include <SDL2/SDL.h>
#include <pdraw/pdraw_backend.h>

#define ULOG_TAG object_detector_viewer
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "overlay.h"

/* UID the mission pushes its detections under (MetadataPublisher). Must match
 * the mission's DETECTIONS_METADATA_UID exactly: the overlay matches the vmeta
 * user entry by the Jenkins hash of this string. Dotless on purpose (telemetry
 * section names with dots break the daemon-side consumer). */
#define DETECTIONS_METADATA_UID "object_detector"

#define WIN_W 1280
#define WIN_H 720

struct context {
	const char *url;
	struct pdraw_backend *pdraw;
	struct pdraw_demuxer *demuxer;
	struct pdraw_video_renderer *renderer;
	unsigned int media_id; /* raw video media to render, 0 if none yet */
	SDL_Window *window;
	SDL_GLContext gl_context;
	struct overlay *overlay;
	int stopped;         /* set when the main loop should exit */
	int close_requested; /* set by any thread to begin a graceful close */
	int closing;         /* graceful close started (main thread only) */
};

/* ── renderer callbacks ─────────────────────────────────────────────── */

static void render_overlay_cb(struct pdraw_backend *pdraw,
			      struct pdraw_video_renderer *renderer,
			      const struct pdraw_rect *render_pos,
			      const struct pdraw_rect *content_pos,
			      const float *view_mat,
			      const float *proj_mat,
			      const struct pdraw_media_info *media_info,
			      struct vmeta_frame *frame_meta,
			      const struct pdraw_video_frame_extra *frame_extra,
			      void *userdata)
{
	struct context *ctx = userdata;
	(void)pdraw;
	(void)renderer;
	(void)view_mat;
	(void)proj_mat;
	(void)media_info;
	(void)frame_extra;

	/* Draw the detection boxes on top of the freshly rendered frame. */
	overlay_render(ctx->overlay, render_pos, content_pos, frame_meta);
}

static const struct pdraw_backend_video_renderer_cbs render_cbs = {
	.render_overlay = &render_overlay_cb,
};

static void create_renderer(struct context *ctx)
{
	if (ctx->renderer != NULL || ctx->media_id == 0 ||
	    ctx->gl_context == NULL)
		return;

	struct pdraw_rect pos = {0, 0, WIN_W, WIN_H};
	struct pdraw_video_renderer_params params;
	memset(&params, 0, sizeof(params));
	params.fill_mode = PDRAW_VIDEO_RENDERER_FILL_MODE_FIT;

	int res = pdraw_be_video_renderer_new(ctx->pdraw,
					      ctx->media_id,
					      &pos,
					      &params,
					      &render_cbs,
					      ctx,
					      &ctx->renderer);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_video_renderer_new", -res);
}

/* ── backend callbacks ──────────────────────────────────────────────── */

static void media_added_cb(struct pdraw_backend *pdraw,
			    const struct pdraw_media_info *info,
			    void *element_userdata,
			    void *userdata)
{
	struct context *ctx = userdata;
	(void)pdraw;
	(void)element_userdata;

	/* Only keep the RAW (decoded) video media — that is what the GL
	 * renderer displays; the CODED media is the decoder input. The renderer
	 * itself MUST be created on the main thread where the GL context is
	 * current (see the render loop), never from this backend callback. */
	if (info->type != PDRAW_MEDIA_TYPE_VIDEO)
		return;
	if (info->video.format != VDEF_FRAME_TYPE_RAW)
		return;
	if (ctx->media_id != 0)
		return;
	ctx->media_id = info->id;
}

static void media_removed_cb(struct pdraw_backend *pdraw,
			     const struct pdraw_media_info *info,
			     void *element_userdata,
			     void *userdata)
{
	(void)pdraw;
	(void)info;
	(void)element_userdata;
	(void)userdata;
}

static void socket_created_cb(struct pdraw_backend *pdraw,
			      int fd,
			      void *userdata)
{
	(void)pdraw;
	(void)fd;
	(void)userdata;
}

static const struct pdraw_backend_cbs be_cbs = {
	.media_added = &media_added_cb,
	.media_removed = &media_removed_cb,
	.socket_created = &socket_created_cb,
};

/* ── demuxer callbacks ──────────────────────────────────────────────── */

static void demuxer_open_resp_cb(struct pdraw_backend *pdraw,
				 struct pdraw_demuxer *demuxer,
				 int status,
				 void *userdata)
{
	(void)pdraw;
	(void)demuxer;
	(void)userdata;
	if (status < 0)
		ULOG_ERRNO("demuxer open", -status);
}

static void demuxer_close_resp_cb(struct pdraw_backend *pdraw,
				  struct pdraw_demuxer *demuxer,
				  int status,
				  void *userdata)
{
	struct context *ctx = userdata;
	(void)status;
	/* Close handshake done: the demuxer is stopped (RTSP client shut down,
	 * no more reconnection) and can now be destroyed safely. */
	int res = pdraw_be_demuxer_destroy(pdraw, demuxer);
	if (res < 0)
		ULOG_ERRNO("pdraw_be_demuxer_destroy", -res);
	ctx->demuxer = NULL;
	ctx->stopped = 1;
}

static void demuxer_unrecoverable_error_cb(struct pdraw_backend *pdraw,
					   struct pdraw_demuxer *demuxer,
					   void *userdata)
{
	struct context *ctx = userdata;
	(void)pdraw;
	(void)demuxer;
	ULOGE("demuxer unrecoverable error");
	ctx->close_requested = 1;
}

static int demuxer_select_media_cb(struct pdraw_backend *pdraw,
				   struct pdraw_demuxer *demuxer,
				   const struct pdraw_demuxer_media *medias,
				   size_t count,
				   uint32_t selected_medias,
				   void *userdata)
{
	(void)pdraw;
	(void)demuxer;
	(void)selected_medias;
	(void)userdata;

	/* Display the AirSDK stream ("Front camera (AirSDK)") — the same stream
	 * inference runs on, where the mission embeds its detections
	 * (front-airsdk resource) — so the boxes align with the video. */
	for (size_t i = 0; i < count; i++) {
		if (medias[i].name != NULL &&
		    strstr(medias[i].name, "AirSDK") != NULL) {
			ULOGI("selecting media %d (%s)", medias[i].media_id,
			      medias[i].name);
			return 1 << medias[i].media_id;
		}
	}
	ULOGW("no AirSDK media found, using default set");
	return 0;
}

static void demuxer_ready_to_play_cb(struct pdraw_backend *pdraw,
				     struct pdraw_demuxer *demuxer,
				     int ready,
				     void *userdata)
{
	struct context *ctx = userdata;
	(void)demuxer;
	if (ready && !ctx->close_requested && !ctx->closing)
		pdraw_be_demuxer_play(pdraw, ctx->demuxer);
}

static const struct pdraw_backend_demuxer_cbs demuxer_cbs = {
	.open_resp = &demuxer_open_resp_cb,
	.close_resp = &demuxer_close_resp_cb,
	.unrecoverable_error = &demuxer_unrecoverable_error_cb,
	.select_media = &demuxer_select_media_cb,
	.ready_to_play = &demuxer_ready_to_play_cb,
};

/* ── main ───────────────────────────────────────────────────────────── */

/* Begin a graceful shutdown, on the main thread (GL context current). Destroys
 * the renderer here, then closes the demuxer asynchronously; the demuxer is
 * destroyed only once close_resp_cb fires, so the RTSP client (and its
 * auto-reconnection) is fully stopped before anything is torn down. Modeled on
 * pdraw_desktop's close sequence. */
static void request_close(struct context *ctx)
{
	if (ctx->closing)
		return;
	ctx->closing = 1;

	if (ctx->renderer != NULL) {
		int res = pdraw_be_video_renderer_destroy(ctx->pdraw,
							  ctx->renderer);
		if (res < 0)
			ULOG_ERRNO("pdraw_be_video_renderer_destroy", -res);
		ctx->renderer = NULL;
	}

	if (ctx->demuxer != NULL) {
		int res = pdraw_be_demuxer_close(ctx->pdraw, ctx->demuxer);
		if (res < 0) {
			ULOG_ERRNO("pdraw_be_demuxer_close", -res);
			ctx->stopped = 1; /* nothing to wait for */
		}
	} else {
		ctx->stopped = 1;
	}
}

int main(int argc, char *argv[])
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s <rtsp-url>\n", argv[0]);
		return EXIT_FAILURE;
	}

	struct context ctx;
	memset(&ctx, 0, sizeof(ctx));
	ctx.url = argv[1];

	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		ULOGE("SDL_Init: %s", SDL_GetError());
		return EXIT_FAILURE;
	}
	/* OpenGL ES 3 context, like pdraw's reference desktop app (ES 3 is
	 * backward compatible with the GLES2 API used in overlay.c). */
#if !defined(__APPLE__)
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
			    SDL_GL_CONTEXT_PROFILE_ES);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
#endif
	ctx.window = SDL_CreateWindow("object_detector viewer",
				      SDL_WINDOWPOS_CENTERED,
				      SDL_WINDOWPOS_CENTERED,
				      WIN_W,
				      WIN_H,
				      SDL_WINDOW_OPENGL);
	if (ctx.window == NULL) {
		ULOGE("SDL_CreateWindow: %s", SDL_GetError());
		return EXIT_FAILURE;
	}
	ctx.gl_context = SDL_GL_CreateContext(ctx.window);
	if (ctx.gl_context == NULL) {
		ULOGE("SDL_GL_CreateContext: %s", SDL_GetError());
		return EXIT_FAILURE;
	}

	if (overlay_new(DETECTIONS_METADATA_UID, &ctx.overlay) < 0) {
		ULOGE("overlay_new failed");
		return EXIT_FAILURE;
	}

	if (pdraw_be_new(&be_cbs, &ctx, &ctx.pdraw) < 0) {
		ULOGE("pdraw_be_new failed");
		return EXIT_FAILURE;
	}

	struct pdraw_demuxer_params demuxer_params;
	memset(&demuxer_params, 0, sizeof(demuxer_params));
	int res = pdraw_be_demuxer_new_from_url(ctx.pdraw,
						ctx.url,
						&demuxer_params,
						&demuxer_cbs,
						&ctx,
						&ctx.demuxer);
	if (res < 0) {
		ULOG_ERRNO("pdraw_be_demuxer_new_from_url", -res);
		return EXIT_FAILURE;
	}

	while (!ctx.stopped) {
		/* Begin the graceful close once requested (Ctrl+C/quit/error).
		 * Done here so the renderer is destroyed on this GL thread. */
		if (ctx.close_requested && !ctx.closing)
			request_close(&ctx);

		/* Create the renderer here (GL context current on this thread)
		 * once media_added_cb has recorded a video media. */
		if (!ctx.closing && ctx.renderer == NULL && ctx.media_id != 0)
			create_renderer(&ctx);

		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT ||
			    (event.type == SDL_KEYDOWN &&
			     event.key.keysym.sym == SDLK_ESCAPE))
				ctx.close_requested = 1;
		}

		if (ctx.renderer != NULL) {
			glViewport(0, 0, WIN_W, WIN_H);
			glClearColor(0.f, 0.f, 0.f, 1.f);
			glClear(GL_COLOR_BUFFER_BIT);
			res = pdraw_be_video_renderer_render(ctx.pdraw,
							     ctx.renderer,
							     NULL);
			if (res < 0)
				ULOG_ERRNO("pdraw_be_video_renderer_render",
					   -res);
			SDL_GL_SwapWindow(ctx.window);
		}
		SDL_Delay(5);
	}

	/* Normally already destroyed by the graceful close; safety nets for
	 * early-exit paths. */
	if (ctx.renderer != NULL)
		pdraw_be_video_renderer_destroy(ctx.pdraw, ctx.renderer);
	if (ctx.demuxer != NULL)
		pdraw_be_demuxer_destroy(ctx.pdraw, ctx.demuxer);
	if (ctx.pdraw != NULL) {
		pdraw_be_stop(ctx.pdraw);
		pdraw_be_destroy(ctx.pdraw);
	}
	overlay_destroy(ctx.overlay);
	SDL_GL_DeleteContext(ctx.gl_context);
	SDL_DestroyWindow(ctx.window);
	SDL_Quit();
	return EXIT_SUCCESS;
}
