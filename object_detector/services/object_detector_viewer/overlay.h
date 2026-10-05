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
 * Detection-box overlay for the object_detector viewer.
 *
 * Reads the per-frame detections that the object_detector mission publishes as
 * Air SDK stream user metadata (carried in the video stream's vmeta protobuf,
 * user[] field, keyed by the Jenkins hash of the mission's dotless metadata
 * UID, not its mission UID) and draws the boxes over the rendered video with
 * OpenGL ES2.
 *
 * Call overlay_render() from the pdraw video renderer's render_overlay
 * callback, which runs on the rendering thread with a current GL context.
 */
#ifndef OBJECT_DETECTOR_VIEWER_OVERLAY_H
#define OBJECT_DETECTOR_VIEWER_OVERLAY_H

#include <pdraw/pdraw_defs.h>
#include <video-metadata/vmeta.h>

#ifdef __cplusplus
extern "C" {
#endif

struct overlay;

/**
 * Create an overlay renderer.
 * @param metadata_uid: the object_detector mission's dotless metadata UID
 *                       (not its mission UID); its Jenkins hash selects the
 *                       matching vmeta user-metadata entry.
 * @param ret: [out] created overlay.
 * @return 0 on success, negative errno on error.
 */
int overlay_new(const char *metadata_uid, struct overlay **ret);

void overlay_destroy(struct overlay *self);

/**
 * Draw the detection boxes for the current frame.
 * @param self: overlay.
 * @param render_pos: renderer viewport (pixels) — from render_overlay.
 * @param content_pos: video content rectangle within the viewport (pixels).
 * @param frame_meta: frame metadata (vmeta) — from render_overlay; may be NULL.
 */
void overlay_render(struct overlay *self,
		    const struct pdraw_rect *render_pos,
		    const struct pdraw_rect *content_pos,
		    struct vmeta_frame *frame_meta);

#ifdef __cplusplus
}
#endif

#endif /* OBJECT_DETECTOR_VIEWER_OVERLAY_H */
