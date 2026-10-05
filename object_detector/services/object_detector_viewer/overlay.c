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
 * Detection-box overlay, see overlay.h.
 */

#include "overlay.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <GLES2/gl2.h>

#define ULOG_TAG object_detector_viewer_overlay
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include <video-metadata/vmeta_frame_proto.h>

#include "detection.pb-c.h"
#include "font8x8.h"

/* Max boxes drawn per frame (matches the mission's cap). */
#define MAX_BOXES 10

/* Font atlas: FONT_COUNT 8x8 glyphs laid out 16 columns x 6 rows. */
#define ATLAS_COLS 16
#define ATLAS_ROWS 6
#define ATLAS_W (ATLAS_COLS * 8)
#define ATLAS_H (ATLAS_ROWS * 8)

struct overlay {
	uint32_t uid_hash; /* Jenkins hash of the detections metadata UID */

	GLuint box_program;
	GLint box_pos_loc;   /* attribute vec2 aPos (NDC) */
	GLint box_color_loc; /* uniform vec4 uColor */

	GLuint text_program;
	GLint text_pos_loc;   /* attribute vec2 aPos (NDC) */
	GLint text_uv_loc;    /* attribute vec2 aUV */
	GLint text_color_loc; /* uniform vec4 uColor */
	GLint text_tex_loc;   /* uniform sampler2D uTex */
	GLuint font_tex;      /* 8x8 font atlas (GL_ALPHA) */
};

/* Jenkins one-at-a-time hash, as used for vmeta UserMetadata.uid_hash. */
static uint32_t jenkins_oaat(const char *key)
{
	uint32_t h = 0;
	for (; *key != '\0'; key++) {
		h += (uint8_t)*key;
		h += h << 10;
		h ^= h >> 6;
	}
	h += h << 3;
	h ^= h >> 11;
	h += h << 15;
	return h;
}

static GLuint compile_shader(GLenum type, const char *src)
{
	GLuint s = glCreateShader(type);
	glShaderSource(s, 1, &src, NULL);
	glCompileShader(s);
	GLint ok = 0;
	glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
	if (!ok) {
		char log[256] = "";
		glGetShaderInfoLog(s, sizeof(log), NULL, log);
		ULOGE("shader compile failed: %s", log);
		glDeleteShader(s);
		return 0;
	}
	return s;
}

/* Link a program from vertex + fragment sources. Returns 0 on failure. */
static GLuint link_program(const char *vs_src, const char *fs_src)
{
	GLuint v = compile_shader(GL_VERTEX_SHADER, vs_src);
	GLuint f = compile_shader(GL_FRAGMENT_SHADER, fs_src);
	if (v == 0 || f == 0)
		return 0;
	GLuint p = glCreateProgram();
	glAttachShader(p, v);
	glAttachShader(p, f);
	glLinkProgram(p);
	glDeleteShader(v);
	glDeleteShader(f);
	return p;
}

/* Upload the 8x8 font as a single-channel (alpha) texture atlas. */
static GLuint make_font_texture(void)
{
	unsigned char atlas[ATLAS_W * ATLAS_H];
	memset(atlas, 0, sizeof(atlas));
	for (int idx = 0; idx < FONT_COUNT; idx++) {
		int gx = idx % ATLAS_COLS;
		int gy = idx / ATLAS_COLS;
		for (int r = 0; r < 8; r++) {
			unsigned char bits = font8x8[idx][r];
			for (int c = 0; c < 8; c++) {
				if (bits & (1 << c))
					atlas[(gy * 8 + r) * ATLAS_W +
					      (gx * 8 + c)] = 0xFF;
			}
		}
	}

	GLuint tex = 0;
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, ATLAS_W, ATLAS_H, 0, GL_ALPHA,
		     GL_UNSIGNED_BYTE, atlas);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	return tex;
}

int overlay_new(const char *metadata_uid, struct overlay **ret)
{
	if (metadata_uid == NULL || ret == NULL)
		return -EINVAL;

	struct overlay *self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->uid_hash = jenkins_oaat(metadata_uid);

	/* Solid-color program for the boxes. */
	self->box_program = link_program(
		"attribute vec2 aPos;\n"
		"void main() { gl_Position = vec4(aPos, 0.0, 1.0); }\n",
		"precision mediump float;\n"
		"uniform vec4 uColor;\n"
		"void main() { gl_FragColor = uColor; }\n");

	/* Textured program for the labels (font atlas sampled as alpha). */
	self->text_program = link_program(
		"attribute vec2 aPos;\n"
		"attribute vec2 aUV;\n"
		"varying vec2 vUV;\n"
		"void main() { vUV = aUV; gl_Position = vec4(aPos, 0.0, 1.0); }\n",
		"precision mediump float;\n"
		"uniform sampler2D uTex;\n"
		"uniform vec4 uColor;\n"
		"varying vec2 vUV;\n"
		"void main() {\n"
		"  float a = texture2D(uTex, vUV).a;\n"
		"  gl_FragColor = vec4(uColor.rgb, uColor.a * a);\n"
		"}\n");

	if (self->box_program == 0 || self->text_program == 0) {
		overlay_destroy(self);
		return -EIO;
	}

	self->box_pos_loc = glGetAttribLocation(self->box_program, "aPos");
	self->box_color_loc = glGetUniformLocation(self->box_program, "uColor");
	self->text_pos_loc = glGetAttribLocation(self->text_program, "aPos");
	self->text_uv_loc = glGetAttribLocation(self->text_program, "aUV");
	self->text_color_loc =
		glGetUniformLocation(self->text_program, "uColor");
	self->text_tex_loc = glGetUniformLocation(self->text_program, "uTex");

	self->font_tex = make_font_texture();

	*ret = self;
	return 0;
}

void overlay_destroy(struct overlay *self)
{
	if (self == NULL)
		return;
	if (self->box_program != 0)
		glDeleteProgram(self->box_program);
	if (self->text_program != 0)
		glDeleteProgram(self->text_program);
	if (self->font_tex != 0)
		glDeleteTextures(1, &self->font_tex);
	free(self);
}

/* Pixel -> NDC (top-left origin metadata, GL bottom-left origin). */
#define NDC_X(px) (2.f * ((px) - render_pos->x) / render_pos->width - 1.f)
#define NDC_Y(py) (1.f - 2.f * ((py) - render_pos->y) / render_pos->height)

/* Draw one normalized box [left,top,right,bottom] as a solid-color line
 * loop, mapping it to the video content rectangle then to NDC. */
static void draw_box(struct overlay *self,
		     const struct pdraw_rect *render_pos,
		     const struct pdraw_rect *content_pos,
		     const ObjectDetector__Metadata__Box *box,
		     float r, float g, float b, float line_width)
{
	float x0 = content_pos->x + box->left * content_pos->width;
	float x1 = content_pos->x + box->right * content_pos->width;
	float y0 = content_pos->y + box->top * content_pos->height;
	float y1 = content_pos->y + box->bottom * content_pos->height;

	const GLfloat verts[] = {
		NDC_X(x0), NDC_Y(y0), NDC_X(x1), NDC_Y(y0),
		NDC_X(x1), NDC_Y(y1), NDC_X(x0), NDC_Y(y1),
	};

	glUseProgram(self->box_program);
	glDisable(GL_DEPTH_TEST);
	glUniform4f(self->box_color_loc, r, g, b, 1.f);
	glLineWidth(line_width);
	glEnableVertexAttribArray(self->box_pos_loc);
	glVertexAttribPointer(self->box_pos_loc, 2, GL_FLOAT, GL_FALSE, 0,
			      verts);
	glDrawArrays(GL_LINE_LOOP, 0, 4);
	glDisableVertexAttribArray(self->box_pos_loc);
}

/* Draw @p str with the font atlas, top-left at pixel (px, py), each glyph
 * scale*8 pixels. */
static void draw_text(struct overlay *self,
		      const struct pdraw_rect *render_pos,
		      float px,
		      float py,
		      float scale,
		      const char *str,
		      float r,
		      float g,
		      float b)
{
	size_t len = strlen(str);
	if (len == 0)
		return;
	if (len > 63)
		len = 63; /* bounds the stack buffer below */

	/* 6 vertices per glyph, each {x, y, u, v}. */
	GLfloat verts[63 * 6 * 4];
	size_t vi = 0;
	const float cell = 8.f * scale; /* glyph cell size in pixels */

	for (size_t i = 0; i < len; i++) {
		unsigned char ch = (unsigned char)str[i];
		if (ch < FONT_FIRST || ch >= FONT_FIRST + FONT_COUNT)
			ch = '?';
		int idx = ch - FONT_FIRST;
		int gx = idx % ATLAS_COLS;
		int gy = idx / ATLAS_COLS;
		float u0 = (float)(gx * 8) / ATLAS_W;
		float u1 = (float)(gx * 8 + 8) / ATLAS_W;
		float v0 = (float)(gy * 8) / ATLAS_H;     /* glyph top */
		float v1 = (float)(gy * 8 + 8) / ATLAS_H; /* glyph bottom */

		float x0 = px + (float)i * cell, x1 = x0 + cell;
		float y0 = py, y1 = py + cell;
		float nx0 = NDC_X(x0), nx1 = NDC_X(x1);
		float ny0 = NDC_Y(y0), ny1 = NDC_Y(y1);

		const GLfloat quad[6 * 4] = {
			nx0, ny0, u0, v0, nx1, ny0, u1, v0,
			nx0, ny1, u0, v1, nx1, ny0, u1, v0,
			nx1, ny1, u1, v1, nx0, ny1, u0, v1,
		};
		for (int k = 0; k < 6 * 4; k++)
			verts[vi++] = quad[k];
	}

	glUseProgram(self->text_program);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, self->font_tex);
	glUniform1i(self->text_tex_loc, 0);
	glUniform4f(self->text_color_loc, r, g, b, 1.f);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnableVertexAttribArray(self->text_pos_loc);
	glEnableVertexAttribArray(self->text_uv_loc);
	glVertexAttribPointer(self->text_pos_loc, 2, GL_FLOAT, GL_FALSE,
			      4 * sizeof(GLfloat), verts);
	glVertexAttribPointer(self->text_uv_loc, 2, GL_FLOAT, GL_FALSE,
			      4 * sizeof(GLfloat), verts + 2);
	glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(len * 6));
	glDisableVertexAttribArray(self->text_pos_loc);
	glDisableVertexAttribArray(self->text_uv_loc);
	glDisable(GL_BLEND);
}

void overlay_render(struct overlay *self,
		    const struct pdraw_rect *render_pos,
		    const struct pdraw_rect *content_pos,
		    struct vmeta_frame *frame_meta)
{
	if (self == NULL || render_pos == NULL || content_pos == NULL)
		return;
	if (frame_meta == NULL || frame_meta->type != VMETA_FRAME_TYPE_PROTO)
		return;

	const Vmeta__TimedMetadata *tm = NULL;
	if (vmeta_frame_proto_get_unpacked(frame_meta, &tm) < 0 || tm == NULL)
		return;

	/* Find our mission's user-metadata entry and draw its detections. */
	for (size_t i = 0; i < tm->n_user; i++) {
		if (tm->user[i]->uid_hash != self->uid_hash)
			continue;

		ObjectDetector__Metadata__Detections *dets =
			object_detector__metadata__detections__unpack(
				NULL, tm->user[i]->data.len,
				tm->user[i]->data.data);
		if (dets == NULL)
			continue;

		size_t n = dets->n_detections;
		if (n > MAX_BOXES)
			n = MAX_BOXES;
		for (size_t j = 0; j < n; j++) {
			const ObjectDetector__Metadata__Detection *d =
				dets->detections[j];
			if (d->box == NULL)
				continue;

			/* Blue: neutral, no "warning" connotation like red would have. */
			const float r = 0.2f, g = 0.6f, b = 1.f;
			draw_box(self, render_pos, content_pos, d->box, r, g, b, 2.f);

			/* "<label> <score>%" just above the box. */
			char txt[64];
			int pct = (int)(d->score * 100.f + 0.5f);
			const char *label =
				(d->label && d->label[0]) ? d->label : "";
			snprintf(txt, sizeof(txt), "%s %d%%", label, pct);

			const float scale = 2.f;
			float tw = (float)strlen(txt) * 8.f * scale;
			float th = 8.f * scale;
			float bx = content_pos->x + d->box->left *
							    content_pos->width;
			float by = content_pos->y +
				   d->box->top * content_pos->height;
			float tx = bx;
			float ty = by - th - 2.f; /* just above the box */

			/* Keep the label fully on screen for cropped/edge boxes. */
			float rx0 = render_pos->x;
			float rx1 = render_pos->x + render_pos->width;
			float ry0 = render_pos->y;
			float ry1 = render_pos->y + render_pos->height;
			if (tx + tw > rx1)
				tx = rx1 - tw;
			if (tx < rx0)
				tx = rx0;
			if (ty < ry0)
				ty = ry0; /* box cropped at top: pin to top edge */
			if (ty + th > ry1)
				ty = ry1 - th;

			draw_text(self, render_pos, tx, ty, scale, txt, r, g, b);
		}
		object_detector__metadata__detections__free_unpacked(dets, NULL);
	}

	vmeta_frame_proto_release_unpacked(frame_meta, tm);
}
