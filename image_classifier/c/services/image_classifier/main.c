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
 * Image classifier example in C: consumes camera frames, preprocesses them
 * (NV12 -> normalized RGB 224x224) and runs a non-blocking airsdk_nn_infer().
 * A worker thread pulls the freshest frame and does the (CPU-heavy)
 * preprocessing off the loop, so frame reception is never blocked; the top-5
 * result comes back on the loop in on_infer_result(). Results are logged.
 */

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libpomp.h>

#include <airsdk/neural_network.h>
#include <airsdk/neural_network/input.h>

#define ULOG_TAG image_classifier_c
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#define UID "com.parrot.missions.samples.image_classifier_c"
#define BASE_PATH "/mnt/missions/" UID "/payload/etc/services/classification"
#define MODEL_FILE BASE_PATH "/ResNet50_quantized.dlc"
#define LABELS_FILE BASE_PATH "/classes_names.csv"
#define STREAM_NAME "fcam_airsdk"
/* Steady-state timeout: infer()'s first call is floored to 30s internally by
 * the SDK to cover the model load, so this only needs to size normal
 * inference latency. */
#define INFER_TIMEOUT_MS 5000
#define INPUT_DIM 224
#define DEFAULT_NUM_CLASSES 1000

static volatile sig_atomic_t s_run = 1;

static struct airsdk_nn *s_nn;
static struct airsdk_nn_input_source *s_src;

/* Network readiness, set on the loop by on_status, read by the worker. */
static atomic_bool s_ready;

/* Class labels loaded from the CSV asset. */
static char **s_labels;
static size_t s_n_labels;

/* Reusable HWC RGB input tensor (filled by the worker). */
static float s_tensor[INPUT_DIM * INPUT_DIM * 3];

static void sig_handler(int sig)
{
	(void)sig;
	s_run = 0;
}

static void on_status(struct airsdk_nn *nn, bool is_ready, void *userdata)
{
	(void)nn;
	(void)userdata;
	atomic_store(&s_ready, is_ready);
	ULOGI("network %s", is_ready ? "ready" : "not ready");
	/* Some missions want the Parrot on-board networks off the NPU they
	 * use, for performance. */
	if (is_ready)
		airsdk_nn_deactivate_native_nns(s_nn, true);
}

static void on_error(struct airsdk_nn *nn, const char *reason, void *userdata)
{
	(void)nn;
	(void)userdata;
	ULOGW("network error: %s", reason ? reason : "?");
}

static void load_labels(void)
{
	FILE *f = fopen(LABELS_FILE, "r");
	if (!f) {
		ULOGW("cannot open %s", LABELS_FILE);
		return;
	}
	char line[256];
	size_t cap = 0;
	while (fgets(line, sizeof(line), f)) {
		size_t n = strlen(line);
		while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'))
			line[--n] = '\0';
		if (s_n_labels == cap) {
			cap = cap ? cap * 2 : 1024;
			s_labels = realloc(s_labels, cap * sizeof(*s_labels));
		}
		s_labels[s_n_labels++] = strdup(line);
	}
	fclose(f);
	ULOGI("loaded %zu class names", s_n_labels);
}

static void free_labels(void)
{
	for (size_t i = 0; i < s_n_labels; i++)
		free(s_labels[i]);
	free(s_labels);
}

/* NV12 -> RGB (BT.601) at integer source pixel (x,y); output range 0..255. */
static void nv12_rgb(const uint8_t *y,
		     const uint8_t *uv,
		     uint32_t ys,
		     uint32_t uvs,
		     int x,
		     int py,
		     float rgb[3])
{
	float yy = y[py * ys + x];
	float u = (float)uv[(py / 2) * uvs + (x & ~1) + 0] - 128.f;
	float v = (float)uv[(py / 2) * uvs + (x & ~1) + 1] - 128.f;
	rgb[0] = yy + 1.402f * v;
	rgb[1] = yy - 0.344136f * u - 0.714136f * v;
	rgb[2] = yy + 1.772f * u;
}

/* Fill dst (INPUT_DIM*INPUT_DIM*3 floats, HWC RGB, ImageNet-normalized) from an
 * NV12 sample: center square crop, bilinear resize, normalize. */
static int preprocess(const struct airsdk_nn_input_sample *s, float *dst)
{
	if (s->n_planes < 2 || s->width == 0 || s->height == 0)
		return -EINVAL;

	const uint8_t *yb = s->data + s->planes[0].offset;
	const uint8_t *uvb = s->data + s->planes[1].offset;
	uint32_t ys = s->planes[0].stride;
	uint32_t uvs = s->planes[1].stride;
	uint32_t crop = s->width < s->height ? s->width : s->height;
	uint32_t x0 = (s->width - crop) / 2;
	uint32_t y0 = (s->height - crop) / 2;

	static const float mean[3] = {0.485f, 0.456f, 0.406f};
	static const float stddev[3] = {0.229f, 0.224f, 0.225f};

	for (int oy = 0; oy < INPUT_DIM; oy++) {
		float sy = (oy + 0.5f) * crop / INPUT_DIM - 0.5f;
		int y1 = (int)floorf(sy);
		float dy = sy - y1;
		int y2 = y1 + 1;
		if (y1 < 0)
			y1 = 0;
		if (y2 > (int)crop - 1)
			y2 = crop - 1;

		for (int ox = 0; ox < INPUT_DIM; ox++) {
			float sx = (ox + 0.5f) * crop / INPUT_DIM - 0.5f;
			int x1 = (int)floorf(sx);
			float dx = sx - x1;
			int x2 = x1 + 1;
			if (x1 < 0)
				x1 = 0;
			if (x2 > (int)crop - 1)
				x2 = crop - 1;

			float c00[3], c01[3], c10[3], c11[3];
			nv12_rgb(yb, uvb, ys, uvs, x0 + x1, y0 + y1, c00);
			nv12_rgb(yb, uvb, ys, uvs, x0 + x2, y0 + y1, c01);
			nv12_rgb(yb, uvb, ys, uvs, x0 + x1, y0 + y2, c10);
			nv12_rgb(yb, uvb, ys, uvs, x0 + x2, y0 + y2, c11);

			float *out = &dst[(oy * INPUT_DIM + ox) * 3];
			for (int c = 0; c < 3; c++) {
				float top = c00[c] + (c01[c] - c00[c]) * dx;
				float bot = c10[c] + (c11[c] - c10[c]) * dx;
				float val = top + (bot - top) * dy;
				if (val < 0.f)
					val = 0.f;
				if (val > 255.f)
					val = 255.f;
				out[c] = (val / 255.f - mean[c]) / stddev[c];
			}
		}
	}
	return 0;
}

static void log_top5(const float *scores, uint32_t n)
{
	float maxv = scores[0];
	for (uint32_t i = 1; i < n; i++)
		if (scores[i] > maxv)
			maxv = scores[i];

	double sum = 0.0;
	for (uint32_t i = 0; i < n; i++)
		sum += exp((double)(scores[i] - maxv));

	bool picked[5] = {false};
	int top[5];
	for (int k = 0; k < 5; k++) {
		int best = -1;
		for (uint32_t i = 0; i < n; i++) {
			bool used = false;
			for (int j = 0; j < k; j++)
				if (top[j] == (int)i)
					used = true;
			if (used)
				continue;
			if (best < 0 || scores[i] > scores[best])
				best = (int)i;
		}
		top[k] = best;
		picked[k] = best >= 0;
	}

	for (int k = 0; k < 5 && picked[k]; k++) {
		double prob = exp((double)(scores[top[k]] - maxv)) / sum;
		const char *name = ((size_t)top[k] < s_n_labels)
					   ? s_labels[top[k]]
					   : "unknown";
		ULOGI("Top%d: %s (%.1f%%)", k + 1, name, prob * 100.0);
	}
}

/* Worker thread: pull the freshest frame, preprocess it off the loop and fire
 * a non-blocking inference. */
static void *worker(void *arg)
{
	(void)arg;

	while (s_run) {
		struct airsdk_nn_input_sample sample;
		if (airsdk_nn_input_wait_sample(s_src, 1000, &sample) != 0)
			continue; /* timeout: the source reconnects itself */
		if (!atomic_load(&s_ready))
			continue; /* network not ready yet */

		if (preprocess(&sample, s_tensor) != 0) {
			ULOGE("preprocessing failed");
			continue;
		}

		struct airsdk_nn_infer_input in = {
			.attributes = {.layer_name = "input",
				       .data_name = "input",
				       .num = 1,
				       .height = INPUT_DIM,
				       .width = INPUT_DIM,
				       .channel = 3},
			.data = s_tensor,
			.size = sizeof(s_tensor)};

		uint32_t out_size = (s_n_labels ? s_n_labels
						: DEFAULT_NUM_CLASSES) *
				    sizeof(float);
		/* Non-blocking: airsdk_nn_infer copies the input and returns
		 * immediately; the result arrives on the loop in
		 * on_infer_result. -EBUSY means a previous inference is still
		 * running, so we drop this frame (one at a time). */
		int rc = airsdk_nn_infer(s_nn, &in, 1, out_size,
					 sample.timestamp_ns);
		if (rc < 0 && rc != -EBUSY)
			ULOGW("infer failed: %d", rc);
	}
	return NULL;
}

/* Inference result, delivered on the loop. */
static void on_infer_result(struct airsdk_nn *nn,
			    const struct airsdk_nn_output *outputs,
			    size_t n_outputs,
			    uint64_t timestamp_ns,
			    void *userdata)
{
	(void)nn;
	(void)timestamp_ns;
	(void)userdata;
	if (n_outputs == 0)
		return;
	log_top5((const float *)outputs[0].data, outputs[0].attributes.channel);
}

static void on_infer_error(struct airsdk_nn *nn,
			   int error,
			   uint64_t timestamp_ns,
			   void *userdata)
{
	(void)nn;
	(void)timestamp_ns;
	(void)userdata;
	ULOGW("inference failed: %d", error);
}

int main(void)
{
	signal(SIGTERM, sig_handler);
	signal(SIGINT, sig_handler);
	ULOGI("Starting image_classifier_c mission");

	load_labels();

	struct pomp_loop *loop = pomp_loop_new();
	if (!loop) {
		ULOGE("pomp_loop_new failed");
		return -1;
	}

	if (airsdk_nn_input_make_vipc(&s_src, loop, STREAM_NAME) != 0) {
		ULOGE("failed to create vipc input source");
		return -1;
	}

	const char *outputs[] = {"/fc/Gemm"};
	struct airsdk_nn_settings settings = {.net_id = "airsdk@ResNet50",
					      .model_file = MODEL_FILE,
					      .output_layers = outputs,
					      .n_output_layers = 1};
	/* settings.diag_log_folder left NULL (disabled): set to BASE_PATH only
	 * when debugging, it adds per-inference I/O overhead. */
	struct airsdk_nn_listener listener = {.status_cb = on_status,
					      .error_cb = on_error,
					      .infer_result_cb = on_infer_result,
					      .infer_error_cb = on_infer_error,
					      .userdata = NULL};
	if (airsdk_nn_new_with_loop(
		    &s_nn, &settings, loop, &listener, INFER_TIMEOUT_MS) != 0) {
		ULOGE("failed to create neural network");
		return -1;
	}

	pthread_t th;
	pthread_create(&th, NULL, worker, NULL);

	/* Pump the loop: this is where on_status / on_infer_result run. The
	 * finite timeout lets us notice SIGTERM promptly. */
	while (s_run)
		pomp_loop_wait_and_process(loop, 500);

	/* Re-enable native NNs on shutdown, or the drone stays degraded. */
	airsdk_nn_deactivate_native_nns(s_nn, false);

	pthread_join(th, NULL);
	airsdk_nn_destroy(s_nn);
	airsdk_nn_input_destroy(s_src);
	pomp_loop_destroy(loop);
	free_labels();

	ULOGI("Stopping image_classifier_c mission");
	return 0;
}
