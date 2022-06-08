/**
 * Copyright (C) 2021 Parrot Drones SAS
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#define ULOG_TAG ms_processing
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include <libpomp.h>
#include <video-ipc/vipc_client.h>

#include <opencv2/opencv.hpp>

#include "processing.h"

// HIST_RANGE_LOW < HIST_RANGE_HIGH
#define HIST_RANGE_LOW 0.f                            // [m]
#define HIST_RANGE_HIGH 15.f                          // [m]
#define HIST_RANGE (HIST_RANGE_HIGH - HIST_RANGE_LOW) // [m]
#define HIST_SIZE 10

struct processing {
	struct pomp_evt *evt;
	bool started;
	bool stop_requested;
	pthread_t thread;
	pthread_mutex_t mutex;
	pthread_cond_t cond;

	struct processing_input input;
	bool input_available;

	struct processing_output output;
	bool output_available;
};

static void do_step(struct processing *self,
		    const struct processing_input *input,
		    struct processing_output *output)
{
	const int hist_size = HIST_SIZE;
	const float range_tuple[] = {HIST_RANGE_LOW, HIST_RANGE_HIGH};
	const float *ranges = {range_tuple};
	const float bin_size = HIST_RANGE / HIST_SIZE;
	const cv::Mat depth_frame(input->frame->height,
				  input->frame->width,
				  CV_32F,
				  (void *)input->frame->planes[0].virt_addr,
				  input->frame->planes[0].stride);
	cv::Mat mask_frame(
		input->frame->height, input->frame->width, CV_8UC1, 1);
	cv::Mat hist;
	int hist_max_index;
	float depth_mean;

	/* Initialize mask: reject negative and infinity depth pixels */
	for (int i = 0; i < depth_frame.rows; ++i) {
		for (int j = 0; j < depth_frame.cols; ++j) {
			float depth = depth_frame.at<float>(i, j);
			if (depth < 0.f || depth == INFINITY)
				mask_frame.at<uint8_t>(i, j) = 0;
		}
	}

	/* Compute histogram */
	cv::calcHist(&depth_frame,
		     1 /* image count */,
		     nullptr /* channels */,
		     mask_frame,
		     hist,
		     1, /* dimensions */
		     &hist_size,
		     &ranges);

	/* Find histogram index with highest count */
	cv::minMaxIdx(hist, nullptr, nullptr, nullptr, &hist_max_index);

	float low = HIST_RANGE_LOW + hist_max_index * bin_size;
	float high = low + bin_size;

	/* Reject depth pixels not in the bin with most frequent depth range */
	for (int i = 0; i < depth_frame.rows; ++i)
		for (int j = 0; j < depth_frame.cols; ++j) {
			float depth = depth_frame.at<float>(i, j);
			if (depth < low || depth > high)
				mask_frame.at<uint8_t>(i, j) = 0;
		}

	/* Compute mean of remaining depth pixels */
	depth_mean = cv::mean(depth_frame, mask_frame).val[0];

	ULOGD("depth_mean: %f", depth_mean);

	/* Fill output */
	output->x = input->position_global.x;
	output->y = input->position_global.y;
	output->z = input->position_global.z;
	output->depth_mean = depth_mean;
	output->confidence = 1.0f;

	/* Save timestamp of the frame */
	output->ts.tv_sec = input->frame->ts_sof_ns / 1000000000UL;
	output->ts.tv_nsec = input->frame->ts_sof_ns % 1000000000UL;
}

static void *thread_entry(void *userdata)
{
	int res = 0;
	struct processing *self = (struct processing *)userdata;
	struct processing_input local_input;
	struct processing_output local_output;

	pthread_mutex_lock(&self->mutex);

	while (!self->stop_requested) {
		/* Atomically unlock the mutex, wait for condition and then
		  re-lock the mutex when condition is signaled */
		res = pthread_cond_wait(&self->cond, &self->mutex);
		if (res != 0) {
			ULOG_ERRNO("pthread_cond_wait", res);
			continue;
		}

		/* Check booleans in case of spurious wakeup */
		if (self->stop_requested)
			break;
		if (!self->input_available)
			continue;

		/* Copy locally input data */
		local_input = self->input;
		memset(&self->input, 0, sizeof(self->input));
		self->input_available = false;

		/* Do the heavy computation outside lock */
		pthread_mutex_unlock(&self->mutex);
		memset(&local_output, 0, sizeof(local_output));
		do_step(self, &local_input, &local_output);
		pthread_mutex_lock(&self->mutex);

		/* Done with the input frame */
		vipcc_release_safe(local_input.frame);
		memset(&local_input, 0, sizeof(local_input));

		/* Copy output data */
		self->output = local_output;
		self->output_available = true;

		/* Notify main loop that result is available */
		res = pomp_evt_signal(self->evt);
		if (res < 0)
			ULOG_ERRNO("pomp_evt_signal", -res);
	}

	pthread_mutex_unlock(&self->mutex);

	return NULL;
}

int processing_new(struct pomp_evt *evt, struct processing **ret_obj)
{
	struct processing *self = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	*ret_obj = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(evt == NULL, EINVAL);

	self = (struct processing *)calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->evt = evt;
	pthread_mutex_init(&self->mutex, NULL);
	pthread_cond_init(&self->cond, NULL);

	*ret_obj = self;
	return 0;
}

void processing_destroy(struct processing *self)
{
	if (self == NULL)
		return;

	processing_stop(self);

	pthread_mutex_destroy(&self->mutex);
	pthread_cond_destroy(&self->cond);

	free(self);
}

int processing_start(struct processing *self)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->started, EBUSY);

	/* Create background thread */
	self->stop_requested = false;
	res = pthread_create(&self->thread, NULL, &thread_entry, self);
	if (res != 0) {
		ULOG_ERRNO("pthread_create", res);
		return -res;
	}
	self->started = true;

	return 0;
}

void processing_stop(struct processing *self)
{
	if (self == NULL || !self->started)
		return;

	/* Ask thread to stop */
	pthread_mutex_lock(&self->mutex);
	self->stop_requested = true;
	pthread_cond_signal(&self->cond);
	pthread_mutex_unlock(&self->mutex);

	/* Wait for thread and release resources */
	pthread_join(self->thread, NULL);
	self->started = false;

	/* Cleanup remaining input data if any */
	pthread_mutex_lock(&self->mutex);
	if (self->input_available) {
		vipcc_release_safe(self->input.frame);
		memset(&self->input, 0, sizeof(self->input));
		self->input_available = false;
	}
	pthread_mutex_unlock(&self->mutex);
}

int processing_step(struct processing *self,
		    const struct processing_input *input)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!self->started, EPERM);

	pthread_mutex_lock(&self->mutex);

	/* If an input is already pending, release it before overwrite */
	if (self->input_available) {
		vipcc_release_safe(self->input.frame);
		memset(&self->input, 0, sizeof(self->input));
		self->input_available = false;
	}

	/* Copy input data and take ownership of frame */
	self->input = *input;
	self->input_available = true;

	/* Wakeup background thread */
	res = pthread_cond_signal(&self->cond);
	if (res != 0) {
		/* pthread return a positive errno value */
		ULOG_ERRNO("pthread_cond_signal", res);
		res = -res;

		/* Do not take frame and give it back to caller */
		memset(&self->input, 0, sizeof(self->input));
		self->input_available = false;
	}

	pthread_mutex_unlock(&self->mutex);

	return 0;
}

int processing_get_output(struct processing *self,
			  struct processing_output *output)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!self->started, EPERM);

	pthread_mutex_lock(&self->mutex);

	if (!self->output_available) {
		res = -ENOENT;
	} else {
		*output = self->output;
		self->output_available = false;
	}

	pthread_mutex_unlock(&self->mutex);

	return res;
}
