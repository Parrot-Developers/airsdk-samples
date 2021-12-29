/**
 * Copyright (C) 2021 Parrot Drones SAS
 */

#pragma once

struct processing;

struct processing_input {
	const struct vipc_frame *frame;

	struct {
		float x;
		float y;
		float z;
	} position_global;
};

struct processing_output {
	struct timespec ts;
	float x, y, z;
	float depth_mean;
	float confidence;
};

/**
 * Create a processing object.
 * @param evt pomp event to be used to notify main loop when a processing
 *            step is completed
 * @param ret_obj pointer to return ed object.
 * @return 0 in case of success, negative errno in case of error.
 */
int processing_new(struct pomp_evt *evt, struct processing **ret_obj);

/**
 * Delete a processing object.
 * @param self processing object.
 */
void processing_destroy(struct processing *self);

/**
 * Start background thread for processing.
 * @param self processing object.
 */
int processing_start(struct processing *self);

/**
 * Stop background thread for processing.
 * @param self processing object.
 */
void processing_stop(struct processing *self);

/**
 * Execute a step of processing. It will delegate the task to a background
 * thread. When done the pomp event given at creation will be signaled, meaning
 * `processing_get_output` can be called to retrieve the result.
 * @param self processing object.
 * @param input: input data including video frame.
 * @return 0 in case of success, negative errno in case of error.
 *
 * @remarks in case of success, the ownership of the frame is transferred to the
 * processing object, otherwise the caller still owns it ans shall release it.
 */
int processing_step(struct processing *self,
		    const struct processing_input *input);

/**
 * Get output of processing step. Shall be called by the main loop when the
 * pomp event given at creation i s signaled.
 * @param self processing object.
 * @return 0 in case of success, negative errno in case of error.
 */
int processing_get_output(struct processing *self,
			  struct processing_output *output);
