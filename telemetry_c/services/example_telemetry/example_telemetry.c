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

#include <errno.h>
#include <time.h>

#include <libtelemetry.h>

#include "example_telemetry.h"

#define ULOG_TAG ex_tlm_c
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#define MAX_SAMPLE 5

/* Union sensor_vector */
union sensor_vector {
	float v[3];
	struct {
		float x;
		float y;
		float z;
	};
};

/**
 * imu_data structure used in the sample consumer example to get
 * some data about the drone
 */
struct sensor_imu_data {
	/* Gyroscope data in SI unit (rad/s) */
	union sensor_vector gyro[MAX_SAMPLE];
	/* Accelerometer data in SI unit (m/s2) */
	union sensor_vector acc[MAX_SAMPLE];
	/* Temperature data in SI unit (K) */
	float temp[MAX_SAMPLE];
	/* fsync information about samples (ns) */
	int32_t fsync[MAX_SAMPLE];
	/* Number of samples in gyroscope & accelerometer fifo */
	uint8_t nb_samples;
	/* Samples timestamps */
	struct timespec timestamp[MAX_SAMPLE];
};

/* Context for the telemetry */
struct example_telemetry_context {

	/* Single sample consumer example */
	struct tlm_consumer *single_sample_consumer;
	float pressure;
	float sensors_tof_distance;
	float sensors_tof_magnitude;
	float sensors_tof_temperature;
	float sensors_tof_ambient;
	int32_t sensors_tof_qualification;

	/* Array sample consumer example */

	struct tlm_consumer *array_sample_consumer_1;
	float altitude_agl[MAX_SAMPLE];
	size_t nb_array_sample_1;
	size_t ref_array_sample_1;

	struct tlm_consumer *array_sample_consumer_2;
	float magnetic_field_ut_x[MAX_SAMPLE];
	float magnetic_field_ut_y[MAX_SAMPLE];
	float magnetic_field_ut_z[MAX_SAMPLE];
	size_t nb_array_sample_2;
	size_t ref_array_sample_2;

	/* Struct sample consumer example */
	struct tlm_consumer *struct_sample_consumer;
	struct sensor_imu_data imu_data;

	/* New value example */
	struct tlm_producer *my_producer;
	int32_t new_value_source;
	struct tlm_consumer *my_consumer;
	int32_t new_value_destination;
};

/* --- this part is used by the struct sample consumer example --- */
static const struct tlm_reg_field sensors_tlm_imu_vec3_fields[] = {
	TLM_REG_FIELD_SCALAR(union sensor_vector, x, TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(union sensor_vector, y, TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(union sensor_vector, z, TLM_TYPE_FLOAT32),
};

static const struct tlm_reg_struct sensors_tlm_imu_acc_reg_struct =
	TLM_REG_STRUCT("acc", sensors_tlm_imu_vec3_fields);

static const struct tlm_reg_struct sensors_tlm_imu_gyro_reg_struct =
	TLM_REG_STRUCT("gyro", sensors_tlm_imu_vec3_fields);

static const struct tlm_reg_field sensors_tlm_imu_fields[] = {
	TLM_REG_FIELD_STRUCT_ARRAY(struct sensor_imu_data,
				   gyro,
				   &sensors_tlm_imu_gyro_reg_struct),
	TLM_REG_FIELD_STRUCT_ARRAY(struct sensor_imu_data,
				   acc,
				   &sensors_tlm_imu_acc_reg_struct),
	TLM_REG_FIELD_SCALAR_ARRAY_EX(struct sensor_imu_data,
				      temp,
				      "temperature",
				      TLM_TYPE_FLOAT32),
};

static const struct tlm_reg_struct sensors_tlm_reg_imu =
	TLM_REG_STRUCT(NULL, sensors_tlm_imu_fields);

/* --- end of this part is used by the struct sample consumer example --- */


static int init_single_sample_consumer_1(struct example_telemetry_context *ctx)
{
	int res;

	/**
	 * There are two methods to register a consumer of single sample:
	 *    - First method in this function.
	 *    - Second method in the init_single_sample_consumer_2 function
	 */

	/* --- FIRST METHOD: tlm_consumer_reg --- */

	/**
	 * Register "pressure" from the sensors_barometer telemetry section.
	 * This telemetry field is a scalar float. Only 1 sample is consumed
	 * with tlm_consumer_reg function. The type of the registered variable
	 * is float.
	 *
	 * Only one variable is registered with this method.
	 */

	res = tlm_consumer_reg(
		ctx->single_sample_consumer,
		&ctx->pressure, // variable to register
		"sensors_barometer.pressure", // section + data name
		TLM_TYPE_FLOAT32, // type float
		sizeof(float), // float size
		1, // 1 element in the variable
		NULL); // no need to store timestamp of samples
	if (res < 0) {
		ULOG_ERRNO("tlm_consumer_reg", -res);
		goto out;
	}

out:
	return res;
}

static int init_single_sample_consumer_2(struct example_telemetry_context *ctx)
{
	int res;

	/**
	 * There are two methods to register a consumer of single sample:
	 *    - First method in the init_single_sample_consumer_1 function
	 *    - Second method in this function.
	 */

	/* --- SECOND METHOD: tlm_consumer_reg_array --- */

	/**
	 * Register several data from the Time of Flight sensor (sensors_tof_0)
	 * telemetry section. These telemetry fields are a scalar float and
	 * int. Only 1 sample is consumed with tlm_consumer_reg_array function.
	 *
	 * Several variables are registered with this method.
	 */

	const struct tlm_consumer_reg_entry sensors_tof_array[] = {
		{.ptr = &ctx->sensors_tof_distance, // variable to register
		 .name = "sensors_tof_0.distance", // section + data name
		 .type = TLM_TYPE_FLOAT32, // type float
		 .size = sizeof(float), // float size
		 .count = 1, // 1 element in the variable
		 .timestamp = NULL}, // no need to store timestamp of samples
		{.ptr = &ctx->sensors_tof_magnitude,
		 .name = "sensors_tof_0.magnitude",
		 .type = TLM_TYPE_FLOAT32,
		 .size = sizeof(float),
		 .count = 1,
		 .timestamp = NULL},
		{.ptr = &ctx->sensors_tof_temperature,
		 .name = "sensors_tof_0.temperature",
		 .type = TLM_TYPE_FLOAT32,
		 .size = sizeof(float),
		 .count = 1,
		 .timestamp = NULL},
		{.ptr = &ctx->sensors_tof_ambient,
		 .name = "sensors_tof_0.ambient",
		 .type = TLM_TYPE_FLOAT32,
		 .size = sizeof(float),
		 .count = 1,
		 .timestamp = NULL},
		{.ptr = &ctx->sensors_tof_qualification, // variable to register
		 .name = "sensors_tof_0.qualification", // section + data name
		 .type = TLM_TYPE_INT32, // type int32
		 .size = sizeof(uint32_t), // int32 size
		 .count = 1, // 1 element in the variable
		 .timestamp = NULL}, // no need to store timestamp of samples
	};

	res = tlm_consumer_reg_array(ctx->single_sample_consumer,
				     sensors_tof_array,
				     ARRAY_SIZE(sensors_tof_array));
	if (res < 0) {
		ULOG_ERRNO("tlm_consumer_reg_array", -res);
		goto out;
	}

	/* --- END OF THE REGISTRATION --- */

	res = tlm_consumer_reg_complete(ctx->single_sample_consumer);
	if (res < 0) {
		ULOG_ERRNO(
			"unable to complete registration of single_sample_consumer",
			-res);
		goto out;
	}

out:
	return res;
}

static int init_array_sample_consumer_1(struct example_telemetry_context *ctx)
{
	int res;

	/**
	 * There are two methods to register a consumer of array of samples:
	 *    - First method in this function.
	 *    - Second method in the init_array_sample_consumer_2 function
	 */

	/* --- FIRST METHOD: tlm_consumer_reg_array_of_samples --- */

	/**
	 * Register the altitude above ground level "altitude_agl" from the
	 * drone_controller telemetry section. This telemetry field is a scalar
	 * float and we want to consume the last MAX_SAMPLE samples. The
	 * type of the registered variable is float[MAX_SAMPLE].
	 *
	 * Only one array of samples is registered with this method.
	 */
	res = tlm_consumer_reg_array_of_samples(
		ctx->array_sample_consumer_1,
		ctx->altitude_agl, // variable to register (array)
		"drone_controller.altitude_agl", // section + data name
		TLM_TYPE_FLOAT32, // type float
		sizeof(float), // float size
		1, // 1 element in the variable
		MAX_SAMPLE, // 10 samples
		NULL); // no need to store timestamp of samples
	if (res < 0) {
		ULOG_ERRNO("tlm_consumer_reg_array_of_samples", -res);
		goto out;
	}

	/* END OF THE REGISTRATION OF array_sample_consumer_1 */

	res = tlm_consumer_reg_complete(ctx->array_sample_consumer_1);
	if (res < 0) {
		ULOG_ERRNO(
			"unable to complete registration of array_sample_consumer_1",
			-res);
		goto out;
	}

out:
	return res;
}

static int init_array_sample_consumer_2(struct example_telemetry_context *ctx)
{
	int res;

	/**
	 * There are two methods to register a consumer of array of samples:
	 *    - First method in the init_array_sample_consumer_1 function
	 *    - Second method in this function.
	 */

	/* --- SECOND METHOD: tlm_consumer_reg_entries_of_samples_array --- */

	/**
	 * Register several data from the magnetometer (sensors_magnetometer)
	 * telemetry section. This telemetry field is a scalar float and we
	 * want to consume the last MAX_SAMPLE samples. The type of the
	 * registered variables is float[MAX_SAMPLE].
	 *
	 * Several arrays of samples are registered with this method.
	 */

	const struct tlm_consumer_reg_samples_entry sensors_magnetometer_array[] = {
		{
			/* variable to register */
			.ptr = ctx->magnetic_field_ut_x,
			/* section + data name */
			.name = "sensors_magnetometer.magnetic_field_ut.x",
			/* type float*/
			.type = TLM_TYPE_FLOAT32,
			/* float size */
			.size = sizeof(float),
			/* 1 element in the variable */
			.count = 1,
			/* 10 samples */
			.nb_samples = MAX_SAMPLE,
			/* no need to store timestamp of samples */
			.timestamps = NULL,
		},
		{
			.ptr = ctx->magnetic_field_ut_y,
			.name = "sensors_magnetometer.magnetic_field_ut.y",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 1,
			.nb_samples = MAX_SAMPLE,
			.timestamps = NULL,
		},
		{
			.ptr = ctx->magnetic_field_ut_z,
			.name = "sensors_magnetometer.magnetic_field_ut.z",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 1,
			.nb_samples = MAX_SAMPLE,
			.timestamps = NULL,
		},
	};

	res = tlm_consumer_reg_entries_of_samples_array(
		ctx->array_sample_consumer_2,
		sensors_magnetometer_array,
		ARRAY_SIZE(sensors_magnetometer_array));
	if (res < 0) {
		ULOG_ERRNO("tlm_consumer_reg_entries_of_samples_array", -res);
		goto out;
	}

	/* END OF THE REGISTRATION OF array_sample_consumer_2 */

	res = tlm_consumer_reg_complete(ctx->array_sample_consumer_2);
	if (res < 0) {
		ULOG_ERRNO(
			"unable to complete registration of array_sample_consumer_2",
			-res);
		goto out;
	}

out:
	return res;
}


static int init_struct_sample_consumer(struct example_telemetry_context *ctx)
{
	int res;

	/**
	 * Register a tlm_reg_struct structure from the sensors_imu
	 * telemetry section.
	 *
	 * Several array of samples are registered with this method.
	 */

	res = tlm_consumer_reg_struct_ptr(ctx->struct_sample_consumer,
					  &ctx->imu_data,
					  "sensors_imu",
					  &sensors_tlm_reg_imu);
	if (res < 0) {
		ULOG_ERRNO("init_struct_sample_consumer", -res);
		goto out;
	}

	res = tlm_consumer_reg_complete(ctx->struct_sample_consumer);
	if (res < 0) {
		ULOG_ERRNO(
			"unable to complete registration of struct_sample_consumer",
			-res);
		goto out;
	}

out:
	return res;
}

static int init_my_producer_consumer(struct example_telemetry_context *ctx)
{
	int res;

	ctx->new_value_source = 0;
	ctx->new_value_destination = 0;

	res = tlm_producer_reg(ctx->my_producer, // Producer
			       &ctx->new_value_source, // Variable to regiter
			       "new_value", // data name
			       TLM_TYPE_INT32, // Type int32
			       sizeof(int32_t), // int32 size
			       1, // 1 element in the variable
			       0); // Flag default
	if (res < 0) {
		ULOG_ERRNO("tlm_producer_reg", -res);
		goto out;
	}

	res = tlm_producer_reg_complete(ctx->my_producer);
	if (res < 0) {
		ULOG_ERRNO("unable to complete registration of my_producer",
			   -res);
		goto out;
	}

	res = tlm_consumer_reg(
		ctx->my_consumer, // Consumer
		&ctx->new_value_destination, // Variable to register
		"new_section.new_value", // Section + data name
		TLM_TYPE_INT32, // Type int32
		sizeof(int32_t), // int32 size
		1, // 1 element in the variable
		NULL); // No need to store timestamp of samples
	if (res < 0) {
		ULOG_ERRNO("tlm_consumer_reg", -res);
		goto out;
	}

	res = tlm_consumer_reg_complete(ctx->my_consumer);
	if (res < 0) {
		ULOG_ERRNO("unable to complete registration of my_consumer",
			   -res);
		goto out;
	}


out:
	return res;
}

struct example_telemetry_context *example_telemetry_new(void)
{
	struct example_telemetry_context *ctx;


	/* allocate memory for context */
	ctx = (struct example_telemetry_context *)calloc(1, sizeof(*ctx));
	if (!ctx) {
		ULOGE("can't allocate process context");
		goto error;
	}

	ctx->single_sample_consumer = tlm_consumer_new();
	if (!ctx->single_sample_consumer) {
		ULOGE("can't create telemetry single_sample_consumer consumer");
		goto error;
	}

	ctx->array_sample_consumer_1 = tlm_consumer_new();
	if (!ctx->array_sample_consumer_1) {
		ULOGE("can't create telemetry array_sample_consumer_1 consumer");
		goto error;
	}

	ctx->array_sample_consumer_2 = tlm_consumer_new();
	if (!ctx->array_sample_consumer_2) {
		ULOGE("can't create telemetry array_sample_consumer_2 consumer");
		goto error;
	}

	ctx->struct_sample_consumer = tlm_consumer_new();
	if (!ctx->struct_sample_consumer) {
		ULOGE("can't create telemetry struct_sample_consumer consumer");
		goto error;
	}

	/* new value example*/
	/* create a new producer with a section name, maximum number of samples
	and an approximative rate of samples */
	ctx->my_producer = tlm_producer_new("new_section", 100, 1000);
	if (!ctx->my_producer) {
		ULOGE("can't create telemetry producer");
		goto error;
	}

	ctx->my_consumer = tlm_consumer_new();
	if (!ctx->my_consumer) {
		ULOGE("can't create telemetry consumer");
		goto error;
	}

	return ctx;

error:
	example_telemetry_destroy(ctx);

	return NULL;
}

void example_telemetry_destroy(struct example_telemetry_context *ctx)
{
	if (!ctx)
		return;

	if (ctx->single_sample_consumer)
		tlm_consumer_destroy(ctx->single_sample_consumer);

	if (ctx->array_sample_consumer_1)
		tlm_consumer_destroy(ctx->array_sample_consumer_1);

	if (ctx->array_sample_consumer_2)
		tlm_consumer_destroy(ctx->array_sample_consumer_2);

	if (ctx->struct_sample_consumer)
		tlm_consumer_destroy(ctx->struct_sample_consumer);

	if (ctx->my_producer)
		tlm_producer_destroy(ctx->my_producer);

	if (ctx->my_consumer)
		tlm_consumer_destroy(ctx->my_consumer);

	free(ctx);
}

int example_telemetry_init(struct example_telemetry_context *ctx)
{
	int res;

	if (!ctx)
		return -EINVAL;

	res = init_single_sample_consumer_1(ctx);
	if (res < 0) {
		ULOGE("init_single_sample_consumer_1 initialization failed");
		goto out;
	}

	res = init_single_sample_consumer_2(ctx);
	if (res < 0) {
		ULOGE("init_single_sample_consumer_2 initialization failed");
		goto out;
	}

	res = init_array_sample_consumer_1(ctx);
	if (res < 0) {
		ULOGE("init_array_sample_consumer_1 initialization failed");
		goto out;
	}

	res = init_array_sample_consumer_2(ctx);
	if (res < 0) {
		ULOGE("init_array_sample_consumer_2 initialization failed");
		goto out;
	}

	res = init_struct_sample_consumer(ctx);
	if (res < 0) {
		ULOGE("init_struct_sample_consumer initialization failed");
		goto out;
	}

	res = init_my_producer_consumer(ctx);
	if (res < 0) {
		ULOGE("my_producer and my_consumer initialization failed");
		goto out;
	}

out:
	return res;
}

void example_telemetry_put_samples(struct example_telemetry_context *ctx)
{
	int res = 0;

	if (ctx->new_value_source > 5) {
		ctx->new_value_source = 0;
	} else {
		ctx->new_value_source++;
	}

	res = tlm_producer_put_sample(
		/* producer */
		ctx->my_producer,
		/* NULL to use current time */
		NULL);
	if (res < 0) {
		ULOG_ERRNO("can't put telemetry from my_producer", -res);
	}
}

void example_telemetry_get_samples(struct example_telemetry_context *ctx)
{
	int res = 0;

	res = tlm_consumer_get_sample(
		/* consumer */
		ctx->single_sample_consumer,
		/* timestamp of query (can be NULL for LATEST method). */
		NULL,
		/* method of query. */
		TLM_LATEST);
	if (res < 0) {
		ULOG_ERRNO("can't get telemetry from single_sample_consumer",
			   -res);
	}

	/* note: tlm_consumer_get_samples work only for a single section, that's
	why there are two consumers array_sample_consumer */
	res = tlm_consumer_get_samples(
		/* consumer */
		ctx->array_sample_consumer_1,
		/* timestamp of query (can be NULL for LATEST method). */
		NULL,
		/* method of query. */
		TLM_LATEST,
		/* number of requested samples before the timestamp */
		MAX_SAMPLE - 1, // samples before
		/* number of requested samples after the timestamp.
		   nb_samples_before + nb_samples_after +1 <= nb_samples */
		0,
		/* number of actual returned samples */
		&ctx->nb_array_sample_1,
		/* index of the reference sample in the table. It is related to
		   the reference timestamp and depends on the method.*/
		&ctx->ref_array_sample_1);
	if (res < 0) {
		ULOG_ERRNO("can't get telemetry from array_sample_consumer_1",
			   -res);
	}

	res = tlm_consumer_get_samples(
		/* consumer */
		ctx->array_sample_consumer_2,
		/* timestamp of query (can be NULL for LATEST method). */
		NULL,
		/* method of query. */
		TLM_LATEST,
		/* number of requested samples before the timestamp */
		MAX_SAMPLE - 1, // samples before
		/* number of requested samples after the timestamp.
		   nb_samples_before + nb_samples_after +1 <= nb_samples */
		0,
		/* number of actual returned samples */
		&ctx->nb_array_sample_2,
		/* index of the reference sample in the table. It is related to
		   the reference timestamp and depends on the method.*/
		&ctx->ref_array_sample_2);
	if (res < 0) {
		ULOG_ERRNO("can't get telemetry from array_sample_consumer_2",
			   -res);
	}

	res = tlm_consumer_get_sample(
		/* consumer */
		ctx->struct_sample_consumer,
		/* timestamp of query (can be NULL for LATEST method). */
		NULL,
		/* method of query. */
		TLM_LATEST);
	if (res < 0) {
		ULOG_ERRNO("can't get telemetry from struct_sample_consumer",
			   -res);
	}

	res = tlm_consumer_get_sample(
		/* consumer */
		ctx->my_consumer,
		/* timestamp of query (can be NULL for LATEST method). */
		NULL,
		/* method of query. */
		TLM_LATEST);
	if (res < 0) {
		ULOG_ERRNO("can't get telemetry from my_consumer", -res);
	}
}

void example_telemetry_log_values(struct example_telemetry_context *ctx)
{
	ULOGI("##################################################");
	ULOGI("#### TELEMETRY VALUE:");
	ULOGI("####");
	ULOGI("#### single sample consumer");
	ULOGI("#### > pressure");
	ULOGI("####     pressure %f", ctx->pressure);
	ULOGI("#### > sensors tof:");
	ULOGI("####     distance       %f", ctx->sensors_tof_distance);
	ULOGI("####     magnitude      %f", ctx->sensors_tof_magnitude);
	ULOGI("####     temperature    %f", ctx->sensors_tof_temperature);
	ULOGI("####     ambient        %f", ctx->sensors_tof_ambient);
	ULOGI("####     qualification  %d", ctx->sensors_tof_qualification);
	ULOGI("####");
	ULOGI("#### array sample consumer ");
	ULOGI("#### > altitude_agl:");
	for (int i = 0; i < MAX_SAMPLE; i++)
		ULOGI("####     altitude_agl[%d]: %f", i, ctx->altitude_agl[i]);
	ULOGI("####  nb_array_sample_1 : %ld", ctx->nb_array_sample_1);
	ULOGI("####  ref_array_sample_1: %ld", ctx->ref_array_sample_1);
	ULOGI("#### > sensors_magnetometer:");
	for (int i = 0; i < MAX_SAMPLE; i++) {
		ULOGI("####     magnetic_field_ut.x[%d]: %f",
		      i,
		      ctx->magnetic_field_ut_x[i]);
	}
	for (int i = 0; i < MAX_SAMPLE; i++) {
		ULOGI("####     magnetic_field_ut.y[%d]: %f",
		      i,
		      ctx->magnetic_field_ut_y[i]);
	}
	for (int i = 0; i < MAX_SAMPLE; i++) {
		ULOGI("####     magnetic_field_ut.z[%d]: %f",
		      i,
		      ctx->magnetic_field_ut_z[i]);
	}
	ULOGI("####  nb_array_sample_2 : %ld", ctx->nb_array_sample_2);
	ULOGI("####  ref_array_sample_2: %ld", ctx->ref_array_sample_2);
	ULOGI("####");
	ULOGI("#### struct sample consumer ");
	ULOGI("#### > sensors_imu:");
	for (int i = 0; i < MAX_SAMPLE; i++) {
		ULOGI("####     gyro.x[%d]: %f", i, ctx->imu_data.gyro[i].x);
		ULOGI("####     gyro.y[%d]: %f", i, ctx->imu_data.gyro[i].y);
		ULOGI("####     gyro.z[%d]: %f", i, ctx->imu_data.gyro[i].z);
	}
	for (int i = 0; i < MAX_SAMPLE; i++) {
		ULOGI("####     acc.x[%d]: %f", i, ctx->imu_data.acc[i].x);
		ULOGI("####     acc.y[%d]: %f", i, ctx->imu_data.acc[i].y);
		ULOGI("####     acc.z[%d]: %f", i, ctx->imu_data.acc[i].z);
	}
	for (int i = 0; i < MAX_SAMPLE; i++) {
		ULOGI("####     temperature.z[%d]: %f",
		      i,
		      ctx->imu_data.temp[i]);
	}
	ULOGI("####");
	ULOGI("#### my_consumer");
	ULOGI("#### > my_section");
	ULOGI("####     my_value %d", ctx->new_value_destination);
}
