/*
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
 */

#include "appserver.h"
#include <json-c/json.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <airsdk/airsdk.h>
#include <airsdk/publisher.h>
#include <libtelemetry.h>

#define ULOG_TAG mission_ui_c
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

/* Global publisher pointer (real type from publisher.h) */
static struct publisher *g_publisher = NULL;

/* publish thread args */
struct pub_args {
	struct publisher *p;
	char svc[128];
	uint32_t port;
};

/* publish thread function using publish_service from publisher.h */
static void *publish_thread_fn(void *raw)
{
	struct pub_args *a = (struct pub_args *)raw;
	if (!a)
		return NULL;

	for (int i = 0; i < 50 && a->p; ++i) {
		int rc = publish_service(a->p, a->svc, a->port);
		if (rc == 0) {
			ULOGI("Published %s on port %u", a->svc, a->port);
			break;
		}
		usleep(100 * 1000);
	}
	free(a);
	return NULL;
}

/* Passed to start_webserver so it can publish the chosen port. Static: only
 * used within this file, no need for external linkage. */
static void publish_port_cb_c(const char *process_name, int port)
{
	if (!g_publisher) {
		ULOGW("publish_port_cb: publisher=null");
		return;
	}

	const char *svc = process_name ? process_name : "unknown";

	struct pub_args *args = calloc(1, sizeof(*args));
	if (!args) {
		ULOGE("alloc pub args failed");
		return;
	}
	args->p = g_publisher;
	strncpy(args->svc, svc, sizeof(args->svc) - 1);
	args->port = (uint32_t)port;

	pthread_t t;
	if (pthread_create(&t, NULL, publish_thread_fn, args) == 0) {
		pthread_detach(t);
	} else {
		ULOGE("failed to create publish thread");
		free(args);
	}
}

/* Telemetry structures */
#define MAX_SAMPLES 10

union sensor_vector {
	float v[3];
	struct {
		float x, y, z;
	};
};

struct sensor_imu_data {
	union sensor_vector gyro[MAX_SAMPLES];
	union sensor_vector acc[MAX_SAMPLES];
	float temp[MAX_SAMPLES];
	int32_t fsync[MAX_SAMPLES];
	uint8_t nb_samples;
	struct timespec timestamp[MAX_SAMPLES];
};

/* TLM registration */
static const struct tlm_reg_field sensors_tlm_imu_vec3_fields[] = {
	TLM_REG_FIELD_SCALAR(union sensor_vector, x, TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(union sensor_vector, y, TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR(union sensor_vector, z, TLM_TYPE_FLOAT32),
};

static const struct tlm_reg_struct sensors_tlm_imu_gyro_reg_struct =
	TLM_REG_STRUCT("gyro", sensors_tlm_imu_vec3_fields);

static const struct tlm_reg_struct sensors_tlm_imu_acc_reg_struct =
	TLM_REG_STRUCT("acc", sensors_tlm_imu_vec3_fields);

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

/* Attitude/altitude/velocity: single latest-sample values, not averaged like
 * the IMU. */
struct drone_attitude_data {
	float roll;
	float pitch;
	float yaw;
	float altitude_agl;
	float vx;
	float vy;
};

static const struct tlm_reg_field drone_attitude_tlm_fields[] = {
	TLM_REG_FIELD_SCALAR_EX(struct drone_attitude_data,
				roll,
				"attitude_euler_angles.roll",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct drone_attitude_data,
				pitch,
				"attitude_euler_angles.pitch",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct drone_attitude_data,
				yaw,
				"attitude_euler_angles.yaw",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct drone_attitude_data,
				altitude_agl,
				"altitude_agl",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct drone_attitude_data,
				vx,
				"linear_velocity_global.x",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(struct drone_attitude_data,
				vy,
				"linear_velocity_global.y",
				TLM_TYPE_FLOAT32),
};

static const struct tlm_reg_struct drone_attitude_tlm_reg =
	TLM_REG_STRUCT(NULL, drone_attitude_tlm_fields);

/* telemetry consumer and local imu/attitude data */
static struct tlm_consumer *g_tlm_consumer = NULL;
static struct sensor_imu_data g_imu_data;
static struct drone_attitude_data g_attitude_data;

/* WebSocket handlers */
static int handler_pingpong(struct lws *wsi, struct json_object *jobj)
{
	const char *val = NULL;
	struct json_object *v = NULL;
	if (json_object_object_get_ex(jobj, "value", &v))
		val = json_object_get_string(v);

	const char *reply = "PING";
	if (val && strcmp(val, "PING") == 0) {
		reply = "PONG";
	} else if (val && strcmp(val, "PONG") == 0) {
		reply = "PING";
	}

	struct json_object *resp = json_object_new_object();
	json_object_object_add(resp, "pingPong", json_object_new_string(reply));
	const char *s = json_object_to_json_string(resp);
	send_message(wsi, s);
	json_object_put(resp);
	return 0;
}

static int handler_sendMsg(struct lws *wsi, struct json_object *jobj)
{
	(void)wsi;
	const char *val = NULL;
	struct json_object *v = NULL;
	if (json_object_object_get_ex(jobj, "value", &v))
		val = json_object_get_string(v);

	struct json_object *resp = json_object_new_object();
	char tmp[256];
	snprintf(tmp, sizeof(tmp), "Drone received: %s", val ? val : "");
	json_object_object_add(resp, "sendMsg", json_object_new_string(tmp));
	const char *s = json_object_to_json_string(resp);
	broadcast(s);
	json_object_put(resp);
	return 0;
}

static int handler_triggerWebviews(struct lws *wsi, struct json_object *jobj)
{
	(void)wsi;
	(void)jobj;
	struct json_object *resp = json_object_new_object();
	json_object_object_add(
		resp, "triggerWebviews", json_object_new_string(""));
	const char *s = json_object_to_json_string(resp);
	broadcast(s);
	json_object_put(resp);
	return 0;
}

/* Telemetry broadcast thread */
static void *telemetry_broadcast(void *arg)
{
	(void)arg;

	if (!g_tlm_consumer) {
		ULOGW("telemetry_broadcast: consumer is NULL");
		return NULL;
	}

	/* Previous altitude sample, to derive a vertical speed (VSI). */
	float previous_altitude = 0.0f;
	bool have_previous_altitude = false;

	while (run) {
		int res = tlm_consumer_get_sample(
			g_tlm_consumer, NULL, TLM_LATEST);
		if (res < 0) {
			ULOGW("tlm_consumer_get_sample failed %d", res);
			/* still sleep and continue */
			usleep(500 * 1000);
			continue;
		}

		/* compute averages on MAX_SAMPLES */
		double gx = 0, gy = 0, gz = 0;
		double ax = 0, ay = 0, az = 0;

		for (int i = 0; i < MAX_SAMPLES; ++i) {
			gx += g_imu_data.gyro[i].x;
			gy += g_imu_data.gyro[i].y;
			gz += g_imu_data.gyro[i].z;

			ax += g_imu_data.acc[i].x;
			ay += g_imu_data.acc[i].y;
			az += g_imu_data.acc[i].z;
		}
		gx /= MAX_SAMPLES;
		gy /= MAX_SAMPLES;
		gz /= MAX_SAMPLES;
		ax /= MAX_SAMPLES;
		ay /= MAX_SAMPLES;
		az /= MAX_SAMPLES;

		/* Broadcast cadence is fixed at 500ms (see usleep below), so that's dt. */
		float vertical_speed =
			have_previous_altitude
				? (g_attitude_data.altitude_agl - previous_altitude) / 0.5f
				: 0.0f;
		previous_altitude = g_attitude_data.altitude_agl;
		have_previous_altitude = true;

		/* Ground speed (GPS/odometry-derived), not true airspeed: a drone
		 * has no pitot tube to measure speed relative to the air. */
		float ground_speed = sqrtf(g_attitude_data.vx * g_attitude_data.vx +
					   g_attitude_data.vy * g_attitude_data.vy);

		/* Build JSON object matching the Python/JS expected shape */
		struct json_object *root = json_object_new_object();
		struct json_object *tlm = json_object_new_object();
		struct json_object *sensors_imu = json_object_new_object();
		struct json_object *gyro = json_object_new_object();
		struct json_object *acc = json_object_new_object();

		json_object_object_add(gyro, "x", json_object_new_double(gx));
		json_object_object_add(gyro, "y", json_object_new_double(gy));
		json_object_object_add(gyro, "z", json_object_new_double(gz));

		json_object_object_add(acc, "x", json_object_new_double(ax));
		json_object_object_add(acc, "y", json_object_new_double(ay));
		json_object_object_add(acc, "z", json_object_new_double(az));

		json_object_object_add(sensors_imu, "gyro", gyro);
		json_object_object_add(sensors_imu, "acc", acc);
		json_object_object_add(tlm, "sensors_imu", sensors_imu);

		struct json_object *attitude = json_object_new_object();
		json_object_object_add(
			attitude, "roll", json_object_new_double(g_attitude_data.roll));
		json_object_object_add(
			attitude, "pitch", json_object_new_double(g_attitude_data.pitch));
		json_object_object_add(
			attitude, "yaw", json_object_new_double(g_attitude_data.yaw));
		json_object_object_add(tlm, "attitude", attitude);
		json_object_object_add(tlm,
					"altitude_agl",
					json_object_new_double(g_attitude_data.altitude_agl));
		json_object_object_add(
			tlm, "ground_speed", json_object_new_double(ground_speed));
		json_object_object_add(
			tlm, "vertical_speed", json_object_new_double(vertical_speed));

		json_object_object_add(root, "telemetry", tlm);

		const char *s = json_object_to_json_string(root);
		broadcast(s);
		json_object_put(root);

		usleep(500 * 1000); /* 500 ms */
	}

	return NULL;
}

int main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	/* signal handling */
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	ULOGI("Starting mission_ui_c...");

	/* create publisher (C API) */
	int rc = publisher_new(&g_publisher,
			       "com.parrot.missions.samples.mission_ui_c");
	if (rc != 0) {
		ULOGW("publisher_new failed: %d", rc);
		g_publisher = NULL;
	}

	/* Register WebSocket commands */
	register_command("pingPong", handler_pingpong);
	register_command("sendMsg", handler_sendMsg);
	register_command("triggerWebviews", handler_triggerWebviews);

	/* Start webserver */
	start_webserver("mission_ui_c", publish_port_cb_c, "0.0.0.0", 0);

	/* Initialize telemetry consumer and register IMU + attitude structs */
	memset(&g_imu_data, 0, sizeof(g_imu_data));
	memset(&g_attitude_data, 0, sizeof(g_attitude_data));
	g_tlm_consumer = tlm_consumer_new();
	if (!g_tlm_consumer) {
		ULOGW("failed to create tlm consumer");
	} else {
		rc = tlm_consumer_reg_struct_ptr(g_tlm_consumer,
						 &g_imu_data,
						 "sensors_imu",
						 &sensors_tlm_reg_imu);
		if (rc >= 0)
			rc = tlm_consumer_reg_struct_ptr(g_tlm_consumer,
							 &g_attitude_data,
							 "drone_controller",
							 &drone_attitude_tlm_reg);
		if (rc < 0) {
			ULOG_ERRNO("tlm_consumer_reg_struct_ptr", -rc);
			tlm_consumer_destroy(g_tlm_consumer);
			g_tlm_consumer = NULL;
		} else {
			rc = tlm_consumer_reg_complete(g_tlm_consumer);
			if (rc < 0) {
				ULOG_ERRNO("tlm_consumer_reg_complete", -rc);
				tlm_consumer_destroy(g_tlm_consumer);
				g_tlm_consumer = NULL;
			} else {
				ULOGI("Telemetry consumer initialized (struct)");
			}
		}
	}

	/* Launch telemetry broadcast thread if telemetry consumer available */
	pthread_t tlm_thread = 0;
	if (g_tlm_consumer) {
		if (pthread_create(
			    &tlm_thread, NULL, telemetry_broadcast, NULL) !=
		    0) {
			ULOGE("failed to create telemetry thread");
			tlm_consumer_destroy(g_tlm_consumer);
			g_tlm_consumer = NULL;
		}
	}

	/* Main loop */
	while (run) {
		sleep(1);
	}

	ULOGI("Stopping...");
	stop_webserver();

	/* cleanup publisher via C API */
	if (g_publisher) {
		(void)stop_publisher(g_publisher);
		(void)destroy_publisher(g_publisher);
		g_publisher = NULL;
	}

	/* ensure run cleared; join thread */
	run = 0;
	if (tlm_thread)
		pthread_join(tlm_thread, NULL);

	if (g_tlm_consumer)
		tlm_consumer_destroy(g_tlm_consumer);

	ULOGI("Exiting mission_ui_c");
	return 0;
}
