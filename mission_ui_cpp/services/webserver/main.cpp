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
 */

#include "appserver.hpp"
#include <airsdk/airsdk.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <json-c/json.h>
#include <libtelemetry.hpp>
#include <thread>
#include <ulog.h>

#define ULOG_TAG mission_ui_cpp
ULOG_DECLARE_TAG(ULOG_TAG);

static airsdk::publisher::Publisher *g_publisher = nullptr;

/* Telemetry structures */
static constexpr int MAX_SAMPLES = 10;

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
	TLM_REG_FIELD_STRUCT_ARRAY(sensor_imu_data,
				   gyro,
				   &sensors_tlm_imu_gyro_reg_struct),
	TLM_REG_FIELD_STRUCT_ARRAY(sensor_imu_data,
				   acc,
				   &sensors_tlm_imu_acc_reg_struct),
	TLM_REG_FIELD_SCALAR_ARRAY_EX(sensor_imu_data,
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
	TLM_REG_FIELD_SCALAR_EX(drone_attitude_data,
				roll,
				"attitude_euler_angles.roll",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(drone_attitude_data,
				pitch,
				"attitude_euler_angles.pitch",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(drone_attitude_data,
				yaw,
				"attitude_euler_angles.yaw",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(drone_attitude_data,
				altitude_agl,
				"altitude_agl",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(drone_attitude_data,
				vx,
				"linear_velocity_global.x",
				TLM_TYPE_FLOAT32),
	TLM_REG_FIELD_SCALAR_EX(drone_attitude_data,
				vy,
				"linear_velocity_global.y",
				TLM_TYPE_FLOAT32),
};

static const struct tlm_reg_struct drone_attitude_tlm_reg =
	TLM_REG_STRUCT(NULL, drone_attitude_tlm_fields);

/* Telemetry consumer wrapper: returns attitude/altitude/speed telemetry
 * (gyro/acc averaged, the rest latest-sample) as JSON. */
class TelemetryManager {
public:
	TelemetryManager()
	{
		consumer = telemetry::Consumer::create();
		imu_data = {};
		attitude_data = {};
		int res = consumer->regStructPtr(
			&imu_data, "sensors_imu", &sensors_tlm_reg_imu);
		if (res >= 0)
			res = consumer->regStructPtr(&attitude_data,
						     "drone_controller",
						     &drone_attitude_tlm_reg);
		if (res < 0) {
			ULOG_ERRNO("Telemetry regStructPtr", -res);
		} else {
			res = consumer->regComplete();
			if (res < 0)
				ULOG_ERRNO("Telemetry regComplete", -res);
			else
				ULOGI("Telemetry initialized (struct)");
		}
	}

	~TelemetryManager()
	{
		telemetry::Consumer::release(consumer);
	}

	/* Reads the latest sample set and returns the full telemetry payload
	 * (gyro/acc averaged over MAX_SAMPLES samples) as JSON. */
	std::string getAveragedTelemetry()
	{
		int res = consumer->getSample(nullptr,
					      telemetry::Method::TLM_LATEST);
		if (res < 0) {
			ULOGW("getSample failed %d", res);
			return "{}";
		}

		float gx = 0, gy = 0, gz = 0;
		float ax = 0, ay = 0, az = 0;

		for (int i = 0; i < MAX_SAMPLES; ++i) {
			gx += imu_data.gyro[i].x;
			gy += imu_data.gyro[i].y;
			gz += imu_data.gyro[i].z;

			ax += imu_data.acc[i].x;
			ay += imu_data.acc[i].y;
			az += imu_data.acc[i].z;
		}

		gx /= MAX_SAMPLES;
		gy /= MAX_SAMPLES;
		gz /= MAX_SAMPLES;
		ax /= MAX_SAMPLES;
		ay /= MAX_SAMPLES;
		az /= MAX_SAMPLES;

		/* Broadcast cadence is fixed at 500ms (see main()), so that's dt. */
		float verticalSpeed = havePreviousAltitude
					      ? (attitude_data.altitude_agl -
						 previousAltitude) / 0.5f
					      : 0.0f;
		previousAltitude = attitude_data.altitude_agl;
		havePreviousAltitude = true;

		/* Ground speed (GPS/odometry-derived), not true airspeed: a drone
		 * has no pitot tube to measure speed relative to the air. */
		float groundSpeed = std::sqrt(attitude_data.vx * attitude_data.vx +
					      attitude_data.vy * attitude_data.vy);

		/* Build JSON object matching Python/JS expected shape */
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
			attitude, "roll", json_object_new_double(attitude_data.roll));
		json_object_object_add(
			attitude, "pitch", json_object_new_double(attitude_data.pitch));
		json_object_object_add(
			attitude, "yaw", json_object_new_double(attitude_data.yaw));
		json_object_object_add(tlm, "attitude", attitude);
		json_object_object_add(
			tlm,
			"altitude_agl",
			json_object_new_double(attitude_data.altitude_agl));
		json_object_object_add(
			tlm, "ground_speed", json_object_new_double(groundSpeed));
		json_object_object_add(
			tlm, "vertical_speed", json_object_new_double(verticalSpeed));

		json_object_object_add(root, "telemetry", tlm);

		/* convert to std::string then free json-c object */
		std::string msg(json_object_to_json_string(root));
		json_object_put(root);
		return msg;
	}

private:
	telemetry::Consumer *consumer;
	sensor_imu_data imu_data;
	drone_attitude_data attitude_data;
	float previousAltitude = 0.0f;
	bool havePreviousAltitude = false;
};

/* Periodically reads averaged telemetry and broadcasts it to all clients. */
void telemetry_broadcast()
{
	TelemetryManager manager;
	while (run) {
		std::string payload = manager.getAveragedTelemetry();
		broadcast(payload);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}

/* WebSocket handlers (registered via register_command) */
static int handler_pingpong(struct lws *wsi, struct json_object *jobj)
{
	const char *val = nullptr;
	json_object *v = nullptr;
	if (json_object_object_get_ex(jobj, "value", &v))
		val = json_object_get_string(v);

	std::string reply = "PING";
	if (val && std::string(val) == "PING") {
		reply = "PONG";
	} else if (val && std::string(val) == "PONG") {
		reply = "PING";
	}

	struct json_object *resp = json_object_new_object();
	json_object_object_add(
		resp, "pingPong", json_object_new_string(reply.c_str()));

	/* convert to std::string before freeing json_object */
	std::string s = json_object_to_json_string(resp);
	send_message(wsi, s);

	json_object_put(resp);
	return 0;
}

static int handler_sendMsg(struct lws *wsi, struct json_object *jobj)
{
	(void)wsi;
	const char *val = nullptr;
	json_object *v = nullptr;
	if (json_object_object_get_ex(jobj, "value", &v))
		val = json_object_get_string(v);

	struct json_object *resp = json_object_new_object();
	json_object_object_add(
		resp,
		"sendMsg",
		json_object_new_string(
			(std::string("Drone received: ") + (val ? val : ""))
				.c_str()));
	std::string s = json_object_to_json_string(resp);
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
	std::string s = json_object_to_json_string(resp);
	broadcast(s);
	json_object_put(resp);
	return 0;
}

/* Passed to start_webserver so it can publish the chosen port. Static: only
 * used within this file. extern "C" for the plain-C callback signature
 * start_webserver() expects. */
extern "C" {
static void publish_port_cb_c(const char *process_name, int port)
{
	if (!g_publisher) {
		ULOGW("publish_port_cb: publisher=null");
		return;
	}
	std::string svc = process_name ? process_name : "unknown";
	std::thread([svc, port]() {
		for (int i = 0; i < 50 && g_publisher; ++i) {
			int rc = g_publisher->publishService(svc,
							     (uint32_t)port);
			if (rc == 0) {
				ULOGI("Published %s on port %d",
				      svc.c_str(),
				      port);
				return;
			}
			std::this_thread::sleep_for(
				std::chrono::milliseconds(100));
		}
		ULOGE("Failed to publish %s", svc.c_str());
	}).detach();
}
} // extern "C"

int main(int argc, char *argv[])
{
	/* signal handling */
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	ULOGI("Starting mission_ui_cpp...");

	airsdk::publisher::Publisher publisher(
		"com.parrot.missions.samples.mission_ui_cpp");
	g_publisher = &publisher;

	/* Register WebSocket commands */
	register_command("pingPong", handler_pingpong);
	register_command("sendMsg", handler_sendMsg);
	register_command("triggerWebviews", handler_triggerWebviews);

	/* Start webserver */
	start_webserver("mission_ui_cpp", publish_port_cb_c, "0.0.0.0", 0);

	/* Launch telemetry broadcast thread */
	std::thread tlm_thread(telemetry_broadcast);

	/* Main loop */
	while (run) {
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	ULOGI("Stopping...");
	stop_webserver();
	publisher.stopPublisher();

	run = 0;
	if (tlm_thread.joinable())
		tlm_thread.join();

	g_publisher = nullptr;
	return 0;
}
