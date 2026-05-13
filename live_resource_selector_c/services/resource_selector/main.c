/**
 * Copyright (C) 2025 Parrot
 *
 * Minimal live resource selector example in C (random camera request)
 */

#include <airsdk/airsdk.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define ULOG_TAG resource_selector_c
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#define UID "com.parrot.missions.samples.live_resource_selector_c"

/* --- Function prototypes --- */
void sig_handler(int sig);
void on_live_resource_list(const struct live_resource_info *resources,
			   size_t count,
			   void *userdata);
void on_notify_stream_camera(int receiver,
			     int camera_type,
			     const char *requester,
			     void *userdata);

static volatile sig_atomic_t run = 1;

/* Global storage of last received resources */
static struct live_resource_info *g_resources = NULL;
static size_t g_count = 0;

void sig_handler(int sig)
{
	(void)sig;
	run = 0;
}

/* Re-declare the enums from camera2.pb.h in C form so we can use them in C
 * code. */
typedef enum {
	STREAM_RECEIVER_DEFAULT = 0,
	STREAM_RECEIVER_SKYCONTROLLER = 1
} StreamReceiver;

typedef enum {
	CAMERA_TYPE_DEFAULT = 0,
	CAMERA_TYPE_FRONT = 1,
	CAMERA_TYPE_STEREO_LEFT = 2,
	CAMERA_TYPE_STEREO_RIGHT = 3,
	CAMERA_TYPE_VERTICAL = 4,
	CAMERA_TYPE_DISPARITY = 5,
	CAMERA_TYPE_HORIZONTAL_STEREO_LEFT = 6,
	CAMERA_TYPE_HORIZONTAL_STEREO_RIGHT = 7,
	CAMERA_TYPE_DOWN_STEREO_LEFT = 8,
	CAMERA_TYPE_DOWN_STEREO_RIGHT = 9,
	CAMERA_TYPE_EXTERNAL = 10
} CameraType;

/* Listener callbacks (match C API prototypes: ints for enums, const char*
 * requester) */
void on_live_resource_list(const struct live_resource_info *resources,
			   size_t count,
			   void *userdata)
{
	(void)userdata;

	/* Free previous array */
	if (g_resources)
		free(g_resources);

	g_count = count;
	g_resources = (struct live_resource_info *)malloc(
		sizeof(struct live_resource_info) * g_count);
	if (!g_resources) {
		ULOGE("Failed to allocate memory for live resources");
		g_count = 0;
		return;
	}

	memcpy(g_resources,
	       resources,
	       g_count * sizeof(struct live_resource_info));

	/* Log the received list */
	ULOGI("Live resources received (%zu):", count);
	for (size_t i = 0; i < g_count; ++i) {
		ULOGI(" - %s (type=%d)",
		      g_resources[i].description,
		      g_resources[i].type);
	}
}

/* The C API expects the notify callback signature to be (int, int, const char*,
 * void*). */
void on_notify_stream_camera(int receiver,
			     int camera_type,
			     const char *requester,
			     void *userdata)
{
	(void)userdata;
	ULOGI("Stream camera notification: receiver=%d, type=%d, requester=%s",
	      receiver,
	      camera_type,
	      requester ? requester : "(null)");
}

int main(void)
{
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	ULOGI("Starting live resource selector (C)");

	/* Seed random generator */
	srand((unsigned int)time(NULL));

	/* Setup listener (C callbacks) */
	struct live_resource_selector_listener listener = {
		.live_resource_list_cb = on_live_resource_list,
		.notify_stream_camera_cb = on_notify_stream_camera,
		.userdata = NULL};

	struct live_resource_selector *selector = NULL;
	if (live_resource_selector_new(&selector, UID, &listener) != 0) {
		ULOGE("Failed to create live resource selector");
		return -1;
	}

	/* Main loop: keep requesting the live resource list and occasionally
	 * request a random camera */
	while (run) {
		int res =
			live_resource_selector_get_live_resource_list(selector);
		if (res == 0) {
			ULOGI("Live resource list request successfully sent");
		} else {
			ULOGW("Failed to request live resource list (res=%d)",
			      res);
		}

		usleep(500 * 1000); /* 0.5s */

		/* If any live resource exists, request a random one */
		if (g_count > 0) {
			size_t idx = rand() % g_count;
			const char *description = g_resources[idx].description;
			int cam_type = (int)g_resources[idx].type;
			int receiver = (rand() % 2 == 0)
					       ? STREAM_RECEIVER_DEFAULT
					       : STREAM_RECEIVER_SKYCONTROLLER;

			if (live_resource_selector_request_stream_camera(
				    selector, receiver, cam_type, UID) == 0) {
				ULOGI("Random camera request sent: receiver=%d, type=%d (%s)",
				      receiver,
				      cam_type,
				      description);
			} else {
				ULOGE("Failed to request random camera");
			}
		}

		sleep(10);
	}

	/* Cleanup */
	live_resource_selector_stop(selector);
	live_resource_selector_destroy(selector);

	if (g_resources)
		free(g_resources);

	ULOGI("Live resource selector exited");
	return 0;
}
