/**
 * Copyright (C) 2025 Parrot
 *
 * Minimal metadata sender example in C (stream + record)
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ulog.h>
#include <unistd.h>

#include <airsdk/airsdk.h>
#include <airsdk/metadata_publisher.h>

#define ULOG_TAG metadata_sender_c
ULOG_DECLARE_TAG(ULOG_TAG);

static volatile sig_atomic_t run = 1;

static void sig_handler(int sig)
{
	(void)sig;
	run = 0;
}

/* Listener callbacks */
static void
on_stream_user_metadata_list(const char *resource_path,
			     const struct stream_user_metadata *user_metadata,
			     size_t count,
			     void *userdata)
{
	(void)userdata;
	for (size_t i = 0; i < count; i++) {
		ULOGI("Stream metadata list updated: resource=%s uid=%s",
		      resource_path,
		      user_metadata[i].uid);
	}
}

static void
on_record_user_metadata_list(uint64_t camera_id,
			     const char *track_name,
			     const struct record_user_metadata *user_metadata,
			     size_t count,
			     void *userdata)
{
	(void)userdata;
	for (size_t i = 0; i < count; i++) {
		ULOGI("Record metadata list updated: camera=%lu track=%s uid=%s",
		      (unsigned long)camera_id,
		      track_name,
		      user_metadata[i].uid);
	}
}

int main(int argc, char *argv[])
{
	ULOGI("Starting metadata sender");

	srand((unsigned int)time(NULL));
	signal(SIGTERM, sig_handler);
	signal(SIGINT, sig_handler);

	/* Prepare listener */
	struct metadata_publisher_listener listener = {
		.stream_user_metadata_list_cb = on_stream_user_metadata_list,
		.record_user_metadata_list_cb = on_record_user_metadata_list,
		.userdata = NULL,
	};

	/* Create metadata publisher */
	struct metadata_publisher *mp = NULL;
	int res = metadata_publisher_new(
		&mp, "com.parrot.missions.samples.user_metadata_c", &listener);
	if (res < 0 || !mp) {
		ULOGE("Failed to create metadata publisher (%d)", res);
		return -1;
	}

	const char *stream_id = "dynamic_stream_value";
	const char *record_id = "dynamic_record_value";
	const char *resource_id = "front";
	const uint64_t camera_id = 1;
	const char *track_name = "DefaultVideo";
	const int max_wait_ms = 5000;
	const int wait_step_ms = 100;
	int waited = 0;

	/* Add stream metadata */
	while (run) {
		res = metadata_publisher_add_stream_user_metadata(
			mp, stream_id, 1, resource_id);
		if (res == 0) {
			ULOGI("Stream metadata '%s' successfully added",
			      stream_id);
			break;
		} else {
			ULOGW("Failed to add stream metadata '%s', error=%d, retrying...",
			      stream_id,
			      res);
			usleep(wait_step_ms * 1000);
			waited += wait_step_ms;
			if (waited >= max_wait_ms) {
				ULOGE("Cannot add stream metadata '%s'",
				      stream_id);
				metadata_publisher_destroy(mp);
				return -1;
			}
		}
	}

	/* Add record metadata */
	waited = 0;
	while (run) {
		res = metadata_publisher_add_record_user_metadata(
			mp, record_id, 1, camera_id, track_name);
		if (res == 0) {
			ULOGI("Record metadata '%s' successfully added",
			      record_id);
			break;
		} else {
			ULOGW("Failed to add record metadata '%s', error=%d, retrying...",
			      record_id,
			      res);
			usleep(wait_step_ms * 1000);
			waited += wait_step_ms;
			if (waited >= max_wait_ms) {
				ULOGE("Cannot add record metadata '%s'",
				      record_id);
				metadata_publisher_destroy(mp);
				return -1;
			}
		}
	}

	/* Main loop: push random metadata */
	while (run) {
		uint8_t random_value = (uint8_t)(rand() % 101);
		metadata_publisher_push(mp, stream_id, &random_value, 1);
		metadata_publisher_push(mp, record_id, &random_value, 1);
		usleep(33000); // ~30 FPS
	}

	/* Cleanup */
	metadata_publisher_stop(mp);
	metadata_publisher_destroy(mp);
	ULOGI("Metadata sender exited");
	return 0;
}
