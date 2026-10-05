/**
 * Copyright (C) 2025 Parrot
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
 * Minimal metadata sender example in C++ (stream + record)
 */

#include <airsdk/airsdk.hpp>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <ulog.hpp>
#include <unistd.h>

#define ULOG_TAG metadata_sender
ULOG_DECLARE_TAG(ULOG_TAG);

static volatile sig_atomic_t run = 1;

extern "C" void sig_handler(int sig)
{
	run = 0;
}

// Metadata Publisher Listener
class MyMetadataListener
		: public airsdk::metadata_publisher::MetadataPublisherListener {
public:
	void onStreamUserMetadataList(
		std::string resource_path,
		std::vector<airsdk::metadata_publisher::StreamUserMetadata>
			user_metadata) override
	{
		for (auto &m : user_metadata) {
			ULOGI("Stream metadata list updated: resource=%s uid=%s",
			      resource_path.c_str(),
			      m.uid.c_str());
		}
	}

	void onRecordUserMetadataList(
		uint64_t camera_id,
		std::string track_name,
		std::vector<airsdk::metadata_publisher::RecordUserMetadata>
			user_metadata) override
	{
		for (auto &m : user_metadata) {
			ULOGI("Record metadata list updated: camera=%lu track=%s uid=%s",
			      camera_id,
			      track_name.c_str(),
			      m.uid.c_str());
		}
	}
};

int main(int argc, char *argv[])
{
	ULOGI("Starting metadata sender");

	std::srand(static_cast<unsigned int>(std::time(nullptr)));
	signal(SIGTERM, sig_handler);
	signal(SIGINT, sig_handler);

	MyMetadataListener listener;
	airsdk::metadata_publisher::MetadataPublisher mp(
		"com.parrot.missions.samples.user_metadata_cpp", &listener);

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
		int res = mp.addStreamUserMetadata(stream_id, 1, resource_id);
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
				return -1;
			}
		}
	}

	waited = 0;
	/* Add record metadata */
	while (run) {
		int res = mp.addRecordUserMetadata(
			record_id, 1, camera_id, track_name);
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
				return -1;
			}
		}
	}

	/* Main loop: push random metadata */
	while (run) {
		uint8_t random_value = static_cast<uint8_t>(std::rand() % 101);
		mp.push(stream_id, &random_value, 1);
		mp.push(record_id, &random_value, 1);
		usleep(33000); // ~30 FPS
	}

	mp.stopMetadataPublisher();
	ULOGI("Metadata sender exited");
	return 0;
}
