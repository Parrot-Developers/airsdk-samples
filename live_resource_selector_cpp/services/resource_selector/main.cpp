/**
 * Copyright (C) 2025 Parrot
 *
 * Minimal live resource selector example in C++ (random camera request)
 */

#include <airsdk/airsdk.hpp>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <string>
#include <ulog.hpp>
#include <unistd.h>
#include <vector>

#include <arsdk/camera2.pb.h>

#define ULOG_TAG resource_selector_cpp
ULOG_DECLARE_TAG(ULOG_TAG);

#define UID "com.parrot.missions.samples.live_resource_selector_cpp"

static volatile sig_atomic_t run = 1;

extern "C" void sig_handler(int sig)
{
	run = 0;
}

/* Live Resource Selector Listener */
class MyLiveResourceListener : public airsdk::live_resource_selector::
				       LiveResourceSelectorListener {
public:
	std::vector<airsdk::live_resource_selector::LiveResourceInfo>
		live_resources;

	void onLiveResourceList(
		std::vector<airsdk::live_resource_selector::LiveResourceInfo>
			resources) override
	{
		live_resources = resources;

		ULOGI("Live resources received (%zu):", live_resources.size());
		for (auto &res : live_resources) {
			ULOGI(" - %s (type=%d, dynamic=%d, path=%s)",
			      res.description.c_str(),
			      static_cast<int>(res.type),
			      res.is_dynamic,
			      res.path.c_str());
		}
	}

	/* Notify callback: prints the requester (the UID) */
	void onNotifyStreamCamera(arsdk::camera::StreamReceiver receiver,
				  arsdk::camera::CameraType type,
				  std::string uid) override
	{
		ULOGI("Stream camera notification: receiver=%d, type=%d, requester=%s",
		      static_cast<int>(receiver),
		      static_cast<int>(type),
		      uid.c_str());
	}
};

int main(int argc, char *argv[])
{
	ULOGI("Starting live resource selector (C++)");

	signal(SIGTERM, sig_handler);
	signal(SIGINT, sig_handler);

	MyLiveResourceListener listener;
	airsdk::live_resource_selector::LiveResourceSelector selector(
		UID, &listener);

	/* Seed random generator */
	std::srand(static_cast<unsigned int>(std::time(nullptr)));

	/* Main loop */
	while (run) {
		/* Request live resource list */
		int res = selector.getLiveResourceList();
		if (res == 0) {
			ULOGI("Live resource list request successfully sent");
		} else {
			ULOGW("Failed to request live resource list (res=%d)",
			      res);
		}

		usleep(500 * 1000); /* 0.5s */

		/* Request random camera (pick a random resource and a random
		 * receiver */
		if (!listener.live_resources.empty()) {
			auto &cam_resource =
				listener.live_resources[std::rand() %
							listener.live_resources
								.size()];

			try {
				arsdk::camera::StreamReceiver stream_receiver =
					(std::rand() % 2 == 0)
						? arsdk::camera::
							  STREAM_RECEIVER_DEFAULT
						: arsdk::camera::
							  STREAM_RECEIVER_SKYCONTROLLER;

				selector.requestStreamCamera(stream_receiver,
							     cam_resource.type,
							     UID);

				ULOGI("Random camera request sent: receiver=%d, type=%d (%s)",
				      static_cast<int>(stream_receiver),
				      static_cast<int>(cam_resource.type),
				      cam_resource.description.c_str());
			} catch (...) {
				ULOGE("Failed to request random camera");
			}
		}

		sleep(10);
	}

	selector.stopLiveResourceSelector();
	ULOGI("Live resource selector exited");
	return 0;
}
