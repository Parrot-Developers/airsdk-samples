"""
Copyright (C) 2025 Parrot

Minimal live resource selector example in Python (random camera request)
"""

import asyncio
import logging
import signal
import random
import ulog
from airsdk_py import (
    LiveResourceSelector,
    LiveResourceSelectorListener,
    StreamReceiver,
    CameraType,
)

PROCESS_NAME = "resource_selector_py"
UID = "com.parrot.missions.samples.live_resource_selector_py"


# Listener to handle live resource callbacks
class MyLiveResourceListener(LiveResourceSelectorListener):
    def __init__(self):
        super().__init__()
        self.live_resources = []

    def onLiveResourceList(self, live_resources):
        self.live_resources = live_resources
        res_summary = "\n".join(
            f"- {res['description']} (type={res['type']}, dynamic={res['is_dynamic']}, path={res['path']})"
            for res in live_resources
        )

        logging.getLogger("main").info(
            "Live resources received (%d):\n%s", len(live_resources), res_summary
        )

    def onNotifyStreamCamera(self, receiver, camera_type, requester):
        logging.getLogger("main").info(
            "Stream camera notification: receiver=%d, type=%d, requester=%s",
            receiver,
            camera_type,
            requester,
        )


async def service_main():
    # Setup logging
    ulog.setup_logging(PROCESS_NAME)
    logger = logging.getLogger("main")
    logger.info("Starting live resource selector (Python)")

    # Initialize Live Resource Selector
    listener = MyLiveResourceListener()
    selector = LiveResourceSelector(UID, listener)

    run = True

    # Signal handlers to gracefully stop the service
    def sig_handler(*_):
        nonlocal run
        run = False

    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, sig_handler)
    loop.add_signal_handler(signal.SIGINT, sig_handler)

    # Main loop: request live resource list periodically
    while run:
        # Request live resource list
        try:
            res = selector.get_live_resource_list()
            if res == 0:
                logger.info("Live resource list request successfully sent")
            else:
                logger.warning(f"Failed to request live resource list (res={res})")
        except Exception as e:
            logger.warning(f"Exception while requesting live resource list: {e}")

        await asyncio.sleep(0.5)  # small wait for resources to populate

        # Request a random camera from the list
        if listener.live_resources:
            cam_resource = random.choice(listener.live_resources)
            try:
                # Randomly choose the stream receiver
                stream_receiver = random.choice(
                    [StreamReceiver.DEFAULT, StreamReceiver.SKYCONTROLLER]
                )
                cam_type = cam_resource["type"]
                selector.request_stream_camera(stream_receiver, cam_type, UID)
                logger.info(
                    "Random camera request sent: receiver=%s, type=%s (%s)",
                    str(stream_receiver),
                    str(cam_type),
                    cam_resource["description"],
                )
            except Exception as e:
                logger.error(f"Failed to request random camera: {e}")

        await asyncio.sleep(10)  # wait 10 seconds before next request

    # Stop selector on exit
    selector.stop_live_resource_selector()
    logger.info("Live resource selector service exited")
    return 0


def main():
    asyncio.run(service_main())
