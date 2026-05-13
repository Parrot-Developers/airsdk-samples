# Copyright (C) 2025 parrot

# This sample service illustrates how we can use the Olympe SDK from
# within an AirSDK mission. It instantiates an Olympe LocalController
# to connect to the drone, configure one camera in photo mode and
# take a photo.

import asyncio
import logging
import signal
import ulog

import olympe
from olympe.messages.camera2 import Command, Event, Config
from olympe.enums.camera2 import CameraMode, PhotoEvent, PhotoStopReason

# Our target camera ID.
CAMERA_ID=1

# This is limited to 15 characters
PROCESS_NAME = b"myphoto-svc"

TIMEOUT_SEC = 10.0

ulog.setup_logging(PROCESS_NAME)
logger = logging.getLogger('main')


def take_photo(drone):

    # Configure the target camera in photo mode.
    cmd = Command.Configure(
        camera_id=CAMERA_ID,
        config=Config(camera_mode=CameraMode.CAMERA_MODE_PHOTO),
        _timeout=TIMEOUT_SEC
    )
    res = drone(cmd).wait().success()
    if not res:
        logger.error("failed to configure camera in photo mode")
        return False

    # Take a photo and wait for it to be done.
    cmd = Command.StartPhoto(camera_id=CAMERA_ID)
    eventDone = Event.Photo(
        type=PhotoEvent.PHOTO_EVENT_STOP,
        stop_reason=PhotoStopReason.PHOTO_STOP_REASON_CAPTURE_DONE,
        _timeout=TIMEOUT_SEC,
        _policy="wait",
    )
    take_pic = drone(eventDone & cmd)
    if not take_pic.wait(_timeout=30).success():
        logger.error("failed to take the photo")
        return False

    return True


async def service_main():
    logger.info("Hello from the photo service")

    # Configure SIGTERM signal handler.
    signaled = asyncio.Event()
    def sig_handler(*_):
        logger.info("received SIGTERM")
        signaled.set()
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, sig_handler)

    with olympe.LocalController() as drone:

        # Connect to the drone local controller.
        res = drone.connect()
        if not res:
            logger.error("failed to connect")
            return 1

        # Take a photo.
        res = take_photo(drone)
        if not res:
            logger.error("failed to take a photo")
            return 1

        # Disconnect.
        res = drone.disconnect()
        if not res:
            logger.error("failed to disconnect")
            return 1

    # Wait until SIGTERM.
    await signaled.wait()

    logger.info("exiting")
    return 0


def main():
    asyncio.run(service_main())
