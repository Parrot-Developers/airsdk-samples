"""
Copyright (C) 2025 Parrot

Minimal metadata sender example in Python (stream + record)
"""

import asyncio
import logging
import signal
import random
import ulog
from airsdk_py import MetadataPublisher, MetadataPublisherListener

PROCESS_NAME = "metadata_sender_py"
UID = "com.parrot.missions.samples.user_metadata_py"
STREAM_UID = "dynamic_stream_value"
RECORD_UID = "dynamic_record_value"
RESOURCE_ID = "front"
CAMERA_ID = 1
TRACK_NAME = "DefaultVideo"


# Listener to log metadata updates
class MyMetadataListener(MetadataPublisherListener):
    def onStreamUserMetadataList(self, resource_path, user_metadatas):
        logging.getLogger("main").info(
            f"Stream metadata list updated: resource={resource_path} values={user_metadatas}"
        )

    def onRecordUserMetadataList(self, camera_id, track_name, user_metadatas):
        logging.getLogger("main").info(
            f"Record metadata list updated: camera={camera_id} track={track_name} values={user_metadatas}"
        )


async def service_main():
    ulog.setup_logging(PROCESS_NAME)
    logger = logging.getLogger("main")
    logger.info("Starting metadata sender")

    listener = MyMetadataListener()
    mp = MetadataPublisher(UID, listener)

    max_wait_ms = 5000
    wait_step_ms = 100
    waited = 0

    # Add stream metadata
    while True:
        res = mp.add_stream_user_metadata(STREAM_UID, 1, RESOURCE_ID)
        if res == 0:
            logger.info(f"Stream metadata '{STREAM_UID}' successfully added")
            break
        else:
            logger.warning(
                f"Failed to add stream metadata '{STREAM_UID}', error={res}, retrying..."
            )
            await asyncio.sleep(wait_step_ms / 1000)
            waited += wait_step_ms
            if waited >= max_wait_ms:
                logger.error(f"Cannot add stream metadata '{STREAM_UID}'")
                return 1

    # Add record metadata
    waited = 0
    while True:
        res = mp.add_record_user_metadata(RECORD_UID, 1, CAMERA_ID, TRACK_NAME)
        if res == 0:
            logger.info(f"Record metadata '{RECORD_UID}' successfully added")
            break
        else:
            logger.warning(
                f"Failed to add record metadata '{RECORD_UID}', error={res}, retrying..."
            )
            await asyncio.sleep(wait_step_ms / 1000)
            waited += wait_step_ms
            if waited >= max_wait_ms:
                logger.error(f"Cannot add record metadata '{RECORD_UID}'")
                return 1

    run = True

    def sig_handler(*_):
        nonlocal run
        run = False

    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, sig_handler)
    loop.add_signal_handler(signal.SIGINT, sig_handler)

    # Main loop: push random metadata
    while run:
        random_value = random.randint(0, 100)
        try:
            mp.push(STREAM_UID, bytes([random_value]))
            logger.info(f"Pushed stream metadata '{STREAM_UID}' = {random_value}")
        except Exception as e:
            logger.error("Failed to push stream metadata: %s", str(e))

        try:
            mp.push(RECORD_UID, bytes([random_value]))
            logger.info(f"Pushed record metadata '{RECORD_UID}' = {random_value}")
        except Exception as e:
            logger.error("Failed to push record metadata: %s", str(e))

        await asyncio.sleep(0.033)  # ~30 FPS

    mp.stop_metadata_publisher()
    logger.info("Metadata sender exited")
    return 0


def main():
    asyncio.run(service_main())
