# Copyright (C) 2025 Parrot

import asyncio
import json
import logging
import signal
import statistics
import sys
import telemetry_binding
import threading
import time
import ulog

from airsdk_py import Publisher
from fastapi import WebSocket
from webserver.uid import UID
from webserver.appserver import WebSocketServer

# Process configuration
PROCESS_NAME = "mission_ui"
ulog.setup_logging(PROCESS_NAME)
logger = logging.getLogger(PROCESS_NAME)

# Initialize WebSocket server
server = WebSocketServer()


# Manages telemetry data and provides averaged sensor values
class TelemetryManager:
    def __init__(self):
        self.single_sample_consumer = telemetry_binding.Consumer("/dev/shm")

        # Initialize gyro and accelerometer arrays with placeholder values
        self.gyro_x = [telemetry_binding.types.float32_t(0.0) for _ in range(10)]
        self.gyro_y = [telemetry_binding.types.float32_t(0.0) for _ in range(10)]
        self.gyro_z = [telemetry_binding.types.float32_t(0.0) for _ in range(10)]
        self.acc_x = [telemetry_binding.types.float32_t(0.0) for _ in range(10)]
        self.acc_y = [telemetry_binding.types.float32_t(0.0) for _ in range(10)]
        self.acc_z = [telemetry_binding.types.float32_t(0.0) for _ in range(10)]

        # Register telemetry data sources
        for i in range(10):
            self.single_sample_consumer.reg(self.gyro_x[i], f"sensors_imu.gyro[{i}].x")
            self.single_sample_consumer.reg(self.gyro_y[i], f"sensors_imu.gyro[{i}].y")
            self.single_sample_consumer.reg(self.gyro_z[i], f"sensors_imu.gyro[{i}].z")
            self.single_sample_consumer.reg(self.acc_x[i], f"sensors_imu.acc[{i}].x")
            self.single_sample_consumer.reg(self.acc_y[i], f"sensors_imu.acc[{i}].y")
            self.single_sample_consumer.reg(self.acc_z[i], f"sensors_imu.acc[{i}].z")

        self.single_sample_consumer.regComplete()
        logger.info("Telemetry initialized")

    def get_averaged_telemetry(self):
        """Retrieves and computes the average of telemetry data."""
        self.single_sample_consumer.getSample(
            telemetry_binding.types.timespec(0, 0),
            telemetry_binding.Method.Latest,
        )

        return {
            "gyro": {
                "x": statistics.mean(float(self.gyro_x[i]) for i in range(10)),
                "y": statistics.mean(float(self.gyro_y[i]) for i in range(10)),
                "z": statistics.mean(float(self.gyro_z[i]) for i in range(10)),
            },
            "acc": {
                "x": statistics.mean(float(self.acc_x[i]) for i in range(10)),
                "y": statistics.mean(float(self.acc_y[i]) for i in range(10)),
                "z": statistics.mean(float(self.acc_z[i]) for i in range(10)),
            },
        }


# Function to continuously broadcast telemetry data
def telemetry_broadcast(stop_event):
    """Telemetry broadcasting thread with a dedicated event loop."""
    telemetry_manager = TelemetryManager()

    # Create and set the loop for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    while not stop_event.is_set():  # Check if stop_event is set
        telemetry_data = telemetry_manager.get_averaged_telemetry()
        message = json.dumps({"telemetry": {"sensors_imu": telemetry_data}})

        # Run the broadcast task within the event loop
        loop.run_until_complete(server.manager.broadcast(message))
        time.sleep(0.5)  # Wait before the next broadcast


# Command handlers for WebSocket messages
async def handle_send_msg(websocket: WebSocket, jsonMessage: dict):
    """Responds to a message by confirming receipt."""
    logger.info("Responding ack to controller...")
    await server.manager.send_message(
        websocket,
        json.dumps({"sendMsg": "Drone received the message " + jsonMessage["value"]}),
    )


async def handle_ping_pong(websocket: WebSocket, jsonMessage: dict):
    """Handles a ping-pong exchange with the drone."""
    logger.info("PingPong with the drone")
    pingPongState = jsonMessage["value"]
    pingPongResponse = "PONG" if pingPongState == "PING" else "PING"
    await server.manager.send_message(
        websocket, json.dumps({"pingPong": pingPongResponse})
    )


async def handle_trigger_webviews(websocket: WebSocket, jsonMessage: dict):
    """Triggers a webview for all clients."""
    logger.info("Broadcasting message to all clients")
    await server.manager.broadcast(json.dumps({"triggerWebviews": ""}))


# Mapping command names to their handlers
server.register_command("sendMsg", handle_send_msg)
server.register_command("pingPong", handle_ping_pong)
server.register_command("triggerWebviews", handle_trigger_webviews)


def main():
    stop_event = threading.Event()  # Event to signal the thread to stop
    telemetry_thread = threading.Thread(
        target=telemetry_broadcast, args=(stop_event,), daemon=True
    )
    telemetry_thread.start()
    airsdk = Publisher(UID)

    # Handle SIGINT and SIGTERM
    def handle_signal(signal_num, frame):
        signal_name = signal.Signals(
            signal_num
        ).name  # Convert signal number to signal name
        logger.info(
            f"Signal {signal_name} ({signal_num}) received, stopping telemetry and exiting..."
        )
        stop_event.set()  # Signal the telemetry thread to stop
        telemetry_thread.join()  # Wait for the telemetry thread to finish
        sys.exit(0)  # Exit the program

    # Register SIGINT and SIGTERM handlers
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        server.start(PROCESS_NAME, airsdk)
    except Exception as e:
        logger.error(f"Error occurred: {e}")
        stop_event.set()
        telemetry_thread.join()
        sys.exit(1)
