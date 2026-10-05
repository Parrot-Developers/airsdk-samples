# Copyright (c) 2026 Parrot Drones SAS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
# * Neither the name of the Parrot Company nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.

import asyncio
import json
import logging
import math
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
PROCESS_NAME = "mission_ui_py"
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

        # Attitude/altitude/velocity: single latest-sample values, not averaged.
        self.roll = telemetry_binding.types.float32_t(0.0)
        self.pitch = telemetry_binding.types.float32_t(0.0)
        self.yaw = telemetry_binding.types.float32_t(0.0)
        self.altitude_agl = telemetry_binding.types.float32_t(0.0)
        self.velocity_x = telemetry_binding.types.float32_t(0.0)
        self.velocity_y = telemetry_binding.types.float32_t(0.0)

        # Previous altitude sample, to derive a vertical speed (VSI).
        self.previous_altitude = None

        # Register telemetry data sources
        for i in range(10):
            self.single_sample_consumer.reg(self.gyro_x[i], f"sensors_imu.gyro[{i}].x")
            self.single_sample_consumer.reg(self.gyro_y[i], f"sensors_imu.gyro[{i}].y")
            self.single_sample_consumer.reg(self.gyro_z[i], f"sensors_imu.gyro[{i}].z")
            self.single_sample_consumer.reg(self.acc_x[i], f"sensors_imu.acc[{i}].x")
            self.single_sample_consumer.reg(self.acc_y[i], f"sensors_imu.acc[{i}].y")
            self.single_sample_consumer.reg(self.acc_z[i], f"sensors_imu.acc[{i}].z")

        self.single_sample_consumer.reg(self.roll, "drone_controller.attitude_euler_angles.roll")
        self.single_sample_consumer.reg(self.pitch, "drone_controller.attitude_euler_angles.pitch")
        self.single_sample_consumer.reg(self.yaw, "drone_controller.attitude_euler_angles.yaw")
        self.single_sample_consumer.reg(self.altitude_agl, "drone_controller.altitude_agl")
        self.single_sample_consumer.reg(self.velocity_x, "drone_controller.linear_velocity_global.x")
        self.single_sample_consumer.reg(self.velocity_y, "drone_controller.linear_velocity_global.y")

        self.single_sample_consumer.regComplete()
        logger.info("Telemetry initialized")

    def get_averaged_telemetry(self):
        """Retrieves and computes the average of telemetry data."""
        self.single_sample_consumer.getSample(
            telemetry_binding.types.timespec(0, 0),
            telemetry_binding.Method.Latest,
        )

        altitude = float(self.altitude_agl)
        # Broadcast cadence is fixed at 0.5s (see telemetry_broadcast), so that's dt.
        vertical_speed = 0.0 if self.previous_altitude is None else (altitude - self.previous_altitude) / 0.5
        self.previous_altitude = altitude

        return {
            "sensors_imu": {
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
            },
            "attitude": {
                "roll": float(self.roll),
                "pitch": float(self.pitch),
                "yaw": float(self.yaw),
            },
            "altitude_agl": altitude,
            # Ground speed (GPS/odometry-derived), not true airspeed: a drone
            # has no pitot tube to measure speed relative to the air.
            "ground_speed": math.hypot(float(self.velocity_x), float(self.velocity_y)),
            "vertical_speed": vertical_speed,
        }


def telemetry_broadcast(stop_event):
    """Telemetry broadcasting thread; dispatches onto the FastAPI event loop."""
    telemetry_manager = TelemetryManager()

    while not stop_event.is_set():
        telemetry_data = telemetry_manager.get_averaged_telemetry()
        message = json.dumps({"telemetry": telemetry_data})

        # server.loop belongs to uvicorn, running in the main thread: hop
        # onto it instead of driving the coroutine from a private loop here.
        future = asyncio.run_coroutine_threadsafe(
            server.manager.broadcast(message), server.loop
        )
        try:
            future.result()
        except Exception as e:
            logger.warning(f"Broadcast failed: {e}")

        time.sleep(0.5)


# Command handlers for WebSocket messages
async def handle_send_msg(websocket: WebSocket, jsonMessage: dict):
    """Responds to a message by confirming receipt."""
    logger.info("Responding ack to controller...")
    await server.manager.send_message(
        websocket,
        json.dumps(
            {"sendMsg": "Drone received: " + jsonMessage.get("value", "")}
        ),
    )


async def handle_ping_pong(websocket: WebSocket, jsonMessage: dict):
    """Handles a ping-pong exchange with the drone."""
    logger.info("PingPong with the drone")
    pingPongState = jsonMessage.get("value")
    pingPongResponse = "PONG" if pingPongState == "PING" else "PING"
    await server.manager.send_message(
        websocket, json.dumps({"pingPong": pingPongResponse})
    )


async def handle_trigger_webviews(websocket: WebSocket, jsonMessage: dict):
    """Triggers a webview for all clients."""
    logger.info("Broadcasting message to all clients")
    await server.manager.broadcast(json.dumps({"triggerWebviews": ""}))


# Mapping command names to their handlers
server.register_command("pingPong", handle_ping_pong)
server.register_command("sendMsg", handle_send_msg)
server.register_command("triggerWebviews", handle_trigger_webviews)


def main():
    stop_event = threading.Event()
    telemetry_thread = None

    # uvicorn installs its own SIGINT/SIGTERM handlers once it runs, so
    # telemetry start/stop is wired to FastAPI's startup/shutdown lifecycle
    # instead of a signal handler (which uvicorn would just overwrite).
    def start_telemetry():
        nonlocal telemetry_thread
        telemetry_thread = threading.Thread(
            target=telemetry_broadcast, args=(stop_event,), daemon=True
        )
        telemetry_thread.start()

    def stop_telemetry():
        logger.info("Stopping telemetry...")
        stop_event.set()
        if telemetry_thread:
            telemetry_thread.join()

    airsdk = Publisher(UID)

    try:
        server.start(
            PROCESS_NAME, airsdk, on_ready=start_telemetry, on_shutdown=stop_telemetry
        )
    except Exception as e:
        logger.error(f"Error occurred: {e}")
        stop_telemetry()
        sys.exit(1)
