# Copyright (C) 2026 Parrot
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

"""
Mission Serial Peripheral – LED Blink Service (Morse Code)

This script manages bidirectional serial communication between the drone and an
Arduino-compatible board. It periodically sends the word "PARROT" via /dev/ttyEXT,
logs all responses from the board (ACK and DONE), and waits for the "DONE" signal
before sending the next command.
"""

import logging
import serial
import ulog
import time
import signal
import sys

name = "led_blink"
WORD = "PARROT"
ulog.setup_logging(name)
logger = logging.getLogger(name)


class MorseController:
    def __init__(self):
        self.serial = None
        self.running = True

    def stop(self):
        self.running = False

    def run(self):
        logger.info("Morse Controller started")

        while self.running:
            # Check/Open serial
            if self.serial is None or not self.serial.is_open:
                try:
                    self.serial = serial.Serial(
                        port="/dev/ttyEXT", baudrate=115200, timeout=1.0
                    )
                    logger.info(f"Connected to board on {self.serial.port}")
                except Exception:
                    self.serial = None
                    time.sleep(2)
                    continue

            # Send command
            try:
                logger.info(f"Drone -> Board: Sending word '{WORD}'")
                self.serial.write(f"{WORD}\n".encode())
            except Exception as e:
                logger.warning(f"Failed to send: {e}")
                if self.serial:
                    self.serial.close()
                self.serial = None
                continue

            # Wait for DONE before sending next command
            try:
                while self.running:
                    line = self.serial.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        logger.info(f"Board -> Drone: {line}")
                        if line == "DONE":
                            break
            except Exception as e:
                logger.warning(f"Read error: {e}")
                if self.serial:
                    self.serial.close()
                self.serial = None
                continue

            time.sleep(10)


def main():
    controller = MorseController()

    def signal_handler(sig, frame):
        logger.info("Stopping service...")
        controller.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    controller.run()


if __name__ == "__main__":
    main()
