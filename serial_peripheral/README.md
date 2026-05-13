# Mission Serial Peripheral

## Overview

The `serial_peripheral` mission demonstrates how to connect an external module to the drone and communicate with it over serial. The drone sends text commands to the peripheral via `/dev/ttyEXT`, and the peripheral processes them — in this example, an Arduino-compatible board translates the received text into Morse code and blinks it on its built-in LED.

## Arduino Firmware

The provided firmware handles Morse code translation and serial communication:

* **Generic Morse Logic:** Supports the full alphabet (A-Z) and numbers (0-9).
* **Automatic Hardware Detection:** Automatically detects the appropriate LED pin for various boards (e.g., Lolin32 Lite, ESP32-C6-Zero).
* **Acknowledgment:** Sends a confirmation message ("ACK: Blinking [word]") back to the drone before starting the LED sequence.

## Python Backend Service (Air SDK)

The Python backend runs as a background service on the drone:

* **Command Controller:** Periodically sends text to the peripheral via the serial port.
* **Serial Logging:** Monitors the connection via `/dev/ttyEXT` and forwards all feedback to the drone's system logs (`ulog`).
* **Robust Connection:** Automatically manages serial port connection and handles peripheral resets or reconnections.

## Files

- `serial_peripheral.ino` – Morse code firmware for Arduino-compatible boards.
- `main.py` – Air SDK background service for command management and logging.

## Usage

1. Flash the firmware (`serial_peripheral.ino`) onto your board using the Arduino IDE.
2. Connect the peripheral to the drone's USB-C port.
3. Build and install the Air SDK mission on the drone.
4. Monitor the communication in the drone logs:
   ```bash
   log | grep led_blink  # service name
   ```

## Configuration

- **Serial Port:** `/dev/ttyEXT`.
- **Baudrate:** 115200.
- **Morse Unit Timing:** 200ms (dot duration).
