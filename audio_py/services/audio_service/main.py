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

"""
Mission audio_py — record-then-playback audio service.

Python equivalent of the C++ ``audio_cpp`` mission. Records ~5 s of audio from
a USB audio device connected to the drone, writes it to
``$MISSION_DATA/record.wav``, then plays the recording back until a stop signal
(SIGINT/SIGTERM) is received.

Whereas the C++ mission builds an asynchronous PDrAW pipeline (ALSA source ->
AAC encoder -> MP4 muxer, then demuxer -> audio renderer), this service uses the
ALSA bindings (``pyalsaaudio`` / module ``alsaaudio``, which links the drone's
native ``libasound``) for capture/playback and the stdlib ``wave`` module for
file I/O. The blocking, synchronous flow replaces the C++ pomp step state
machine.

Note: the recording is a WAV file rather than the C++ mission's ``.m4a`` (no AAC
encoder in this stack); the record->playback behaviour is unchanged.

Runs only on a physical drone (Anafi UKR) with a USB audio device connected.
"""

import contextlib
import logging
import os
import signal
import sys
import time
import wave

import alsaaudio
import ulog

# ulog process name is limited to 15 characters. Same tag as the C++ mission,
# so both variants are read from the logs the same way.
PROCESS_NAME = b"audio_service"

# Recording duration, matching the C++ mission's RECORDING_TIME (5000 ms).
RECORDING_TIME_S = 5.0

# PCM format, matching recorder.cpp: 44100 Hz, 16-bit signed LE, stereo.
SAMPLE_RATE = 44100
CHANNELS = 2
SAMPLE_WIDTH = 2  # bytes per sample (S16_LE)
PERIOD_SIZE = 1024

# Bounded retries on ALSA read()/write() errors, so a persistent failure ends
# the loop instead of spinning at 100 % CPU.
MAX_ALSA_RETRIES = 10
ALSA_RETRY_DELAY_S = 0.01

# Hardware addresses, matching the C++ mission.
CAPTURE_DEVICE = "hw:1"  # raw capture device
PLAYBACK_DEVICE = "plughw:1"  # "plughw" for automatic format conversion

ulog.setup_logging(PROCESS_NAME)
logger = logging.getLogger("main")

# Stop flag, set to True by the signal handler to exit cleanly.
stop = False


def sig_handler(signum, _frame):
    """Request a clean shutdown on SIGINT/SIGTERM."""
    global stop
    logger.info("Signal %d received", signum)
    stop = True


def wait_for_stop(wake_fd):
    """Block until a stop signal has been received.

    Reading the wake-up pipe parks the process instead of polling, and the byte
    signal delivery leaves in it unblocks the read whether it landed before or
    after the read started -- unlike ``signal.pause()``, which can miss one.
    """
    while not stop:
        os.read(wake_fd, 4096)


def record(file_path):
    """Capture ~RECORDING_TIME_S of audio from the USB device into a WAV file.

    Returns once the duration has elapsed or a stop request is received.

    :return: True if a recording was written, False if the capture device
        could not be opened.
    """
    logger.info("Starting recording to %s", file_path)

    try:
        capture = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE,
            device=CAPTURE_DEVICE,
            channels=CHANNELS,
            rate=SAMPLE_RATE,
            format=alsaaudio.PCM_FORMAT_S16_LE,
            periodsize=PERIOD_SIZE,
        )
    except (alsaaudio.ALSAAudioError, OSError) as exc:
        # Usually no USB audio device connected. Report it and let the caller
        # idle: an exiting service is relaunched, i.e. a restart loop.
        logger.error("Cannot open capture device %s: %s", CAPTURE_DEVICE, exc)
        return False

    # closing() so the device is released even if wave.open() raises.
    with contextlib.closing(capture), wave.open(file_path, "wb") as wav:
        wav.setnchannels(CHANNELS)
        wav.setsampwidth(SAMPLE_WIDTH)
        wav.setframerate(SAMPLE_RATE)

        failures = 0
        deadline = time.monotonic() + RECORDING_TIME_S
        while not stop and time.monotonic() < deadline:
            # A SIGINT/SIGTERM can interrupt the blocking read(); tolerate the
            # resulting error and let the loop re-check the stop flag instead of
            # crashing the service.
            try:
                length, data = capture.read()
            except (alsaaudio.ALSAAudioError, OSError) as exc:
                if stop:
                    break
                failures += 1
                if failures > MAX_ALSA_RETRIES:
                    logger.error(
                        "Capture failed %d times in a row, giving up: %s",
                        failures,
                        exc,
                    )
                    break
                time.sleep(ALSA_RETRY_DELAY_S)
                continue
            failures = 0
            if length > 0:
                wav.writeframes(data)

    logger.info("Recording stopped")
    return True


def play(file_path):
    """Play back the recorded WAV file.

    :return: True if the file was played, False if it cannot be played on the
        playback device.
    """
    logger.info("Starting playback of %s", file_path)

    with wave.open(file_path, "rb") as wav:
        # Channels and rate come from the file, so the sample width must
        # match the format the device is opened with.
        if wav.getsampwidth() != SAMPLE_WIDTH:
            logger.error(
                "%s has a sample width of %d byte(s), expected %d (S16_LE)",
                file_path,
                wav.getsampwidth(),
                SAMPLE_WIDTH,
            )
            return False

        try:
            playback = alsaaudio.PCM(
                type=alsaaudio.PCM_PLAYBACK,
                device=PLAYBACK_DEVICE,
                channels=wav.getnchannels(),
                rate=wav.getframerate(),
                format=alsaaudio.PCM_FORMAT_S16_LE,
                periodsize=PERIOD_SIZE,
            )
        except (alsaaudio.ALSAAudioError, OSError) as exc:
            # Same as for the capture device: report and let the caller idle
            # rather than exiting into a restart loop.
            logger.error(
                "Cannot open playback device %s: %s", PLAYBACK_DEVICE, exc
            )
            return False

        with contextlib.closing(playback):
            failures = 0
            data = wav.readframes(PERIOD_SIZE)
            while not stop and data:
                # A SIGINT/SIGTERM can interrupt the blocking write(); tolerate
                # the resulting error and let the loop re-check the stop flag
                # instead of crashing the service.
                try:
                    playback.write(data)
                except (alsaaudio.ALSAAudioError, OSError) as exc:
                    if stop:
                        break
                    # The frames were not written: retry the same buffer,
                    # bounded so a persistent failure ends the loop.
                    failures += 1
                    if failures > MAX_ALSA_RETRIES:
                        logger.error(
                            "Playback failed %d times in a row, giving up: %s",
                            failures,
                            exc,
                        )
                        break
                    time.sleep(ALSA_RETRY_DELAY_S)
                    continue
                failures = 0
                data = wav.readframes(PERIOD_SIZE)

    logger.info("Playback stopped")
    return True


def main():
    # Initialisation code.
    #
    # The service is automatically started by the drone when the mission is
    # loaded.
    logger.info("Hello from audio_service")

    # Wake-up pipe: signal delivery writes a byte, wait_for_stop() reads it.
    wake_r, wake_w = os.pipe()
    os.set_blocking(wake_w, False)
    signal.set_wakeup_fd(wake_w)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    signal.signal(signal.SIGPIPE, signal.SIG_IGN)

    # Record into the persistent mission data directory, so the file survives a
    # reboot and shows up under the "missions-data" folder when the drone is
    # connected to a computer through USB.
    mission_data = os.environ.get("MISSION_DATA")
    if mission_data is None:
        logger.error("MISSION_DATA environment variable is not set")
        return 1
    record_path = os.path.join(mission_data, "record.wav")

    # No playback if recording failed or a stop was requested meanwhile.
    if record(record_path) and not stop:
        play(record_path)

    # Stay alive until SIGINT/SIGTERM, mirroring the C++ main loop: a service
    # which exits on its own is relaunched by the system.
    if not stop:
        logger.info("Waiting for stop signal")
        wait_for_stop(wake_r)

    return 0


if __name__ == "__main__":
    sys.exit(main())
