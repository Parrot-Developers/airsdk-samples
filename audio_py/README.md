# Audio Python mission example

**This mission only works on a physical drone.**

A USB audio device must be connected to the drone **before the mission is
loaded**. See the
[external USB accessories](https://developer.parrot.com/docs/airsdk/general/external_usb_accessories.html)
documentation for the list of supported devices.

The mission addresses the ALSA card 1 (`hw:1` / `plughw:1`): card 0 is the
drone's own audio, so a single USB sound card lands on card 1. Check it on your
device from the developer shell with `aplay -l` and `arecord -l`, and update
the device constants if it shows up elsewhere.

## Overview

`audio_py` is the Python equivalent of the C++ [`audio_cpp`](../audio_cpp)
mission. It records ~5 s of audio from a USB audio device connected to the
drone, writes it to `$MISSION_DATA/record.wav`, then plays the recording back
until the mission is stopped.

Unlike the C++ mission, which builds an asynchronous PDrAW pipeline (ALSA
source → AAC encoder → MP4 muxer, then demuxer → audio renderer), this service
uses the ALSA bindings [`pyalsaaudio`](https://pypi.org/project/pyalsaaudio/)
(module `alsaaudio`, linking the drone's native `libasound`) for capture and
playback, and the stdlib `wave` module for file I/O:

* **Recording:** `<hw:1>` → `alsaaudio.PCM` capture → `wave` → `<record.wav>`
* **Playback:** `<record.wav>` → `wave` → `alsaaudio.PCM` playback →
  `<plughw:1>`

The output is a WAV file rather than an `.m4a` — no AAC encoder is usable from
Python on the drone, which has no ffmpeg/libav stack to mux MP4 — but the
end-to-end behaviour (record then play back) is the same.

## Python backend service (Air SDK)

The Python service `audio_service` starts as soon as the mission is
loaded — the drone does not need to take off:

* **Recording:** captures 5 s of 44100 Hz / 16-bit / stereo PCM from `hw:1` and
  writes it to `$MISSION_DATA/record.wav`.
* **Playback:** plays the recorded file back through `plughw:1`, then stays
  alive until a stop signal (`SIGINT`/`SIGTERM`). A service that exits on its
  own is considered to have crashed and is relaunched by the drone.
* **Logging:** forwards messages to the drone's system logs via `ulog`.

Capture and playback block in `read()` / `write()`, so a signal interrupts the
ALSA call instead of letting the loop reach its next iteration. Both loops
swallow the resulting error and re-check the stop flag, rather than dying with
a traceback. A persistent ALSA error is retried a bounded number of times
(`MAX_ALSA_RETRIES`) and then gives up, so the service never spins on it.

Failing to open the capture or playback device — typically because no USB
audio device is connected — is logged and the service then idles until it is
stopped, instead of exiting: a service which exits on its own is considered to
have crashed and is relaunched by the drone, which would turn a missing device
into a restart loop.

The recording is written to the
[mission data directory](https://developer.parrot.com/docs/airsdk/general/persistent_data_and_storage.html)
(`MISSION_DATA`), which is persistent and available over MTP/PTP under
`missions-data` when the drone is connected to a computer with a USB cable.

## Files

* `services/audio_service/main.py` – the whole service: record loop,
  playback loop, signal handling.
* `wheelhouse/` – prebuilt `pyalsaaudio` wheel for the drone architecture (see
  below).

## Configuration

All constants are at the top of `main.py`:

* **Capture device:** `hw:1` — **Playback device:** `plughw:1` (`plughw` for
  automatic format conversion)
* **PCM format:** 44100 Hz, 16-bit signed little-endian, stereo, 1024 samples
  per period
* **Recording duration:** 5 s (`RECORDING_TIME_S`)

## Build and install

The only pip dependency is `pyalsaaudio`, declared in `mission.yaml`
(`pip.requirements`). It does not publish a precompiled wheel for
`manylinux2014_aarch64`, so its wheel must be built once and stored in the
mission's `wheelhouse` directory before the mission itself can be built. The
requirement is pinned (`pyalsaaudio==0.11.0`) to the version of the committed
wheel: an unpinned requirement would pick a newer release from PyPI and build
it from source instead.

That wheel is already committed in [wheelhouse/](wheelhouse/), so building the
mission works out of the box. To rebuild it (to pick up a new version, or to
target another architecture), run from the mission's directory:

```bash
$ airsdk build-wheels
```

See the [airsdk-cli documentation][cli-wheels] for the full wheel-builder shell
(`search-dep`, `add-dep`, `build`).

[cli-wheels]: https://developer.parrot.com/docs/airsdk-cli/basic_usage.html#build-wheels

The command reads the `pip` requirements and the `targets` of `mission.yaml`,
starts a manylinux build container for every architecture the mission needs,
then opens a shell. `pyalsaaudio` compiles against ALSA, so install its system
dependency before building:

```
(wheel-builder) add-dep alsa-lib-devel
(wheel-builder) build
```

Leaving the shell removes the build containers; the wheels stay in
`wheelhouse/`. Note that `build` skips any requirement pip already finds a
wheel for, including one already in the wheelhouse, so delete the existing
wheel first when rebuilding it.

## Run the Audio Python mission sample

The mission is built and installed with:

```bash
$ airsdk build
$ airsdk install --default
```

`airsdk build` picks the wheels up from `wheelhouse/`. If one is missing, it
offers to run the wheel builder and resumes the build afterwards.

Monitor the service in the drone logs with:

```bash
$ log | grep audio_service   # ulog tag, same as the C++ mission
```

> **_NOTE_**: For more information on Air SDK, please refer to the
[Air SDK documentation](https://developer.parrot.com/docs/airsdk/general/overview.html).
