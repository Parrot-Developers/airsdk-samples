# Audio mission example

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

The `audio_cpp` mission records ~5 s of audio from a USB audio device connected
to the drone, writes it to `$MISSION_DATA/record.m4a`, then plays the recording
back through the same device until the mission is stopped.

Both steps are built with
[PDrAW](https://developer.parrot.com/docs/pdraw/overview.html) (`libpdraw`),
the drone's audio/video pipeline library:

* **Recording:** `<hw:1>` → `IAlsaSource` → `IAudioEncoder` → `IMuxer` →
  `<record.m4a>` (PCM 44100 Hz / 16-bit / stereo, encoded to AAC-LC and muxed
  into MP4).
* **Playback:** `<record.m4a>` → `IDemuxer` → `IAudioRenderer` → `<plughw:1>`.

The capture side uses the raw device `hw:1`, the rendering side uses `plughw:1`
so ALSA inserts the format conversions the sound card may need.

There is a Python equivalent of this mission in [`audio_py`](../audio_py),
built on the ALSA bindings instead of PDrAW.

## C++ service (Air SDK)

The service `audio_service` starts as soon as the mission is loaded — the drone
does not need to take off — and drives a four-phase sequence from its
`pomp::Loop`: start the recorder, record for 5 s, stop the recorder, play back.

PDrAW elements are created step by step: each element needs the media produced
by the previous one, so every step is scheduled from the callback announcing
that media (`alsaSourceReadyToPlay`, `onMediaAdded`, `demuxerReadyToPlay`, …)
rather than created up front. On `SIGTERM`, the active pipeline is torn down in
the reverse order and the loop is pumped until both pipelines report
`stopped()`, so the muxer is closed and the recorded file is coherent before
exiting.

## Files

* `services/audio_service/main.cpp` – signal handling, phase sequencing,
  cleanup.
* `services/audio_service/pdraw_pipeline.hpp/.cpp` – base class shared by both
  pipelines: PDrAW instance lifecycle, step scheduling, listener boilerplate.
* `services/audio_service/recorder.hpp/.cpp` – ALSA source, encoder and muxer.
* `services/audio_service/player.hpp/.cpp` – demuxer and audio renderer.

## Configuration

Everything is set in the static parameter structs of `recorder.cpp` and
`player.cpp`:

* **Capture device:** `hw:1` — **Playback device:** `plughw:1`
* **PCM format:** 44100 Hz, 16-bit signed little-endian, stereo, 1024 samples
  per period
* **Encoder:** FDK AAC-LC, CBR, 320 kbit/s max
* **Recording duration:** 5000 ms (`RECORDING_TIME` in `main.cpp`)

## Run the Audio mission sample

The mission is built and installed with:

```bash
$ airsdk build
$ airsdk install --default
```

The recording is written to the
[mission data directory](https://developer.parrot.com/docs/airsdk/general/persistent_data_and_storage.html)
(`MISSION_DATA`), which is persistent and available over MTP/PTP under
`missions-data` when the drone is connected to a computer with a USB cable.

Follow the record-then-play sequence in the drone logs:

```bash
$ log | grep -E 'audio_service|recorder|player|pdraw_pipeline'
```

> **_NOTE_**: For more information on Air SDK, please refer to the
[Air SDK documentation](https://developer.parrot.com/docs/airsdk/general/overview.html).
