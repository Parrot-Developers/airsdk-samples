# object_detector viewer

Desktop app that plays the drone's live video and draws the `object_detector`
mission's detection boxes (class + score) on top, in OpenGL.

It does three things using only the **public pdraw**: get the stream, read the
per-frame detection metadata, and draw it over the video in OpenGL. It is shipped
**with the mission** (companion app, like the Arduino firmware inside its
mission) and is **self-contained** — it carries its own copy of the wire format,
so it builds against the public pdraw with nothing to edit.

## Quick start

The viewer builds against the **public** pdraw, via the groundsdk-tools
workspace (https://github.com/Parrot-Developers/groundsdk-tools). It is **not**
an Air SDK service (not listed in `mission.yaml`) — it is a plain pdraw app.

```bash
# 1. Get the groundsdk-tools workspace (once), somewhere OUTSIDE this repo:
mkdir ~/groundsdk-tools && cd ~/groundsdk-tools
repo init -u https://github.com/Parrot-Developers/groundsdk-tools-manifest
repo sync -j"$(nproc)"

# 2. Drop this folder in as a pdraw app:
cp -r <this-mission>/services/object_detector_viewer \
      packages/pdraw/apps/object_detector_viewer

# 3. Build JUST this app. NOTE: `-A <module>` (drive alchemy directly), NOT
#    `-t build` — the app is not in the product's default graph, so `-t build`
#    skips it.
./build.sh -p groundsdk-linux -j -A object_detector_viewer

# 4. Run it against the drone's live RTSP stream:
#    - physical drone, USB: 192.168.43.1
#    - simulated drone, Sphinx: 10.202.0.1
./out/groundsdk-linux/staging/native-wrapper.sh \
    object_detector_viewer rtsp://<drone-ip>/live
```

The video window opens with blue boxes + `"<class> <score>%"` labels.
`Ctrl+C` or `Esc` quits cleanly.

> Tip: prefix the run command with `ULOG_STDERR=y` to print the app's logs
> (selected media, errors) to the terminal.

## How it works

```
  object_detector mission (drone)              viewer (this app, PC)
  ───────────────────────────────              ─────────────────────
  Object detection on the AirSDK stream             libpdraw-backend
    → Detections protobuf                        → open RTSP stream
    → MetadataPublisher.push("object_detector")  → decode + GL render (AirSDK media)
        │                                        → render_overlay():
        │  Air SDK embeds it in the                  unpack vmeta proto
        ▼  "front-airsdk" stream's vmeta             → user[] by uid_hash
  video stream (RTSP) ────────────────────────►     → parse Detections
                      (frame metadata: vmeta proto)  → draw boxes + labels
```

The boxes travel **inside the video stream's frame metadata**, in the Parrot
Video Metadata protobuf (`vmeta`), `repeated UserMetadata user` field. Each entry
is keyed by `uid_hash` = Jenkins one-at-a-time hash of the metadata UID, and its
`data` is the `Detections` protobuf. Coordinates are normalized to `[0, 1]`.

Two details that make it work (both were non-obvious):

- **Detect and display the SAME stream.** Inference runs on the AirSDK camera
  stream (`fcam_airsdk`), so the mission registers its metadata on that stream's
  resource, **`front-airsdk`** (see `stream-server.cfg`), and the viewer selects
  the `Front camera (AirSDK)` media. Same framing ⇒ boxes align with the video.
  (The plain `front` media carries drone/camera telemetry but not our boxes.)
- **The metadata UID must not contain a dot.** `libtelemetry` rejects section
  names with `.` (`consumers won't work`), so the mission uses the dotless UID
  `"object_detector"` — **not** the mission UID `com.parrot.missions.…`. The
  viewer hashes the same `"object_detector"` string (`DETECTIONS_METADATA_UID`).

## Under the hood: the pdraw-backend calls, in order

`main.c` is a trimmed-down version of pdraw's own `apps/pdraw_desktop`. The
sequence below is the actual call order, with the real function/struct
names, so you can follow it straight in the code.

**1. Startup.** SDL2 opens a window and an OpenGL ES context
(`SDL_CreateWindow` + `SDL_GL_CreateContext`); `overlay_new()` compiles the
two GLES2 shader programs used for drawing (see below); `pdraw_be_new()`
creates the pdraw-backend instance, registered with `be_cbs`
(`media_added`/`media_removed`/`socket_created`); `pdraw_be_demuxer_new_from_url()`
opens the RTSP URL given on the command line, registered with `demuxer_cbs`.
Everything past this point is driven by callbacks, there is no manual
"connect then wait" step.

**2. Media negotiation.** Once pdraw has parsed the RTSP SDP, it calls
`demuxer_select_media_cb()` with the list of available medias (one per
camera/stream the drone exposes): this is where the viewer picks
**"Front camera (AirSDK)"** specifically, by matching `"AirSDK"` in the
media name, and returns a bitmask selecting just that one. This has to be
the *same* stream the mission runs inference on (`fcam_airsdk`/
`front-airsdk`), or the detection boxes would be computed against one
camera feed and drawn on a differently-framed one. `ready_to_play_cb()`
then fires once pdraw is ready, and calls `pdraw_be_demuxer_play()` to
actually start streaming.

**3. Media creation.** For the selected media, pdraw creates a CODED
element (the decoder's input, raw H.264/265 NAL units) and a RAW element
(the decoded frames pdraw hands to the renderer): `media_added_cb()` fires
for both, and only keeps the RAW one (`info->video.format ==
VDEF_FRAME_TYPE_RAW`), storing its id in `ctx.media_id`. Deliberately not
creating the renderer here: this callback can fire off the main thread,
while the renderer needs the OpenGL context, which is only current on the
thread that created it. `create_renderer()` is instead called from the main
loop once `ctx.media_id` is set, calling `pdraw_be_video_renderer_new()`
with `render_cbs` (just `render_overlay`) and a `PDRAW_VIDEO_RENDERER_FILL_MODE_FIT`
viewport.

**4. The render loop.** Each iteration: poll SDL events (quit/Esc trigger a
graceful close), clear the GL framebuffer, and call
`pdraw_be_video_renderer_render()`. This single call does the actual video
decode-and-draw *and*, before returning, calls back into `render_overlay_cb()`
with the frame's `vmeta_frame`, the render/content rectangles (needed to map
normalized box coordinates to pixels), and the view/projection matrices
(unused here, this app never repositions the video). `render_overlay_cb()`
just forwards straight to `overlay_render()` (see "How it works" above for
what that does): the video and the boxes are drawn as two separate GL passes
into the same framebuffer, video first (inside `pdraw_be_video_renderer_render`),
boxes second (the callback it triggers).

**5. Shutdown.** `request_close()` runs on the main/GL thread specifically
so it can call `pdraw_be_video_renderer_destroy()` there (same GL-context
constraint as creation). It then calls `pdraw_be_demuxer_close()`, an
*asynchronous* request: the demuxer is only actually destroyed once
`demuxer_close_resp_cb()` fires, confirming the RTSP client (and its
auto-reconnect logic) has fully stopped. Destroying it any earlier risks a
reconnect racing the teardown. This two-step close (destroy renderer now,
destroy demuxer later on confirmation) is copied from `pdraw_desktop`'s own
shutdown sequence.

**The overlay itself (`overlay.c`)** doesn't touch pdraw at all past the
`vmeta_frame` it's handed: `vmeta_frame_proto_get_unpacked()` unpacks the
frame's protobuf metadata, `tm->user[]` is scanned for the entry whose
`uid_hash` matches the mission's `DETECTIONS_METADATA_UID`
("object_detector", Jenkins-hashed), and its bytes are unpacked as the
`Detections` protobuf-c message (generated from `detection.proto`). Each
box is drawn as a `GL_LINE_LOOP` (a tiny custom shader, since GLES2 has no
built-in rectangle primitive) and each label as textured quads sampling a
hand-built 8x8 font atlas texture (`font8x8.h`, since GLES2 also has no text
primitive at all).

## Files

| File | Role |
|---|---|
| `main.c` | SDL2 window + GLES3 context, pdraw-backend demuxer/renderer, clean teardown |
| `overlay.c` / `overlay.h` | `render_overlay` hook: read vmeta `user[]`, draw boxes + labels (GLES2) |
| `font8x8.h` | public-domain 8x8 bitmap font (GL has no text primitive; the only complex bit) |
| `detection.proto` | wire format — **keep in sync** with the mission's `msghub/object_detector/detection.proto` |
| `atom.mk` | builds the app + compiles `detection.proto` (protobuf-c) |

## pdraw components used

Public pdraw: https://github.com/Parrot-Developers/pdraw

| Component | Use |
|---|---|
| `libpdraw` / `libpdraw-backend` | RTSP demux, H.264/265 decode, OpenGL video renderer |
| `apps/pdraw_desktop` | reference app this viewer is modeled on (SDL2 + GL, `render_overlay`) |
| `libvideo-metadata` (`vmeta`) | parse the frame-metadata protobuf, read `user[]` |

The key hook is the renderer's `render_overlay` callback: pdraw renders the
frame, then calls us with the `vmeta_frame` and the render geometry — we unpack
the proto, match our `UserMetadata` by `uid_hash`, and draw with GLES2.

## Upstreaming into pdraw

Because it carries its own `detection.proto`, moving it into the public pdraw
repo is a single copy — nothing to edit:

```bash
cp -r services/object_detector_viewer  <pdraw>/apps/object_detector_viewer
```
