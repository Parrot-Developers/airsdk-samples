# Image Classifier mission

Reference sample for the Air SDK neural network API: on-board image
classification with a quantized `ResNet50` model. Every fresh camera frame is
preprocessed to a normalized RGB tensor, run through the network, and the
top-5 classes are logged (`Top1: <label> (<score>%)`, ...).

It ships as **seven missions** under `image_classifier/`, not one: the same
task across the three mission languages (`cpp`, `c`, `py`), in two inference
variants each, plus a seventh that adds an interactive Mission UI panel.

## What it does

1. **Input** — a camera source is created for the `fcam_airsdk` stream
   (`airsdk::neural_network::input::makeVipcSource`). A worker thread pulls
   the freshest frame with `waitSample()`, keeping the (CPU-heavy)
   preprocessing off the mission loop.
2. **Preprocess** (`preProcessFrame()`) — the NV12 frame is center-cropped,
   converted to RGB, resized to 224x224 and normalized with the ImageNet
   mean/std. C++ (OpenCV) crops the Y/UV planes *before* the color
   conversion, so `cvtColor` only touches the region that survives the crop.
3. **Infer** — either the recommended `infer()` (non-blocking, result comes
   back in `onInferResult`) or the manual `allocateNNBufSync` /
   `loadInputBlobs` / `run` / `getOutputBlobs` sequence (blocking, driven from
   a worker thread), depending on the variant.
4. **Postprocess** — `softmax` + arg-sort on the logits, top-5 labels (from
   `classes_names.csv`) logged.

## Missions

| Mission | Language | Inference path |
|---|---|---|
| `cpp` / `c` / `py` | C++ / C / Python | `infer()` — recommended, non-blocking |
| `advanced_cpp` / `advanced_c` / `advanced_py` | C++ / C / Python | manual path — allocate/load/run/get |
| `advanced_with_mission_ui_cpp` | C++ | manual path, plus a Mission UI side panel |

Start with `cpp`/`c`/`py`: it's the recommended way to run a network on a
mission. The `advanced_*` variants show the lower-level manual path for
missions that need step-by-step control over the shared buffers.
`advanced_with_mission_ui_cpp` shows how to surface results to the pilot
instead of just the logs, and how to drive the mission back from the panel.

## Getting started

Build and install any variant as usual, then watch the classification in the
logs:

```bash
adb shell
log | grep Top1
```

For `advanced_with_mission_ui_cpp` specifically: unlike the other six, it
does **not** run inference or deactivate native NNs on its own — open its
Mission UI side panel and press **Start**, or nothing will show up in the
logs. See "Mission UI controls" below.

The Python missions need a drone-compatible `numpy` wheel: the generic
`manylinux_*_aarch64` wheel `pip` would otherwise fetch is incompatible with
the drone's libc (`undefined symbol` / GLIBC error at import). Provide one
built for the drone target in the mission's `wheelhouse/` directory; the Air
SDK build passes it to `pip` via `-f wheelhouse`.

## Mission UI controls (`advanced_with_mission_ui_cpp`)

The panel (`assets/static/ff8-side-panel.html` + `script.js`) is more than a
passive display: it drives the mission back over the same libwebsockets
server used to broadcast results (`appserver.hpp`/`.cpp`). Two buttons, each
a `register_command` handler in `main.cpp`:

* **Native NNs** — sends `{"cmd": "set_native_nns_deactivated", "value": bool}`,
  toggles `deactivateNativeNns()`.
* **Inference** — sends `{"cmd": "set_inference_enabled", "value": bool}`,
  gates the worker thread's inference calls.

Both start **off**: the network object is created at mission startup like
every other variant (creating it costs nothing), but nothing is deactivated
and no inference runs until the pilot presses the corresponding button. This
is deliberate — unlike the other six missions, which auto-deactivate native
NNs as soon as the network is ready, this one only ever calls
`deactivateNativeNns()` from the panel, so every state change is a direct,
visible result of a click, not something that happens silently in the
background.

After every command (and after every `onStatus` readiness change), the
mission broadcasts `{"status": {"native_nns_deactivated": bool,
"inference_enabled": bool}}` to all connected panels. The buttons reflect
that broadcast, not what was optimistically clicked — `deactivateNativeNns()`
itself is fire-and-forget, so this is the only way a panel finds out whether
a toggle actually took effect. A panel that connects after a toggle already
changed only sees it on the *next* change, not the current state; fine for a
sample, not for a mission that needs a reliable initial sync.

## Configuration files

* `mission.yaml` — mission descriptor; declares the service and, for Python,
  the `numpy` dependency; `advanced_with_mission_ui_cpp` additionally depends
  on `libwebsockets` and `json`.
* `assets/etc/services/classification/ResNet50_quantized.dlc` — the
  classification model: torchvision's `ResNet50`, exported and quantized to
  the Qualcomm SNPE `.dlc` format for the drone NPU.
* `assets/etc/services/classification/classes_names.csv` — newline-separated
  labels loaded at startup.

## Notes and caveats

* Preprocessing assumes NV12 frames from the front camera — guaranteed by
  `fcam_airsdk`, the stream every variant reads from. Reading a different
  source instead, check `sample.format` rather than assuming NV12.
* `fcam_airsdk`'s measured frame delivery rate on the physical drone can fall
  well short of its documented spec (30 fps video / 24 fps photo) — one
  measurement on `advanced_cpp` showed `waitSample()` itself returning at
  ~18 fps, confirmed by timing frame arrivals before any preprocessing ran.
  This is a property of the stream, not something mission-side preprocessing
  fixes. The drone's internal detection feed (`fcam_detect`, smaller
  resolution) measured close to 30 fps in the same conditions, but it's not a
  documented Air SDK contract — resolution, format or availability can
  change without notice — so treat it as a fallback, not a default choice.
* One inference runs at a time per network; the `infer()` variants
  deliberately drop frames while one is in flight (`-EBUSY`).
* `infer()`'s very first call is floored to a hidden 30s timeout internally
  by the SDK to cover the model load; `INFERENCE_TIMEOUT_MS` in the samples
  only needs to size steady-state latency (5s), not the cold start.
* The input shape and output layer (`/fc/Gemm`) match the provided ResNet50
  DLC. Swapping the model means updating `Settings` (model file, output
  layer) and the tensor shape accordingly.
* Measured inference throughput varies with the model, the hardware and the
  input resolution — measure on your own setup rather than assuming a fixed
  fps.
