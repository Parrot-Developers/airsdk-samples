"""
Copyright (c) 2026 Parrot Drones SAS

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.
* Neither the name of the Parrot Company nor the names
  of its contributors may be used to endorse or promote products
  derived from this software without specific prior written
  permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.

Image classifier example in Python: consumes camera frames, preprocesses them
(NV12 -> normalized RGB 224x224) and runs a non-blocking NeuralNetwork.infer().
A worker thread pulls the freshest frame and does the (CPU-heavy) preprocessing
off the loop, so frame reception is never blocked; the top-5 result comes back
via Listener.onInferResult. Results are logged.
"""

import asyncio
import errno
import logging
import signal
import threading

import numpy as np
import ulog
from airsdk_py import (
    BlobAttributes,
    NeuralNetwork,
    NeuralNetworkListener,
    Settings,
    Tensor,
    make_vipc_source,
)

PROCESS_NAME = "image_classifier_py"
UID = "com.parrot.missions.samples.image_classifier_py"
BASE = f"/mnt/missions/{UID}/payload/etc/services/classification"
MODEL_FILE = f"{BASE}/ResNet50_quantized.dlc"
LABELS_FILE = f"{BASE}/classes_names.csv"
STREAM_NAME = "fcam_airsdk"
INPUT_DIM = 224
# Steady-state timeout: infer()'s first call is floored to 30s internally by
# the SDK to cover the model load, so this only needs to size normal
# inference latency.
INFER_TIMEOUT_MS = 5000

_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
_STD = np.array([0.229, 0.224, 0.225], np.float32)


class Listener(NeuralNetworkListener):
    """Readiness/errors and inference results, on the network loop thread."""

    def __init__(self, labels):
        super().__init__()
        self.ready = False
        self._labels = labels
        self._log = logging.getLogger("main")

    def onStatus(self, neural_network, is_ready):
        self.ready = is_ready
        self._log.info("network %s", "ready" if is_ready else "not ready")
        # Some missions want the Parrot on-board networks off the NPU they use, for performance.
        if is_ready:
            neural_network.deactivate_native_nns(True)

    def onError(self, neural_network, reason):
        self._log.warning("network error: %s", reason)

    def onInferResult(self, neural_network, outputs, timestamp_ns):
        if not outputs:
            return
        logits = np.asarray(outputs[0].data, dtype=np.float32).flatten()
        exps = np.exp(logits - logits.max())
        prob = exps / exps.sum()
        for rank, idx in enumerate(prob.argsort()[::-1][:5]):
            name = self._labels[idx] if idx < len(self._labels) else "unknown"
            self._log.info("Top%d: %s (%.1f%%)", rank + 1, name, prob[idx] * 100.0)

    def onInferError(self, neural_network, error, timestamp_ns):
        self._log.warning("inference failed: %d", error)


def load_labels():
    try:
        with open(LABELS_FILE) as f:
            return [line.rstrip("\n\r") for line in f]
    except OSError:
        logging.getLogger("main").warning("cannot open %s", LABELS_FILE)
        return []


def preprocess(sample):
    """NV12 sample -> normalized RGB float32 tensor (INPUT_DIM, INPUT_DIM, 3).

    Samples the center square crop straight onto the INPUT_DIM grid (bilinear
    luma, nearest chroma), so the cost is O(INPUT_DIM^2) instead of converting
    and resizing the whole frame -- same approach as the C sample.
    """
    w, h = sample.width, sample.height
    if len(sample.planes) < 2 or w == 0 or h == 0:
        return None

    data = sample.data
    p0, p1 = sample.planes[0], sample.planes[1]
    y = data[p0["offset"] : p0["offset"] + p0["stride"] * h].reshape(h, p0["stride"])
    uv = data[p1["offset"] : p1["offset"] + p1["stride"] * (h // 2)].reshape(
        h // 2, p1["stride"]
    )

    # Bilinear sample coordinates over the center square crop.
    crop = min(w, h)
    x0, y0 = (w - crop) // 2, (h - crop) // 2
    gy = (np.arange(INPUT_DIM, dtype=np.float32) + 0.5) * crop / INPUT_DIM - 0.5 + y0
    gx = (np.arange(INPUT_DIM, dtype=np.float32) + 0.5) * crop / INPUT_DIM - 0.5 + x0
    iy = np.clip(np.floor(gy).astype(int), 0, h - 1)
    ix = np.clip(np.floor(gx).astype(int), 0, w - 1)
    iy2 = np.clip(iy + 1, 0, h - 1)
    ix2 = np.clip(ix + 1, 0, w - 1)
    wy = (gy - np.floor(gy))[:, None]
    wx = (gx - np.floor(gx))[None, :]

    # Luma: bilinear over the 4 neighbours of each grid point (cast only 224x224).
    a = y[np.ix_(iy, ix)].astype(np.float32)
    b = y[np.ix_(iy, ix2)].astype(np.float32)
    c = y[np.ix_(iy2, ix)].astype(np.float32)
    d = y[np.ix_(iy2, ix2)].astype(np.float32)
    Y = (a + (b - a) * wx) * (1.0 - wy) + (c + (d - c) * wx) * wy

    # Chroma: nearest sample (U,V interleaved, 2x2-subsampled).
    cy = np.clip(iy // 2, 0, h // 2 - 1)
    cx = np.clip((ix // 2) * 2, 0, uv.shape[1] - 2)
    U = uv[np.ix_(cy, cx)].astype(np.float32) - 128.0
    V = uv[np.ix_(cy, cx + 1)].astype(np.float32) - 128.0

    r = Y + 1.402 * V
    g = Y - 0.344136 * U - 0.714136 * V
    bl = Y + 1.772 * U
    rgb = np.clip(np.stack([r, g, bl], axis=-1), 0.0, 255.0) / 255.0
    return np.ascontiguousarray(((rgb - _MEAN) / _STD).astype(np.float32))


def classify_loop(nn, source, listener, attrs, n_classes, stop):
    """Worker: pull the freshest frame, preprocess off the loop, fire infer."""
    log = logging.getLogger("main")
    while not stop.is_set():
        sample = source.wait_sample(1000)
        if sample is None:
            continue  # timeout: the source reconnects on its own
        if not listener.ready:
            continue  # network not ready yet
        tensor = preprocess(sample)
        if tensor is None:
            log.error("preprocessing failed")
            continue
        # Non-blocking: infer copies the input and returns immediately; the
        # result arrives via Listener.onInferResult. -EBUSY means a previous
        # inference is still running, so we drop this frame (one at a time).
        rc = nn.infer([Tensor(attrs, tensor)], n_classes * 4, sample.timestamp_ns)
        if rc < 0 and rc != -errno.EBUSY:
            log.warning("infer failed: %d", rc)


async def service_main():
    ulog.setup_logging(PROCESS_NAME)
    log = logging.getLogger("main")
    log.info("Starting image_classifier_py mission")

    labels = load_labels()
    attrs = BlobAttributes("input", "input", 1, INPUT_DIM, INPUT_DIM, 3)
    n_classes = len(labels) or 1000

    listener = Listener(labels)
    settings = Settings()
    settings.net_id = "airsdk@ResNet50"
    settings.model_file = MODEL_FILE
    settings.output_layers = ["/fc/Gemm"]
    # settings.diag_log_folder = BASE  # debug only: adds per-inference I/O overhead
    nn = NeuralNetwork(settings, listener, INFER_TIMEOUT_MS)
    source = make_vipc_source(STREAM_NAME)

    stop = threading.Event()
    worker = threading.Thread(
        target=classify_loop, args=(nn, source, listener, attrs, n_classes, stop)
    )
    worker.start()

    done = asyncio.Event()
    aloop = asyncio.get_running_loop()
    aloop.add_signal_handler(signal.SIGTERM, done.set)
    aloop.add_signal_handler(signal.SIGINT, done.set)
    await done.wait()

    # Re-enable native NNs on shutdown, or the drone stays degraded.
    nn.deactivate_native_nns(False)

    log.info("Stopping image_classifier_py mission")

    stop.set()
    worker.join()


def main():
    asyncio.run(service_main())
