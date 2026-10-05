// WebSocket initialization
WebSocketHelper.onMessageReceived = (event) => onRecvMsg(event.data);

// Mirrors the mission's state so each button knows what to ask for next.
let inferenceEnabled = false;
let nativeNnsDeactivated = false;

function onRecvMsg(data) {
    var msg = JSON.parse(data);

    if (msg.hasOwnProperty("classification")) {
        renderPredictions(msg.classification.predictions);
        updateFps(msg.classification.fps);
    }
    if (msg.hasOwnProperty("status")) {
        updateControls(msg.status);
    }
}

// Reflects the mission's real state, not what was optimistically clicked.
function updateControls(status) {
    nativeNnsDeactivated = status.native_nns_deactivated;
    let nnBtn = document.getElementById("nativeNnsBtn");
    nnBtn.textContent = nativeNnsDeactivated ? "Reactivate" : "Deactivate";
    nnBtn.className = "btn btn-sm " + (nativeNnsDeactivated ? "btn-primary" : "btn-secondary");

    inferenceEnabled = status.inference_enabled;
    let btn = document.getElementById("inferenceBtn");
    btn.textContent = inferenceEnabled ? "Stop" : "Start";
    btn.className = "btn btn-sm " + (inferenceEnabled ? "btn-primary" : "btn-secondary");
}

function onToggleNativeNns() {
    WebSocketHelper.sendMessage(JSON.stringify({ cmd: "set_native_nns_deactivated", value: !nativeNnsDeactivated }));
}

function onToggleInference() {
    WebSocketHelper.sendMessage(JSON.stringify({ cmd: "set_inference_enabled", value: !inferenceEnabled }));
}

// fps is computed on the mission side, from completed inferences.
function updateFps(fps) {
    let fpsEl = document.getElementById("fps");
    if (fpsEl && fps !== undefined)
        fpsEl.textContent = fps.toFixed(1) + " fps";
}

// Rebuilt from the top-5 predictions of the latest inference result. Bar
// width is the score itself (0-100%), not relative to the top prediction, so
// a low-confidence Top1 doesn't render as a full bar.
function renderPredictions(predictions) {
    if (!predictions || predictions.length === 0)
        return;

    for (let i = 0; i < 5; ++i) {
        let label = document.getElementById("pred" + i + "-label");
        let score = document.getElementById("pred" + i + "-score");
        let bar = document.getElementById("pred" + i + "-bar");
        if (!label || !score || !bar)
            continue;

        if (i < predictions.length) {
            let pct = predictions[i].score * 100;
            label.textContent = predictions[i].label;
            score.textContent = pct.toFixed(1) + "%";
            bar.style.width = pct + "%";
        } else {
            label.textContent = "";
            score.textContent = "";
            bar.style.width = "0%";
        }
    }
}
