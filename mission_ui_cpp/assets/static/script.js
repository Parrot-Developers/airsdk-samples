// WebSocket initialization
WebSocketHelper.onMessageReceived = (event) => onRecvMsg(event.data);

// When the WebView receives a message, we want to display information about it below the button
let messagesArray = [];
function onRecvMsg(data) {
    var msg = JSON.parse(data);

    if (msg.hasOwnProperty("sendMsg")) {
        let button = document.getElementById("sendMsg");
        let message = document.createElement('p');
        message.textContent = msg.sendMsg;
        message.style.textAlign = "center";
        message.style.margin = "10px 0";

        messagesArray.push(message);
        button.parentNode.insertBefore(message, button.nextSibling);

        // Display the message below the button for a specified duration (5 seconds here)
        setTimeout(function () {
            message.remove();
            messagesArray = messagesArray.filter(item => item !== message);
        }, 5000);
    } else if (msg.hasOwnProperty("telemetry")) {
        // Side panel and settings.html share this script but show different
        // things; only update the elements that are actually on this page.
        if (document.getElementById("roll")) {
            updateAttitudeNumbers(msg.telemetry.attitude);
            updateFlightNumbers(
                msg.telemetry.altitude_agl,
                msg.telemetry.ground_speed,
                msg.telemetry.vertical_speed
            );
        } else if (document.getElementById("groundSpeedGauge")) {
            updateSixPack(msg.telemetry);
        }
    } else if (msg.hasOwnProperty("pingPong")) {
        let pingPongBtn = document.getElementById("pingPong")
        pingPongBtn.textContent = msg.pingPong
    } else if (msg.hasOwnProperty("triggerWebviews")) {
        alert(document.title + " received the trigger")
    } else {
        // Handle other messages here
    }
}

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

// --- Side panel: numeric glance info only ---
// Raw gyro/acc aren't shown here: they're not meaningful at a glance, and
// the settings six-pack already surfaces what they drive (VSI, turn
// coordinator) in a readable form.

function updateAttitudeNumbers(attitude) {
    if (!attitude) return;
    document.getElementById("roll").textContent = (attitude.roll * 180 / Math.PI).toFixed(1) + "°";
    document.getElementById("pitch").textContent = (attitude.pitch * 180 / Math.PI).toFixed(1) + "°";
    document.getElementById("yaw").textContent = (attitude.yaw * 180 / Math.PI).toFixed(1) + "°";
}

function updateFlightNumbers(altitudeAgl, groundSpeed, verticalSpeed) {
    document.getElementById("altitude").textContent = altitudeAgl.toFixed(2) + " m";
    document.getElementById("groundSpeed").textContent = groundSpeed.toFixed(1) + " m/s";
    document.getElementById("verticalSpeed").textContent = verticalSpeed.toFixed(2) + " m/s";
}

// --- Settings view: full six-pack instrument panel ---
// Sign/axis choices below are a first guess for "looks right on screen";
// tune against real hardware if needed.

const GAUGE_SWEEP_DEG = 240; // -120deg to +120deg from the top (12 o'clock)

// Builds a dial's ticks/needle/hub/readout once; updateGauge() then only
// rotates the needle and updates the readout text.
function buildGauge(gaugeId, tickCount) {
    let gauge = document.getElementById(gaugeId);
    if (!gauge) return;

    for (let i = 0; i <= tickCount; i++) {
        let angle = -GAUGE_SWEEP_DEG / 2 + (GAUGE_SWEEP_DEG * i) / tickCount;
        let tick = document.createElement("div");
        tick.className = "gauge-tick";
        tick.style.transform = `rotate(${angle}deg) translateY(-45px)`;
        gauge.appendChild(tick);
    }

    let needle = document.createElement("div");
    needle.className = "gauge-needle";
    needle.id = gaugeId + "Needle";
    gauge.appendChild(needle);

    let hub = document.createElement("div");
    hub.className = "gauge-hub";
    gauge.appendChild(hub);

    let value = document.createElement("span");
    value.className = "gauge-value";
    value.id = gaugeId + "Value";
    gauge.appendChild(value);
}

function updateGauge(gaugeId, value, min, max, unit) {
    let ratio = (clamp(value, min, max) - min) / (max - min);
    let angle = -GAUGE_SWEEP_DEG / 2 + GAUGE_SWEEP_DEG * ratio;
    document.getElementById(gaugeId + "Needle").style.transform = `rotate(${angle}deg)`;
    document.getElementById(gaugeId + "Value").textContent = value.toFixed(1) + " " + unit;
}

// Roll ticks (fixed, near the top) and pitch ladder (on the moving ball) for
// the horizon; built once, same "rotate + translateY" placement trick as
// buildGauge()'s ticks.
const HORIZON_ROLL_TICKS_DEG = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60];
const HORIZON_PITCH_LADDER_DEG = [-30, -20, -10, 10, 20, 30]; // 0 is the horizon line itself
const HORIZON_PX_PER_DEG = 1.2; // matches updateHorizon()'s pitch scale

function buildHorizon() {
    let ball = document.getElementById("horizonBall");
    let rollTicks = document.getElementById("horizonRollTicks");
    if (!ball || !rollTicks) return;

    HORIZON_PITCH_LADDER_DEG.forEach((deg) => {
        let line = document.createElement("div");
        line.className = "horizon-ladder";
        line.style.top = `calc(50% + ${deg * HORIZON_PX_PER_DEG}px)`;
        let label = document.createElement("span");
        label.textContent = Math.abs(deg);
        line.appendChild(label);
        ball.appendChild(line);
    });

    HORIZON_ROLL_TICKS_DEG.forEach((deg) => {
        let tick = document.createElement("div");
        let isMajor = deg === 0 || Math.abs(deg) === 30 || Math.abs(deg) === 60;
        tick.className = isMajor ? "horizon-roll-tick horizon-roll-tick-major" : "horizon-roll-tick";
        tick.style.transform = `rotate(${deg}deg) translateY(-46px)`;
        rollTicks.appendChild(tick);
    });
}

// Heading indicator card: a tick every 5deg, a label every 30deg (the
// aviation convention drops the trailing zero: "3" is 030, "12" is 120...).
const HEADING_MAJOR_LABELS = ["N", "3", "6", "E", "12", "15", "S", "21", "24", "W", "30", "33"];

function buildCompass() {
    let card = document.getElementById("compassCard");
    if (!card) return;

    for (let deg = 0; deg < 360; deg += 5) {
        let tick = document.createElement("div");
        tick.className = deg % 30 === 0 ? "compass-tick compass-tick-major" : "compass-tick";
        tick.style.transform = `rotate(${deg}deg) translateY(-44px)`;
        card.appendChild(tick);
    }

    HEADING_MAJOR_LABELS.forEach((label, i) => {
        let anchor = document.createElement("div");
        anchor.className = "compass-label-anchor";
        anchor.style.transform = `rotate(${i * 30}deg) translateY(-34px)`;
        let text = document.createElement("span");
        let isCardinal = /[NESW]/.test(label);
        text.className = isCardinal ? "compass-label compass-label-cardinal" : "compass-label";
        text.textContent = label;
        anchor.appendChild(text);
        card.appendChild(anchor);
    });
}

if (document.getElementById("groundSpeedGauge")) {
    buildGauge("groundSpeedGauge", 5);
    buildGauge("altitudeGauge", 5);
    buildGauge("vsiGauge", 6);
    buildHorizon();
    buildCompass();
}

function updateSixPack(telemetry) {
    if (!telemetry.attitude) return;
    let rollDeg = telemetry.attitude.roll * 180 / Math.PI;
    let pitchDeg = telemetry.attitude.pitch * 180 / Math.PI;
    let yawDeg = telemetry.attitude.yaw * 180 / Math.PI;
    let yawRateDeg = telemetry.sensors_imu.gyro.z * 180 / Math.PI;

    updateHorizon(rollDeg, pitchDeg);
    updateCompass(yawDeg);
    updateGauge("groundSpeedGauge", telemetry.ground_speed, 0, 15, "m/s");
    updateGauge("altitudeGauge", telemetry.altitude_agl, 0, 10, "m");
    updateGauge("vsiGauge", telemetry.vertical_speed, -3, 3, "m/s");
    updateTurnCoordinator(yawRateDeg, telemetry.sensors_imu.acc.y);
}

// Turn coordinator: plane banks with rate of turn (yaw rate, not attitude),
// ball shifts sideways with lateral acceleration (slip/skid).
function updateTurnCoordinator(yawRateDeg, lateralAccel) {
    let bank = clamp(yawRateDeg, -30, 30);
    // ➤ points east by default, not up: the -90deg offset makes it point
    // up (wings level) at bank=0, same reference as the compass' plane.
    document.getElementById("tcPlane").style.transform =
        `translate(-50%, -50%) rotate(${bank - 90}deg)`;

    let ballOffsetPx = clamp(lateralAccel, -4, 4) / 4 * 25;
    document.getElementById("tcBall").style.left = `calc(50% + ${ballOffsetPx}px)`;
}

// Artificial horizon: ball rotates with roll, slides with pitch.
function updateHorizon(rollDeg, pitchDeg) {
    let pitchPx = clamp(pitchDeg, -45, 45) * 1.2;
    document.getElementById("horizonBall").style.transform =
        `translateY(${pitchPx}px) rotate(${-rollDeg}deg)`;
}

// Heading indicator: the graduated card rotates with yaw (opposite sign, so
// the heading currently at the top lubber line increases as yaw increases);
// the lubber line itself is fixed in CSS, nothing to update here for it.
function updateCompass(yawDeg) {
    document.getElementById("compassCard").style.transform = `rotate(${-yawDeg}deg)`;
}

// Send the message to the drone with WebSocket
function sendMsg(msg) {
    console.log('Send: ' + JSON.stringify(msg));
    WebSocketHelper.sendMessage(JSON.stringify(msg));
}

function onClickSendMsg() {
    var message = document.getElementById("messageInput").value;

    if (message.trim() !== "") {
        var msg = { cmd: "sendMsg", value: message };
        sendMsg(msg);
        document.getElementById("messageInput").value = "";
        document.getElementById("messageInput").blur();
    } else {
        alert("Message cannot be empty!");
    }
}

// Add event listener to send the message when ENTER is pressed.
// Only settings.html has this field; side panel and other pages don't.
let messageInputEl = document.getElementById("messageInput");
if (messageInputEl) {
    messageInputEl.addEventListener("keypress", function (event) {
        if (event.key === "Enter") {
            onClickSendMsg(); // Call the function to send the message
        }
    });
}

function onClickPingPong() {
    var msg = { cmd: "pingPong", value: document.getElementById("pingPong").textContent };
    sendMsg(msg);
}

function onClickTriggerWebviews() {
    var msg = { cmd: "triggerWebviews" };
    sendMsg(msg);
}
