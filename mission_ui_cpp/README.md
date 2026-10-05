# Mission UI mission

Add a custom UI panel to FreeFlight: a WebSocket server (`appserver.py` +
`main.py`) serves a side panel and a settings page, both driven by live
drone telemetry, and both able to send commands back to the mission.

It ships as **three missions**, one per language: `mission_ui_py`,
`mission_ui_c`, `mission_ui_cpp`. Same UI, same telemetry, same wire
protocol — only the backend implementation differs. This file documents all
three; language-specific notes are called out where they apply.

## What it does

1. **Backend** — a `register_command`/`broadcast` WebSocket server
   (`appserver.py`) publishes its port via the AirSDK `Publisher` so
   FreeFlight can reach it, then a background thread reads telemetry every
   500ms and broadcasts it as JSON to every connected panel.
2. **Side panel** (`ff8-side-panel.html`) — small, always visible during
   flight: numeric glance info only (Attitude, Altitude, Ground speed), plus
   a free-text message box.
3. **Settings page** (`ff8-settings.html`) — opened deliberately, more room:
   the full six-instrument flight panel, plus two demo buttons.

The side panel and settings page share the same `script.js`; each only
updates the elements actually present on its own page (guarded by
`document.getElementById(...)` checks), so telemetry broadcasts don't have
to know which pages are currently open.

## The six-pack instrument panel

`ff8-settings.html` reproduces the classic aviation "six-pack" layout, each
instrument driven by the telemetry it would really use on an aircraft — not
by whatever was easiest to compute:

| Instrument | Driven by | Note |
|---|---|---|
| Ground speed | `linear_velocity_global.{x,y}`, combined as `hypot(vx, vy)` | Ground speed, not airspeed — a drone has no pitot tube |
| Horizon (attitude indicator) | `attitude_euler_angles.{roll,pitch}` | Roll ticks (fixed, top of the dial) and a pitch ladder (±10/20/30°, drawn on the moving ball) make the angles actually readable, not just a vague tilt |
| Altimeter | `altitude_agl` | |
| Turn coordinator | `sensors_imu.gyro.z` (yaw **rate**) for the plane bank; `sensors_imu.acc.y` (lateral acceleration) for the ball | Shows *rate of turn*, not bank angle. A fixed-wing metaphor, not fully accurate for a quadcopter — kept deliberately (see below) |
| Heading indicator | `attitude_euler_angles.yaw` | The graduated **card rotates** (5° ticks, 30°-interval labels with the trailing zero dropped: `3`=030°, `12`=120°...) while a fixed lubber line and aircraft symbol at the top stay put — same as a real directional gyro. It's *not* a compass needle spinning against fixed N/E/S/W labels |
| VSI (vertical speed) | `d(altitude_agl)/dt`, computed in the backend over the fixed 500ms broadcast cadence | Not a telemetry key — there isn't one; derived from two altitude samples |

Ground speed, Altimeter and VSI share one generic radial-gauge component
(`buildGauge()`/`updateGauge()` in `script.js`: dial + needle + tick marks,
built once, updated by rotating the needle) instead of three bespoke
widgets — the difference between them is just a `(min, max, unit)` triple.

**Why keep an instrument that doesn't fully apply?** A turn coordinator
assumes fixed-wing physics — a plane must bank to turn, and "slip/skid"
exists because flying with the wind off-axis is aerodynamically wrong for a
winged aircraft. A quadcopter has neither constraint: yaw and roll are
independent, and moving sideways is normal, not an error. It stayed
anyway, deliberately, so the panel keeps six distinct widgets instead of
five plus a dial that duplicates the others' look; `gyro.z` (yaw rate) is
still real, useful data, only the ball's framing is borrowed and
inaccurate here.

Sign/axis conventions (e.g. `rotate(${-rollDeg}deg)` on the horizon) are a
first guess for "looks right on screen"; they haven't been validated against
real hardware yet.

## Telemetry sections used

* `sensors_imu.gyro[i].{x,y,z}` / `sensors_imu.acc[i].{x,y,z}` (0..9) —
  averaged over the 10 samples, matches the other `telemetry_*` samples.
* `drone_controller.attitude_euler_angles.{roll,pitch,yaw}` — radians,
  latest single sample (not averaged).
* `drone_controller.altitude_agl` — meters.
* `drone_controller.linear_velocity_global.{x,y}` — m/s; registered but
  unused elsewhere in the repo before this mission.

All on the *same* telemetry consumer as the IMU — no extra thread, no extra
polling loop, just more fields registered before the one `regComplete()`.

## Communication patterns (`ff8-settings.html` → mission)

All three demos live under `ff8-settings.html`'s "Toolbox"/"Communication"
blocks — the side panel is numeric glance info only, no interactive
controls. Each demonstrates a different way a panel can talk back to the
mission:

* **PING** — `register_command("pingPong", ...)`: unicast request/response
  via a button. The panel sends its current button text, the mission
  replies (to that panel only) with the opposite (`PING` ⇄ `PONG`), the
  button updates from the reply.
* **Trigger Webviews** — `register_command("triggerWebviews", ...)`:
  broadcast via a button. One panel triggers it, the mission broadcasts to
  *every* connected panel, each shows an `alert()`.
* **Communication** (`sendMsg`) — unicast request/response via a free-text
  `<input>` instead of a button: the panel sends whatever was typed, the
  mission replies (to that panel only) with an acknowledgment echoing it
  back.

## Configuration files

* `mission.yaml` — mission descriptor; depends on `libairsdk`, `libtelemetry`
  and, for `mission_ui_c`/`mission_ui_cpp`, `libwebsockets` + `json`; for
  `mission_ui_py`, the `pip` requirements (`uvicorn`, `websockets`,
  `fastapi==0.115.12`, `pyserial`).
* `assets/static/` — the whole UI: HTML/CSS/JS, identical across all three
  missions (the backend publishes the same wire protocol, so the frontend
  doesn't need to differ).

## Notes and caveats

* **`mission_ui_py` only** — the telemetry thread dispatches onto uvicorn's
  loop via `asyncio.run_coroutine_threadsafe()` instead of a private loop in
  that thread (driving Starlette `WebSocket` objects from a foreign loop is
  undefined behavior, even if it appears to work in casual testing).
  Telemetry start/stop is wired to FastAPI's `startup`/`shutdown` lifecycle
  rather than `signal.signal()`, since uvicorn installs its own SIGINT/
  SIGTERM handlers and would silently overwrite a custom one registered
  earlier.
* **Dead-peer detection differs by language.** `mission_ui_py` tightens
  uvicorn's ping to `ws_ping_interval=5, ws_ping_timeout=5` (~10s vs the
  ~40s default). `mission_ui_c`/`mission_ui_cpp` still use libwebsockets'
  defaults — the equivalent (`info.ws_ping_pong_interval`) doesn't exist on
  the `lws_context_creation_info` struct shipped by the AirSDK toolchain;
  fixing this needs someone with the actual headers to find the right field.
* `broadcast()` is best-effort in all three languages: one client's
  send failure (e.g. a connection that just dropped) is logged and skipped,
  it doesn't abort delivery to the other connected panels.
* `socket-helpers.js` closes its socket (code 1000) on `pagehide` — without
  this, a reloaded/re-opened panel leaves its old connection open server
  side until something eventually times it out, so `broadcast()` (e.g.
  Trigger Webviews) keeps hitting stale sessions in the meantime; observed
  on real hardware as several stray "WebSocket client connected" log lines
  with no matching disconnect for up to a minute.
* Ground speed/VSI ranges (`0-15 m/s`, `±3 m/s`) and the turn coordinator's
  clamp ranges (`±30 deg/s`, `±4 m/s²`) were picked for an Anafi-class
  quadcopter; resize the gauge scales for a faster platform.
