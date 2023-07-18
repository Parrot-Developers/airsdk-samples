# Tracking mission example

**This mission only works on a physical drone.**

Two tracking modes are available (mTrackingMode is set when the StateMachine
class is initialized):

- TRACKING_MODE_AUTO

    Auto mode enabled. Run the tracking algorithm with box proposals. This mode
    automatically selects a target from the proposed boxes. If the target is
    lost for more than 5 seconds, another suggestion box is selected.

- TRACKING_MODE_MANUAL

    Manual mode enabled. Run the tracking algorithm without box proposals. The
    user is responsible for interactively drawing a square over the live video
    stream to define the target. If the target is lost for more than 5
    seconds, the user should select a target again.

# Run the Tracking mission sample

The mission is built and installed with:

```bash
$ airsdk build
$ airsdk install --default
```

> **_NOTE_**: For more information on Air SDK, please refer to the
[Air SDK documentation](https://developer.parrot.com/docs/airsdk/general/overview.html).

You need to install (or reinstall) Olympe with the `rendering` extra
dependencies:

```bash
$ pip3 install parrot-olympe[rendering]
```

Switch on the SkyController and connect it to the computer using a USB cable.
Once the SkyController is connected to the drone, run the `tracking_hud.py`
script:

```bash
$ python3 tracking_hud.py
```

A window appears, showing the drone's front camera feed, with green boxes
representing the proposed boxes and a blue box representing the current
tracked object.

> **_NOTE_**: For more information on Olympe, please refer to the
[Olympe documentation](https://developer.parrot.com/docs/olympe/overview.html).
