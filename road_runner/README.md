# Run the Road Runner mission sample

The Sphinx simulator must be installed to test this mission. If the Sphinx
simulator is not installed yet, please follow these [instructions](https://developer.parrot.com/docs/sphinx/installation.html) to install it.

The Road Runner mission sample works with the world
[parrot-ue4-landscape-ecosystem](https://developer.parrot.com/docs/sphinx/available_worlds.html#id8)
(the *main* map) in the Sphinx simulator.

First, install and launch the simulator with a specific drone position using
the `pose` parameter:

```bash
# Install Landscape ecosystem
$ sudo apt install parrot-ue4-landscape-ecosystem

# Launch the simulator with the custom 'pose' parameter
$ sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone"::pose="1255.41273 -2342.71922 268.21695 0 0 1.5708" & parrot-ue4-landscape-ecosystem
```

The `pose` option places the drone directly on the road. To run the mission
correctly, change the simulation weather once the simulator has started
(unnecessary if you run the simulator with the `-quality=low` option):

```bash
$ sphinx-cli param -m world sky/sky preset daylight_verycloudy
```

The mission can be installed with:

```bash
$ airsdk install --default
```

> **_NOTE_**: For more information on Airsdk, please refer to the [Airsdk documentation](https://developer.parrot.com/docs/airsdk/general/overview.html).

After restarting the drone, the Road Runner mission sample is ready to go and
will start once a take-off command is received. To order the drone to take off,
you can use an Olympe script.

First, install Olympe with this command:

```bash
$ pip3 install parrot-olympe
```

and execute the script take_off.py to send a takeoff command to the drone:

```bash
$ python3 take_off.py
```

The drone takes off at a height of 10 metres, points its camera
to the ground and starts moving forward. Simply send a take-off/landing command
or any movement command (up/down, left/right, forward/back or turn left/turn
right) to stop the mission. The drone resumes its nominal behaviour in its
hovering state.

> **_NOTE_**: For more information on Olympe, please refer to the [Olympe documentation](https://developer.parrot.com/docs/olympe/overview.html).
