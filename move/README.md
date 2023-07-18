###### 'MOVE' MISSION TESTING ######
The drone won't move and execute its pre-calculated trajectory before takingoff.

To order the drone to take off, an Olympe script is needed.

### Documentation ###
To install Olympe, follow the set up guide.
(https://developer.parrot.com/docs/olympe/installation.html)
To code your own script, follow the instructions of the documentation.
(https://developer.parrot.com/docs/olympe/userguide.html)

### Helping script ###

Install Olympe with this command:

```bash
$ pip3 install parrot-olympe
```

and execute the script take_off.py to send a takeoff command to the drone:

```bash
$ python3 take_off.py
```

The drone takes off and executes the 'move' trajectory

### WATCH OUT ###
This mission aims to move the drone along a pre-defined trajectory. Test the mission on a simulator before any test on a physical drone.
/!\ TEST THE MISSION ON A PHYSICAL DRONE ONLY IF YOU ARE CAPABLE TO REGAIN CONTROL OF THE DRONE USING THE REMOTE CONTROLLER.
/!\ MAKE SURE THERE IS NO PERSON OR OBSTACLES AROUND THE DRONE. Lower the distance of the trajectory if necessary.