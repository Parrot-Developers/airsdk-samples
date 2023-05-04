#!/usr/bin/env python3

import olympe

from olympe.messages.ardrone3.Piloting import TakeOff

drone = olympe.Drone('10.202.0.1')
drone.connect()

assert drone(TakeOff()).wait()

drone.disconnect()