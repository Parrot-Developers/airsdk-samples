#!/usr/bin/env python3

import argparse

import olympe

def main():
    parser  = argparse.ArgumentParser()
    parser.add_argument("drone_ip", help="Ip address of drone")
    parser.add_argument("mission_path", help="Path to mission archive file on host")
    options = parser.parse_args()

    drone = olympe.Drone(options.drone_ip)
    with drone.mission.from_path(options.mission_path) as hello:
        from olympe.airsdk.messages.parrot.missions.samples.hello.Command import Say
        from olympe.airsdk.messages.parrot.missions.samples.hello.Event import count

        drone.connect()
        drone(Say()).wait()

        while drone.connected:
            expectation = drone(count(_policy="wait")).wait(_timeout=10000)
            print(expectation.matched_events())

        drone.disconnect()

if __name__ == "__main__":
    main()

