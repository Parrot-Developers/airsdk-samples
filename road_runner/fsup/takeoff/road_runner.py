#  Copyright (c) 2023 Parrot Drones SAS
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of the Parrot Company nor the names
#    of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written
#    permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
#  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
#  SUCH DAMAGE.

from fsup.genstate import guidance_modes
from fsup.missions.default.takeoff.normal import (
    Ascent as DefaultAscent,
    AscentManual as DefaultAscentManual,
    Reset as DefaultReset,
    StartingMotors as DefaultStartingMotors,
    WaitMotorRamping as DefaultWaitMotorRamping,
    WaitAscent as DefaultWaitAscent,
    StoppingMotors as DefaultStoppingMotors,
    WaitUntilSteady as DefaultWaitUntilSteady,
)

from fsup.missions.default.uid import UID

from fsup.utils import get_property

import os
import cfgreader

if get_property("ro.model", "None") == "anafi2":  # anafi2
    import colibrylite.estimation_mode_pb2 as ctl_est
    import guidance.ascent_pb2 as gdnc_ascent_msgs
else:  # anafi3
    import drone_controller.estimation_mode_pb2 as ctl_est
    import guidance.local.plugins.ascent_pb2 as gdnc_ascent_msgs

_STATES_TO_REMOVE = ["ascent"]

CONFIG_FILENAME = "etc/services/road_following.cfg"


def _config(field):
    cfg = cfgreader.build_config_start()
    for (root, include) in field:
        cfg = cfgreader.build_config_update(cfg, root, include)
    return cfgreader.build_config_end(cfg)


@guidance_modes(UID + ".ascent")
class Ascent(DefaultAscent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Get configuration values
        field = [
            (
                os.path.join(
                    self.mission.env.get_product_cfg_dir(), CONFIG_FILENAME
                ),
                "droneAltitude",
            ),
        ]

        self.road_runner_ascent = _config(field)

    def enter(self, msg):
        self.gdnc_asc_svc = self.mc.attach_client_service_pair(
            self.mc.gdnc_channel, gdnc_ascent_msgs, forward_events=True
        )
        self.mc.dctl.cmd.sender.set_estimation_mode(ctl_est.TAKEOFF)

        self.set_guidance_mode(
            "com.parrot.missions.default.ascent",
            gdnc_ascent_msgs.Config(
                type=gdnc_ascent_msgs.TYPE_DEFAULT,
                altitude=self.road_runner_ascent.road_following.droneAltitude,
            ),
        )


ROAD_RUNNER_STATE = {
    "name": "normal",
    "initial": "reset",
    "children": [
        {
            "name": "reset",
            "class": DefaultReset,
        },
        {
            "name": "starting_motors",
            "class": DefaultStartingMotors,
        },
        {
            "name": "wait_motor_ramping",
            "class": DefaultWaitMotorRamping,
        },
        {
            "name": "wait_ascent",
            "class": DefaultWaitAscent,
        },
        {
            "name": "ascent",
            "class": Ascent,
        },
        {
            "name": "ascent_manual",
            "class": DefaultAscentManual,
        },
        {
            "name": "stopping_motors",
            "class": DefaultStoppingMotors,
        },
        {
            "name": "wait_until_steady",
            "class": DefaultWaitUntilSteady,
        },
    ],
}
