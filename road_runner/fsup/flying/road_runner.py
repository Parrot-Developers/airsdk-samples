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

from fsup.genstate import State, guidance_modes

from ..uid import UID

_LOOK_DOWN_MODE_NAME = UID + ".look_down"
_ROAD_FOLLOWING_MODE_NAME = UID + ".road_following"

@guidance_modes(_LOOK_DOWN_MODE_NAME)
class LookDown(State):
    def enter(self, msg):
        self.set_guidance_mode(_LOOK_DOWN_MODE_NAME)

@guidance_modes(_ROAD_FOLLOWING_MODE_NAME)
class RoadFollowing(State):
    def enter(self, msg):
        self.set_guidance_mode(_ROAD_FOLLOWING_MODE_NAME, None)


ROAD_RUNNER_STATE = {
    "name": "road_runner",
    "initial": "look_down",
    "children": [
        {
            "name": "look_down",
            "class": LookDown,
        },
        {
            "name": "road_following",
            "class": RoadFollowing,
        },
    ],
}
