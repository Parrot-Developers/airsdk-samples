from msghub_utils import is_msg
from fsup.genstate import State, guidance_modes
import colibrylite.estimation_mode_pb2 as cbry_est

from fsup.missions.default.ground.stage import GROUND_STAGE as DEF_GROUND_STAGE
import samples.hello.guidance.messages_pb2 as hello_gdnc_mode_msgs

from ..mission import UID

_STATES_TO_REMOVE = ["idle"]

_GROUND_MODE_NAME = UID + ".ground"


@guidance_modes(_GROUND_MODE_NAME)
class Idle(State):
    def enter(self, msg):
        self.mc.dctl.cmd.sender.set_estimation_mode(cbry_est.MOTORS_STOPPED)
        self.set_guidance_mode(
            _GROUND_MODE_NAME, hello_gdnc_mode_msgs.Config(say=False)
        )


@guidance_modes(_GROUND_MODE_NAME)
class Say(State):
    def enter(self, msg):
        self.mc.dctl.cmd.sender.set_estimation_mode(cbry_est.MOTORS_STOPPED)
        self.set_guidance_mode(
            _GROUND_MODE_NAME, hello_gdnc_mode_msgs.Config(say=True)
        )

    # State machine will call the state step method with the guidance ground
    # mode "count" event.
    def step(self, msg):
        # It is required to check the kind of message received as multiple
        # messages can trigger the step method.
        if is_msg(msg, hello_gdnc_mode_msgs.Event, "count"):
            self.log.info("ground mode count event: msg=%s", msg)
            self.mission.ext_ui_msgs.evt.sender.count(msg.count)


IDLE_STATE = {
    "name": "idle",
    "class": Idle,
}

SAY_STATE = {
    "name": "say",
    "class": Say,
}

GROUND_STAGE = {
    "name": "ground",
    "class": DEF_GROUND_STAGE["class"],
    "initial": "say",
    "children": [
        child
        for child in DEF_GROUND_STAGE["children"]
        if not child["name"] in _STATES_TO_REMOVE
    ]
    + [IDLE_STATE, SAY_STATE],
}
