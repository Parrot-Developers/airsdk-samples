from fsup.genstate import (State, guidance_modes)
import colibrylite.estimation_mode_pb2 as cbry_est

from fsup.missions.default.ground.stage import GROUND_STAGE as DEF_GROUND_STAGE
import samples.hello.guidance.messages_pb2 as HelloGdncGroundModeMessages

import fsup.services.events as events

from ..mission import UID

_STATES_TO_REMOVE = [
    'idle'
]

_GUIDANCE_MODE = UID + ".ground"

@guidance_modes(_GUIDANCE_MODE)
class Idle(State):
    def enter(self, msg):
        m = self.mc.dctl.cmd.alloc()
        m.set_estimation_mode.mode = cbry_est.MOTORS_STOPPED
        self.mc.send(m)

        m = self.mc.gdnc.cmd.alloc()
        m.set_mode.mode = _GUIDANCE_MODE
        m.set_mode.config.Pack(HelloGdncGroundModeMessages.Config(say=False))
        self.mc.send(m)

@guidance_modes(_GUIDANCE_MODE)
class Say(State):
    def enter(self, msg):
        m = self.mc.dctl.cmd.alloc()
        m.set_estimation_mode.mode = cbry_est.MOTORS_STOPPED
        self.mc.send(m)

        m = self.mc.gdnc.cmd.alloc()
        m.set_mode.mode = _GUIDANCE_MODE
        m.set_mode.config.Pack(HelloGdncGroundModeMessages.Config(say=True))
        self.mc.send(m)

    # State machine will call the state step method with the
    # guidance "count" event.
    def step(self, msg):
        # It is required to check the kind of message received as multiple
        # messages can trigger the step method. Instance can be checked too
        # if messages can have the same name.
        msgname = msg.WhichOneof('id')
        if msgname == "count":
            svc = self.mission.hello_svc.evt
            evt = svc.alloc()
            evt.count = msg.count
            self.log.info('count event: msg=%s evt=%s', msg, evt)
            svc.send(evt)

IDLE_STATE = {
    'name': 'idle',
    'class': Idle,
}

SAY_STATE = {
    'name': 'say',
    'class': Say,
}

GROUND_STAGE = {
    'name': 'ground',
    'class': DEF_GROUND_STAGE['class'],
    'initial': 'say',
    'children': [
        child for child in DEF_GROUND_STAGE['children']
            if not child['name'] in _STATES_TO_REMOVE
    ] + [
        IDLE_STATE,
        SAY_STATE
    ]
}
