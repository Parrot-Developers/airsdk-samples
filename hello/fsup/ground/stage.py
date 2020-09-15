from fsup.genstate import (State, guidance_modes)
import colibrylite.estimation_mode_pb2 as cbry_est

from fsup.missions.default.ground.stage import GROUND_STAGE as DEF_GROUND_STAGE
import hello.guidance.hello_ground_mode_pb2 as HelloGroundMode

import fsup.services.events as events

from ..mission import UID

_STATES_TO_REMOVE = [
    'idle'
]

@guidance_modes(UID + '_ground')
class Idle(State):
    def enter(self, msg):
        m = self.mc.dctl.cmd.alloc()
        m.set_estimation_mode.mode = cbry_est.MOTORS_STOPPED
        self.mc.send(m)

        m = self.mc.gdnc.cmd.alloc()
        m.set_mode.mode = UID + '_ground'
        m.set_mode.config.Pack(HelloGroundMode.Config(say=False))
        self.mc.send(m)

@guidance_modes(UID + '_ground')
class Say(State):
    def enter(self, msg):
        m = self.mc.dctl.cmd.alloc()
        m.set_estimation_mode.mode = cbry_est.MOTORS_STOPPED
        self.mc.send(m)

        m = self.mc.gdnc.cmd.alloc()
        m.set_mode.mode = UID + '_ground'
        m.set_mode.config.Pack(HelloGroundMode.Config(say=True))
        self.mc.send(m)

    # Since we defined a self-transition on the "count" message, the
    # state machine will call the step method with that kind of
    # message.
    def step(self, msg):
        self.log.warning("step: %s", msg)
        # It is a good practice to check the kind of message received
        # in case there are multiple events that can trigger the step.
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
