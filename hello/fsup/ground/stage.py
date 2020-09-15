
from fsup.genstate import (State, guidance_modes)
import colibrylite.estimation_mode_pb2 as cbry_est
import hello.guidance.hello_ground_mode_pb2 as HelloGroundMode
import hello.messages.com_parrot_mission_samples_hello_pb2 as HelloMessages

from fsup.missions.default.ground.stage import GROUND_STAGE as DEF_GROUND_STAGE

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

    def exit(self, msg):
        self.mc.detach_client_service_pair(self.gdnc_grd_svc)
        self.gdnc_grd_svc = None

    def step(self, msg):
        msgname = msg.WhichOneof('id')
        if msgname == "count" and isinstance(msg, HelloGroundMode.Event):
            #TODO: send HelloMessages event count.
            self.log.info('%s', msg)

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
