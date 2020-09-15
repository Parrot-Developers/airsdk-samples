
import logging

from fsup.missions.default.takeoff.stage import TAKEOFF_STAGE as DEF_TAKEOFF_STAGE
from fsup.missions.default.hovering.stage import HOVERING_STAGE as DEF_HOVERING_STAGE
from fsup.missions.default.landing.stage import LANDING_STAGE as DEF_LANDING_STAGE
from fsup.missions.default.critical.stage import CRITICAL_STAGE as DEF_CRITICAL_STAGE
from fsup.missions.default.mission import TRANSITIONS as DEF_TRANSITIONS

UID = "com.parrot.missions.samples.hello"
from .ground.stage import GROUND_STAGE
from .flying.stage import FLYING_STAGE

class Mission(object):
    def __init__(self, mission_environment):
        self.env = mission_environment
        self.mc = self.env.mc
        self.log = logging.getLogger()

    def on_activate(self):
        pass

    def on_deactivate(self):
        pass

    def states(self):
        return [
            GROUND_STAGE,
            DEF_TAKEOFF_STAGE,
            DEF_HOVERING_STAGE,
            FLYING_STAGE,
            DEF_LANDING_STAGE,
            DEF_CRITICAL_STAGE,
        ]

    def transitions(self):
        return TRANSITIONS + DEF_TRANSITIONS


hext = "ComParrotMissionsSamplesHello."

TRANSITIONS = [
    [hext+'say',                        'ground.idle',                          'ground.say'],
    [hext+'say',                        'ground.say',                           'ground.idle'],
]
