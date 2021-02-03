import logging

from fsup.services.message_center import ServicePair

from fsup.missions.default.takeoff.stage import TAKEOFF_STAGE as DEF_TAKEOFF_STAGE
from fsup.missions.default.hovering.stage import HOVERING_STAGE as DEF_HOVERING_STAGE
from fsup.missions.default.landing.stage import LANDING_STAGE as DEF_LANDING_STAGE
from fsup.missions.default.critical.stage import CRITICAL_STAGE as DEF_CRITICAL_STAGE
from fsup.missions.default.mission import TRANSITIONS as DEF_TRANSITIONS

# messages exchanged with mission UI
import parrot.missions.samples.hello.airsdk.messages_pb2 as HelloMessages

# messages exchanged with the guidance ground mode
import samples.hello.guidance.messages_pb2 as HelloGdncGroundModeMessages

# messages exchanged with the cv service
import samples.hello.cv_service.messages_pb2 as HelloCvServiceMessages

# events that are not expressed as protobuf messages
import fsup.services.events as events

UID = "com.parrot.missions.samples.hello"

from .ground.stage import GROUND_STAGE
from .flying.stage import FLYING_STAGE

class Mission(object):
    def __init__(self, mission_environment):
        self.env = mission_environment
        self.mc = self.env.mc
        self.log = logging.getLogger()

    def on_load(self):
        ##################################
        # Messages / communication setup #
        ##################################
        # Create a ServicePair instance from the Python module that is
        # generated from the source protobuf file. The module is
        # expected to define two classes, Command and Event. The
        # associated Service instances are stored in the respective
        # fields "cmd" and "evt" of the ServicePair object. The
        # mission_environment object and its channel assumes that the
        # mission is a server: as such it sends events and receive
        # commands. That's why the attach() and detach() methods of
        # the ServicePair object are setup to automatically attach a
        # message handler for the cmd Service, and a message sender
        # for the evt Service.
        self.ext_ui_msgs = self.env.make_airsdk_service_pair(HelloMessages)

        # Create Guidance ground mode messages
        self.gdnc_grd_mode_msgs = ServicePair(self.mc,
            HelloGdncGroundModeMessages, self.mc.gdnc_channel)

        # Create Computer Vision service messages
        self.cv_service_msgs_channel = \
            self.mc.start_client_channel('unix:/tmp/hello-cv-service')
        self.cv_service_msgs = ServicePair(self.mc,
            HelloCvServiceMessages, self.cv_service_msgs_channel)

    def on_unload(self):
        ##################################
        # Messages / communication setup #
        ##################################
        self.cv_service_msgs = None
        self.cv_service_msgs_channel = None
        self.gdnc_grd_mode_msgs = None
        self.ext_ui_msgs = None

    def on_activate(self):
        ##################################
        # Messages / communication setup #
        ##################################

        # All messages are forwarded to the supervisor's message
        # center, which feeds the state machine and makes transitions
        # based on those messages possible

        # Attach mission UI messages
        self.ext_ui_msgs.attach()

        # Attach Guidance ground mode messages
        self.gdnc_grd_mode_msgs = self.mc.attach_client_service_pair(
            self.mc.gdnc_channel, HelloGdncGroundModeMessages, True)

        # Attach Computer Vision service messages
        self.cv_service_msgs = self.mc.attach_client_service_pair(
            self.cv_service_msgs_channel, HelloCvServiceMessages, True)

        # For debugging, also observe UI messages manually using an observer
        self._dbg_observer = self.ext_ui_msgs.observe({
            events.Service.MESSAGE: self._on_msg_evt
        })

        ############
        # Commands #
        ############
        # Start Computer Vision service processing
        self.cv_service_msgs.cmd.sender.set_process(True)

    def _on_msg_evt(self, event, service, message):
        # It is recommended that log functions are only called with a
        # format string and additional arguments, instead of a string
        # that is already interpolated (e.g. with % or .format()),
        # especially in a context where logs happen frequently. This
        # is due to the fact that string interpolation need only be
        # done when the record is actually logged (for example, by
        # default log.debug(...) is not logged).
        self.log.debug("%s: message %s", UID, message)

    def on_deactivate(self):
        ############
        # Commands #
        ############
        # Stop Computer Vision service processing
        self.cv_service_msgs.cmd.sender.set_process(False)

        ##################################
        # Messages / communication setup #
        ##################################
        # For debugging, unobserve
        self._dbg_observer.unobserve()
        delattr(self, '_dbg_observer')

        # Detach Guidance ground mode messages
        self.mc.detach_client_service_pair(self.cv_service_msgs)

        # Detach Computer Vision service messages
        self.mc.detach_client_service_pair(self.gdnc_grd_mode_msgs)

        # Detach mission UI messages
        self.ext_ui_msgs.detach()

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
        # Service objects hold an object named "idx" (index), which
        # itself contains as many fields as there are message types
        # for in the associated protobuf message (one for each oneof
        # value).

        # Each field is a string that represents an event in the
        # state-machine; it means "this particular message from that
        # service was received". In the state-machine, an event is
        # used to trigger a transition from a source state to a target
        # state.

        TRANSITIONS = [
            # "say/hold" messages from the mission UI alternate between "say"
            # and "idle" states in the ground stage.
            [self.ext_ui_msgs.cmd.idx.say, 'ground.idle', 'ground.say'],
            [self.ext_ui_msgs.cmd.idx.hold, 'ground.say', 'ground.idle'],
        ]

        return TRANSITIONS + DEF_TRANSITIONS
