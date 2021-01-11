import logging

from fsup.missions.default.takeoff.stage import TAKEOFF_STAGE as DEF_TAKEOFF_STAGE
from fsup.missions.default.hovering.stage import HOVERING_STAGE as DEF_HOVERING_STAGE
from fsup.missions.default.landing.stage import LANDING_STAGE as DEF_LANDING_STAGE
from fsup.missions.default.critical.stage import CRITICAL_STAGE as DEF_CRITICAL_STAGE
from fsup.missions.default.mission import TRANSITIONS as DEF_TRANSITIONS

# messages exchanged with mission UI code
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
        self.hello_svc = self.env.make_airsdk_service_pair(HelloMessages)

        # Directly create an instance of Service instead of a
        # ServicePair because HelloGdncGroundModeMessages only defines an Event
        # class (there is no Command message).
        self.gdnc_grd_svc = \
            self.mc.make_service(HelloGdncGroundModeMessages.Event)

        # Create Computer Vision service messages channel
        self.cv_channel = \
            self.mc.start_client_channel('unix:/tmp/hello-cv-service')

    def on_unload(self):
        ##################################
        # Messages / communication setup #
        ##################################
        self.cv_channel = None
        self.gdnc_grd_svc = None
        self.hello_svc = None

    def on_activate(self):
        ##################################
        # Messages / communication setup #
        ##################################
        # All messages are forwarded to the supervisor's message
        # center, which feeds the state machine and makes transitions
        # based on those messages possible
        self.hello_svc.attach()
        # Attach an event handler for service self.gdnc_grd_svc to the
        # guidance channel.
        self.mc.attach_handler(self.mc.gdnc_channel,
                               self.gdnc_grd_svc)
        # Forward all service events to the notify method of the
        # supervisor's message center. This is necessary to make sure
        # messages from the custom guidance mode are seen by the
        # message center and used as transition events in the
        # supervisor's state-machine.
        self.gdnc_grd_svc.start_forwarding(self.mc.notify)

        # Attach Computer Vision service messages
        self.cv_svc = self.mc.attach_client_service_pair( \
            self.cv_channel, HelloCvServiceMessages, True)

        # For debugging, also observe messages manually.
        self._dbg_observer = self.hello_svc.observe({
            events.Service.MESSAGE: self._on_msg_evt
        })

        ############
        # Commands #
        ############
        # Start Computer Vision service processing
        self.cv_svc.cmd.sender.set_process(True)

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
        self.cv_svc.cmd.sender.set_process(False)

        ##################################
        # Messages / communication setup #
        ##################################
        self.mc.detach_client_service_pair(self.cv_svc)
        self.cv_svc = None

        self.hello_svc.detach()
        self._dbg_observer.unobserve()
        delattr(self, '_dbg_observer')
        self.mc.detach_handler(self.mc.gdnc_channel, self.gdnc_grd_svc)
        # while it is not strictly necessary here (since no event will
        # be recevied after detaching from the channel), also stop
        # forwarding events.
        self.gdnc_grd_svc.stop_forwarding(self.mc.notify)

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
            # "say/hold" messages from the mission UI alternate between "say" and
            # "idle" states in the ground stage.
            [self.hello_svc.cmd.idx.say, 'ground.idle', 'ground.say'],
            [self.hello_svc.cmd.idx.hold, 'ground.say', 'ground.idle'],
        ]

        return TRANSITIONS + DEF_TRANSITIONS
