import logging

from fsup.missions.default.takeoff.stage import TAKEOFF_STAGE as DEF_TAKEOFF_STAGE
from fsup.missions.default.hovering.stage import HOVERING_STAGE as DEF_HOVERING_STAGE
from fsup.missions.default.landing.stage import LANDING_STAGE as DEF_LANDING_STAGE
from fsup.missions.default.critical.stage import CRITICAL_STAGE as DEF_CRITICAL_STAGE
from fsup.missions.default.mission import TRANSITIONS as DEF_TRANSITIONS

# messages exchanged with mission UI code
import parrot.missions.samples.hello.airsdk.messages_pb2 as HelloMessages

# messages exchanged with the guidance mode
import parrot.missions.samples.hello.guidance.messages_pb2 as HelloGroundMode

# events that are not expressed as protobuf messages
import fsup.services.events as events

UID = "com.parrot.missions.samples.hello"
GUIDANCE_MODE = UID + '.ground'

from .ground.stage import GROUND_STAGE
from .flying.stage import FLYING_STAGE

class Mission(object):
    def __init__(self, mission_environment):
        self.env = mission_environment
        self.mc = self.env.mc
        self.log = logging.getLogger()

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
        # for the evt Service (see example of client code in
        # flight_supervisor/tools/)
        self.hello_svc = self.env.make_airsdk_service_pair(HelloMessages)

        # Directly create an instance of Service instead of a
        # ServicePair because HelloGroundMode only defines an Event
        # class (there is no Command message).
        self.gdnc_grd_svc = self.mc.make_service(HelloGroundMode.Event)

    def on_activate(self):
        # It is recommanded that log functions are only called with a
        # format string and additional arguments, instead of a string
        # that is already interpolated (e.g. with % or .format()),
        # especially in a context where logs happen frequently. This
        # is due to the fact that string interpolation need only be
        # done when the record is actually logged (for example, by
        # default log.debug(...) is not logged).
        self.log.info("%s: activated", UID)
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
        # For debugging, also observe messages manually.
        self._dbg_observer = self.hello_svc.observe({
            events.Service.MESSAGE: self._on_msg_evt
        })

    def _on_msg_evt(self, event, service, message):
        self.log.info("%s: message %s", UID, message)

    def on_deactivate(self):
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
            # "say" messages from the user alternate between "say" and
            # "idle" states in the ground stage.
            [self.hello_svc.cmd.idx.say, 'ground.idle', 'ground.say'],
            [self.hello_svc.cmd.idx.say, 'ground.say',  'ground.idle'],

            # Add this transition so that "count" events trigger the
            # step method in the state. "None" target state means
            # "stay in same state" (this has not much impact here,
            # this is more useful when the source state is a list of
            # states).
            [self.gdnc_grd_svc.idx.count, 'ground.say',  None],

        ]

        return TRANSITIONS + DEF_TRANSITIONS
