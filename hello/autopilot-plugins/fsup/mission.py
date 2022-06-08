from msghub_utils import msg_id
from fsup.genmission import AbstractMission
from fsup.missions.default.takeoff.stage import (
    TAKEOFF_STAGE as DEF_TAKEOFF_STAGE,
)
from fsup.missions.default.hovering.stage import (
    HOVERING_STAGE as DEF_HOVERING_STAGE,
)
from fsup.missions.default.landing.stage import (
    LANDING_STAGE as DEF_LANDING_STAGE,
)
from fsup.missions.default.critical.stage import (
    CRITICAL_STAGE as DEF_CRITICAL_STAGE,
)
from fsup.missions.default.mission import TRANSITIONS as DEF_TRANSITIONS

# messages exchanged with mission UI
import parrot.missions.samples.hello.airsdk.messages_pb2 as hello_msgs

# messages exchanged with the guidance ground mode
import samples.hello.guidance.messages_pb2 as hello_gdnc_mode_msgs

# messages exchanged with the cv service
import samples.hello.cv_service.messages_pb2 as hello_cv_service_msgs

# events that are not expressed as protobuf messages
import fsup.services.events as events

# messages exchanged with the drone controller
import drone_controller.drone_controller_pb2 as dctl_msgs
import colibrylite.motion_state_pb2 as cbry_motion_state

UID = "com.parrot.missions.samples.hello"

from .ground.stage import GROUND_STAGE  # noqa: E402
from .flying.stage import FLYING_STAGE  # noqa: E402


class Mission(AbstractMission):
    def __init__(self, env):
        super().__init__(env)
        self.ext_ui_msgs = None
        self.cv_service_msgs_channel = None
        self.cv_service_msgs = None
        self.gdnc_grd_mode_msgs = None
        self.observer = None
        self.dbg_observer = None

    def on_load(self):
        ##################################
        # Messages / communication setup #
        ##################################
        # The airsdk service assumes that the mission is a server: as such it
        # sends events and receive commands.
        self.ext_ui_msgs = self.env.make_airsdk_service_pair(hello_msgs)

    def on_unload(self):
        ####################################
        # Messages / communication cleanup #
        ####################################
        self.ext_ui_msgs = None

    def on_activate(self):
        ##################################
        # Messages / communication setup #
        ##################################

        # All messages are forwarded to the supervisor's message
        # center, which feeds the state machine and makes transitions
        # based on those messages possible

        # Attach mission UI messages
        self.ext_ui_msgs.attach(self.env.airsdk_channel, True)

        # Attach Guidance ground mode messages
        self.gdnc_grd_mode_msgs = self.mc.attach_client_service_pair(
            self.mc.gdnc_channel, hello_gdnc_mode_msgs, True
        )

        # Create Computer Vision service channel
        self.cv_service_msgs_channel = self.mc.start_client_channel(
            "unix:/tmp/hello-cv-service"
        )

        # Attach Computer Vision service messages
        self.cv_service_msgs = self.mc.attach_client_service_pair(
            self.cv_service_msgs_channel, hello_cv_service_msgs, True
        )

        # For forwarding, observe messages using an observer
        self.observer = self.mc.observe(
            {
                events.Channel.CONNECTED: lambda _, c: self._on_connected(c),
                msg_id(
                    hello_cv_service_msgs.Event, "close"
                ): lambda *args: self._send_to_ui_stereo_camera_close_state(
                    True
                ),
                msg_id(
                    hello_cv_service_msgs.Event, "far"
                ): lambda *args: self._send_to_ui_stereo_camera_close_state(
                    False
                ),
                msg_id(
                    hello_cv_service_msgs.Event, "depth_mean"
                ): lambda _, msg: self._send_to_ui_depth_mean(msg.depth_mean),
                msg_id(
                    dctl_msgs.Event, "motion_state_changed"
                ): lambda _, msg: self._send_to_ui_drone_motion_state(
                    msg.motion_state_changed == cbry_motion_state.MOVING
                ),
            }
        )

        # For debugging, also observe UI messages manually using an observer
        self.dbg_observer = self.ext_ui_msgs.cmd.observe(
            {events.Service.MESSAGE: self._on_ui_msg_cmd}
        )

        ############
        # Commands #
        ############
        # Start Computer Vision service processing
        self.cv_service_msgs.cmd.sender.processing_start()

    def _on_connected(self, channel):
        if channel == self.env.airsdk_channel:
            self.log.info("connected to airsdk channel")
        elif channel == self.cv_service_msgs_channel:
            self.log.info("connected to cv service channel")

    def _send_to_ui_stereo_camera_close_state(self, state):
        self.ext_ui_msgs.evt.sender.stereo_close(state)

    def _send_to_ui_drone_motion_state(self, state):
        self.ext_ui_msgs.evt.sender.drone_moving(state)

    def _send_to_ui_depth_mean(self, depth_mean):
        self.ext_ui_msgs.evt.sender.depth_mean(depth_mean)

    def _on_ui_msg_cmd(self, event, message):
        # It is recommended that log functions are only called with a
        # format string and additional arguments, instead of a string
        # that is already interpolated (e.g. with % or .format()),
        # especially in a context where logs happen frequently. This
        # is due to the fact that string interpolation need only be
        # done when the record is actually logged (for example, by
        # default log.debug(...) is not logged).
        self.log.debug("%s: UI command message %s", UID, message)

    def on_deactivate(self):
        ############
        # Commands #
        ############
        # Stop Computer Vision service processing
        self.cv_service_msgs.cmd.sender.processing_stop()

        ####################################
        # Messages / communication cleanup #
        ####################################
        # For forwarding, unobserve
        self.observer.unobserve()
        self.observer = None

        # For debugging, unobserve
        self.dbg_observer.unobserve()
        self.dbg_observer = None

        # Detach Computer Vision service messages
        self.mc.detach_client_service_pair(self.cv_service_msgs)
        self.cv_service_msgs = None

        # Detach Guidance ground mode messages
        self.mc.detach_client_service_pair(self.gdnc_grd_mode_msgs)
        self.gdnc_grd_mode_msgs = None

        # Stop Computer Vision service channel
        self.mc.stop_channel(self.cv_service_msgs_channel)
        self.cv_service_msgs_channel = None

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
        # Each field is a string that represents an event in the
        # state-machine; it means "this particular message from that
        # component was received". In the state-machine, an event is
        # used to trigger a transition from a source state to a target
        # state.

        TRANSITIONS = [
            # "say/hold" messages from the mission UI alternate between "say"
            # and "idle" states in the ground stage.
            [
                msg_id(hello_msgs.Command, "say"),
                "ground.idle",
                "ground.say",
            ],
            [
                msg_id(hello_msgs.Command, "hold"),
                "ground.say",
                "ground.idle",
            ],
        ]

        return TRANSITIONS + DEF_TRANSITIONS
