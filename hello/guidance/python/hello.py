import numpy as np

import libpomp
import telemetry

import cam_controller.frame_of_reference_pb2 as cam_for_pb2
import cam_controller.control_mode_pb2 as cam_cm_pb2

import guidance.core as gdnc_core
import guidance.guidance_pb2 as guidance_pb
import hello.guidance.hello_ground_mode_pb2 as ground_mode_pb


class HelloGroundMode(gdnc_core.Mode):
    FCAM_PITCH_ANIMATION_PERIOD_MS = 5000
    FCAM_PITCH_ANIMATION = [
        0.0, -0.2, -0.8, -2.0, -3.8, -6.6, -10.4, -15.5, -22.0, -30.1, -40.0,
        -25.0, -10.0, 4.9, 19.9, 34.9, 49.9, 55.5, 42.0, 28.5, 15.0, 1.5, -11.9,
        -25.4, -26.9, -22.4, -18.0, -13.5, -9.0, -4.4, 0.0
    ]

    def __init__(self, guidance, name):
        super().__init__(guidance, name)
        self.loop = self.guidance.get_loop()
        self.msghub =  self.guidance.get_message_hub()
        self.config = self.guidance.get_config()
        self.front_cam_pitch_index = 0

        subset = ['attitude_euler_angles.yaw',
                  'attitude_euler_angles.pitch',
                  'attitude_euler_angles.roll']
        self.tlm_dctl = telemetry.TlmSection('/dev/shm', "drone_controller",
                                             subset=subset)
        self.timer_cb = \
            libpomp.pomp_timer_cb_t(lambda t,d: self._timer_cb())
        self.timer = \
            libpomp.pomp_timer_new(self.loop, self.timer_cb, None)

        self.say = False

    def shutdown(self):
        self.loop = None
        self.msghub = None
        self.config = None
        self.tlm_dctl = None
        libpomp.pomp_timer_destroy(self.timer)
        self.timer_cb = None
        self.timer = None

    def get_triggers(self):
        return (gdnc_core.Trigger.TIMER, 30, 30)

    def configure(self, msg, disable_oa):
        if not msg.type_url.endswith('/Guidance.HelloGroundMode.Messages.Config'):
            raise ValueError("Ground: unexpected config: %s" % msg.type_url)

        ground_mode_msg = ground_mode_pb.Config()
        msg.Unpack(ground_mode_msg)
        self.say = ground_mode_msg.say

        self.output_config.has_front_cam_config = True
        self.output_config.front_cam_config.yaw.locked = True
        self.output_config.front_cam_config.yaw.filtered = False
        self.output_config.front_cam_config.roll.locked = True
        self.output_config.front_cam_config.roll.filtered = False
        self.output_config.front_cam_config.pitch.locked = True
        self.output_config.front_cam_config.pitch.filtered = False

        if self.say:
            libpomp.pomp_timer_set_periodic(self.timer, \
                HelloGroundMode.FCAM_PITCH_ANIMATION_PERIOD_MS, \
                HelloGroundMode.FCAM_PITCH_ANIMATION_PERIOD_MS)

    def enter(self):
        pass

    def exit(self):
        pass

    def begin_step(self):
        self.tlm_dctl.fetch_sample()

    def end_step(self):
        if self.front_cam_pitch_index < \
               len(HelloGroundMode.FCAM_PITCH_ANIMATION) - 1:
            self.front_cam_pitch_index += 1

    def generate_drone_reference(self):
        pass

    def correct_drone_reference(self):
        pass

    def generate_attitude_references(self):
        # Front
        self.output.has_front_cam_reference = True
        fcam_ref = self.output.front_cam_reference

        fcam_ref.yaw.ctrl_mode = cam_cm_pb2.POSITION
        fcam_ref.yaw.frame_of_ref = cam_for_pb2.ABSOLUTE
        fcam_ref.yaw.position = self.tlm_dctl['attitude_euler_angles.yaw']
        fcam_ref.pitch.ctrl_mode = cam_cm_pb2.POSITION
        fcam_ref.pitch.frame_of_ref = cam_for_pb2.ABSOLUTE
        fcam_ref.pitch.position = \
            HelloGroundMode.FCAM_PITCH_ANIMATION[self.front_cam_pitch_index] \
            * np.pi / 180.0
        fcam_ref.roll.ctrl_mode = cam_cm_pb2.POSITION
        fcam_ref.roll.frame_of_ref = cam_for_pb2.ABSOLUTE
        fcam_ref.roll.position = 0.0
        # Stereo
        self.output.has_stereo_cam_reference = True
        stcam_ref = self.output.stereo_cam_reference

        stcam_ref.yaw.ctrl_mode = cam_cm_pb2.POSITION
        stcam_ref.yaw.frame_of_ref = cam_for_pb2.ABSOLUTE
        stcam_ref.yaw.position = self.tlm_dctl['attitude_euler_angles.yaw']

        stcam_ref.pitch.ctrl_mode = cam_cm_pb2.POSITION
        stcam_ref.pitch.frame_of_ref = cam_for_pb2.ABSOLUTE
        stcam_ref.pitch.position = self.tlm_dctl['attitude_euler_angles.pitch']

        stcam_ref.roll.ctrl_mode = cam_cm_pb2.POSITION
        stcam_ref.roll.frame_of_ref = cam_for_pb2.ABSOLUTE
        stcam_ref.roll.position = self.tlm_dctl['attitude_euler_angles.roll']

    def _timer_cb(self):
        self.log.info("Hello world")
        self.front_cam_pitch_index = 0

GUIDANCE_MODES = {
    'com.parrot.missions.samples.hello_ground' : HelloGroundMode
}
