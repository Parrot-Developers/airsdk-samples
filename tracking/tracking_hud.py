import olympe
import os
import sys
import time
from olympe.video.pdraw import Pdraw, PdrawState
from olympe.video.renderer import PdrawRenderer
from olympe.video import HudType


SKYCTRL_IP = os.environ.get("SKYCTRL_IP", "192.168.53.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", "554")


def main():
    skyctrl = olympe.SkyController4(SKYCTRL_IP)
    skyctrl.connect()

    pdraw = Pdraw()
    pdraw.play(url=f"rtsp://{SKYCTRL_IP}:{DRONE_RTSP_PORT}/live")
    renderer = PdrawRenderer(pdraw=pdraw, hud_type=HudType.TRACKING)
    assert pdraw.wait(PdrawState.Playing, timeout=5)

    time.sleep(90)

    pdraw.close()
    skyctrl.disconnect()
    assert pdraw.wait(PdrawState.Closed, timeout=5)
    renderer.stop()
    pdraw.destroy()


def test_pdraw():
    main()


if __name__ == "__main__":
    main()
