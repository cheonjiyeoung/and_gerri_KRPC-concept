import time
import os, sys


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.piper.piper_dual_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO

### Network system setting ###

# 🚀 데몬 실행
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    # video_info=VIDEO_INFO,
    # audio_info=AUDIO_INFO
)

daemon.connect()
# daemon.run_forever()

### Robot system setting

from gerri.robot.examples.piper.piper_base_controller import PiperBaseController
from gerri.robot.examples.piper.piper_sub_controller import PiperSubController
from gerri.robot.examples.piper.piper_dual_config import PUPPET_ARM_LEFT, PUPPET_ARM_RIGHT

left_piper_base = PiperBaseController(PUPPET_ARM_LEFT)
left_piper_sub = PiperSubController(PUPPET_ARM_LEFT['id'])
left_piper_base.sub_controller = left_piper_sub
left_piper_sub.base_controller = left_piper_base
left_piper_base.connect()

right_piper_base = PiperBaseController(PUPPET_ARM_RIGHT)
right_piper_sub = PiperSubController(PUPPET_ARM_RIGHT['id'])
right_piper_base.sub_controller = right_piper_sub
right_piper_sub.base_controller = right_piper_base
right_piper_base.connect()


# from gerri.robot.examples.pantilt_2kng.pan_tilt_controller import PanTiltController

# pan_tilt = PanTiltController()
# pan_tilt.connect()


while True:
    time.sleep(1)