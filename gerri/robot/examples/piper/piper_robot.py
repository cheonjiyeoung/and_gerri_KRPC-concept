import time
import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.piper.piper_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO

### Network system setting ###

# 🚀 데몬 실행
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video=VIDEO_INFO,
    audio=AUDIO_INFO
)

daemon.connect()
# daemon.run_forever()

### Robot system setting

from gerri.robot.controller.manipulator_controller import ManipulatorController
from gerri.robot.examples.pantilt_2kng.pantilt_controller import PanTiltController
from gerri.robot.examples.piper.piper_config import PUPPET_ARM_LEFT, PUPPET_ARM_RIGHT

left_piper = ManipulatorController(PUPPET_ARM_LEFT)
left_piper.connect()

right_piper = ManipulatorController(PUPPET_ARM_RIGHT)
right_piper.connect()

pan_tilt = PanTiltController()
pan_tilt.connect()

while True:
    time.sleep(1)