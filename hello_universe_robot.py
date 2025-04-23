import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon

### Network system setting ###

# 🎥 카메라 구성
from hello_universe_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO

# 🚀 데몬 실행
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video=VIDEO_INFO,
    audio=AUDIO_INFO
)

daemon.connect()
daemon.run_forever()

### Robot system setting ###

from gerri.robot.manipulator_controller import ManipulatorController
robot = ManipulatorController(ROBOT_INFO)
robot.connect()