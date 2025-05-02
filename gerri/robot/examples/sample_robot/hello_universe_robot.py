import os, sys
import time

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
# daemon.run_forever()

### Robot system setting ###

# If robot already support by gerri, you can add from ROBOT_INFO['model']
# If you want to check the robot is support by gerri check

# from gerri.robot.controller.manipulator_controller import ManipulatorController
# robot = ManipulatorController(ROBOT_INFO)
# robot.connect()

# But if you want new custom controller, you can make new controller
# Base controller is

from gerri.robot.examples.sample_robot.sample_base_controller import SampleBaseController
# robot = SampleBaseController(ROBOT_INFO)

from gerri.robot.examples.sample_robot.sample_sub_controller import SampleSubController
robot = SampleBaseController(ROBOT_INFO, controller=SampleSubController)
robot.connect()

while True:
    time.sleep(1)