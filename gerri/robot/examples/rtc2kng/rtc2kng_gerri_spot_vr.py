import os, sys
import time

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.rtc2kng.rtc2kng_gerri_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO

# Initialize communication module (AND)
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    # network='ketirtc',
    network='rtc2kng',
    command="command",
    video=VIDEO_INFO,
    audio=AUDIO_INFO
)

daemon.connect()

# Initialize robot controller (GERRI)
# - If the model in ROBOT_INFO is predefined, the base controller will auto-select it.
# - Otherwise, manually specify a sub-controller as shown below.

from gerri.robot.examples.rtc2kng.sample_base_controller import SampleBaseController
from gerri.robot.examples.rtc2kng.sample_sub_controller import SampleSubController

# from gerri.robot.examples.pantilt_2kng.pantilt_controller import PanTiltController
# pantilt = PanTiltController()
# pantilt.connect()

robot = SampleBaseController(ROBOT_INFO, sub_controller=SampleSubController())
robot.connect()

# Keep process alive
while True:
    time.sleep(1)
