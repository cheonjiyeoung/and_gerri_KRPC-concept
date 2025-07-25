import os, sys
import time

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from hello_universe_robot_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO

# Initialize communication module (AND)
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video=VIDEO_INFO,
    audio=AUDIO_INFO,
)
daemon.connect()

# Initialize robot controller (GERRI)
# - If the model in ROBOT_INFO is predefined, the base controller will auto-select it.
# - Otherwise, manually specify a sub-controller as shown below.

from gerri.robot.examples.sample_robot.sample_base_controller import SampleBaseController
from gerri.robot.examples.sample_robot.sample_sub_controller import SampleSubController

sample_base_controller = SampleBaseController(ROBOT_INFO)
sample_sub_controller = SampleSubController()
sample_base_controller.sub_controller = sample_sub_controller
sample_sub_controller.base_controller = sample_base_controller

sample_base_controller.connect()

# Keep process alive
while True:
    time.sleep(1)
