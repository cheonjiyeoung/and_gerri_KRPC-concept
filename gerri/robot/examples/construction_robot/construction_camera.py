import os, sys
import time

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from construction_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO

# Initialize communication module (AND)
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command=None,
    video=VIDEO_INFO,
    audio=None
)
daemon.connect()

# Initialize robot controller (GERRI)
# - If the model in ROBOT_INFO is predefined, the base controller will auto-select it.
# - Otherwise, manually specify a sub-controller as shown below.

# import rclpy
#
# from gerri.robot.examples.construction_robot.construction_base_controller import ConstructionBaseController
# from gerri.robot.examples.construction_robot.construction_sub_controller import DoosanMSubController
#
# robot = ConstructionBaseController(ROBOT_INFO, sub_controller=DoosanMSubController())
# robot.connect()

# Keep process alive
while True:
    time.sleep(1)
