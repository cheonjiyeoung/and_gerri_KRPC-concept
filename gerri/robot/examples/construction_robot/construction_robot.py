import os, sys
import time

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from realsense_D435if_rgb import RosRealsenseRGBCameraManager
from construction_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO

import rclpy
from rclpy.executors import SingleThreadedExecutor

rclpy.init()

def run_camera_node(rgb_node, depth_node):
    """Run ROS2 node in its own thread."""
    # if not rclpy.ok():

    executor = SingleThreadedExecutor()
    executor.add_node(rgb_node)
    executor.add_node(depth_node)
    
    while True:
        executor.spin_once(timeout_sec=0.01)
        time.sleep(0.01)
    rclpy.shutdown()

source_rgb = RosRealsenseRGBCameraManager()
VIDEO_INFO["realsense_D435if_custom"]["source"] = source_rgb

# Initialize communication module (AND)
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video=VIDEO_INFO,
    audio=AUDIO_INFO
)
daemon.connect()

# Initialize robot controller (GERRI)
# - If the model in ROBOT_INFO is predefined, the base controller will auto-select it.
# - Otherwise, manually specify a sub-controller as shown below.

from gerri.robot.examples.construction_robot.construction_base_controller import ConstructionBaseController
from gerri.robot.examples.construction_robot.construction_sub_controller import ConstructionSubController

robot = ConstructionBaseController(ROBOT_INFO, sub_controller=ConstructionSubController())
robot.connect()
# Keep process alive
while True:
    time.sleep(1)
