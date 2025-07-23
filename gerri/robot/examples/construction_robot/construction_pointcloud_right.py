import os, sys
import time

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon

ROBOT_INFO = {
    "id": "construction_pointcloud_right",
    "model": "doosanM1509",
    "category": "sample",
}

# Initialize communication module (AND)
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video=None,
    audio=None
)
daemon.connect()


import rclpy
rclpy.init(args=None)

from gerri.robot.examples.construction_robot.construction_base_pointcloud_right import ConstructionBasePointCloud
# from gerri.robot.examples.construction_robot.construction_sub_controller import DoosanMSubController
#
pointcloud = ConstructionBasePointCloud(ROBOT_INFO, sub_controller=None)
pointcloud.connect()
# Keep process alive
while True:
    time.sleep(1)
