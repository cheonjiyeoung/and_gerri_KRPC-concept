from and_gerri._and_.and_robot import AdaptiveNetworkDaemon
from and_gerri.gerri.robot.raas_dataset_builder import RaasDatasetBuilder
from robot.__ROBOT_CONFIG__ import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO
# Initialize communication module (AND)
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video_info=VIDEO_INFO,
    audio_info=AUDIO_INFO,
)
daemon.connect()

# Initialize robot controller (GERRI)
# - If the model in ROBOT_INFO is predefined, the base controller will auto-select it.
# - Otherwise, manually specify a sub-controller as shown below.

from __ROBOT_SUB_CONTROLLER__ import __ROBOT_SUB_CONTROLLER_CLASS__
from and_gerri.gerri.robot.interface.krpc_interface import KrpcInterface

# If you want control multiple robots. you can add more subcontroller in sub_controllers
"""
ex)
sub_controllers = {"robot_id_1 : robot_id_1_subcontroller,
                   "robot_id_2 : robot_id_2_subcontroller,
                   ...}
"""

sub_controllers = __SUB_CONTROLLERS__
krpc_interface = KrpcInterface(sub_controllers)
krpc_interface.connect()

raas_dataset_builder = RaasDatasetBuilder(controller=sub_controllers,
                                          camera_info=VIDEO_INFO)

# Keep process alive
import time
while True:
    time.sleep(5)