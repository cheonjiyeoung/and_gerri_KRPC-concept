from and_gerri._and_.and_robot import AdaptiveNetworkDaemon
from and_gerri.gerri.robot.raas_dataset_builder import RaasDatasetBuilder
from gyd_mobile.robot.gyd_mobile_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO
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

from gyd_mobile.robot.gyd_mobile_sub_controller import GydMobileSubController
from gyd_mobile.robot.interface.rubberneck_joystick import RubberNeckJoyStick
from and_gerri.gerri.robot.interface.krpc_interface import KrpcInterface

# If you want control multiple robots. you can add more subcontroller in sub_controllers
# But if you control multiple robots. not recommend using RubberNeckJoyStick
"""
ex)
sub_controllers = {"robot_id_1 : robot_id_1_subcontroller,
                   "robot_id_2 : robot_id_2_subcntroller,
                   ...}
krpc_interface = KrpcInterface(sub_controllers)
"""
gyd_mobile_subcontroller = GydMobileSubController(ROBOT_INFO['id'], ROBOT_INFO['model'], ROBOT_INFO['category'])
krpc_interface = KrpcInterface(ROBOT_INFO,gyd_mobile_subcontroller)
rubberneck_joystick = RubberNeckJoyStick(gyd_mobile_subcontroller)
krpc_interface.connect()
# raas_dataset_builder = RaasDatasetBuilder(controller=gyd_mobile_subcontroller,
#                                           camera_info=VIDEO_INFO)

# Keep process alive
import time
while True:
    time.sleep(5)