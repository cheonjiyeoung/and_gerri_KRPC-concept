import os, sys
import time

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))


from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.rtc2kng.construction_vr_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO, SERVER_INFO

# Initialize communication module (AND)
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    server_info=SERVER_INFO,
    network='rtc2kng',
    video_info=VIDEO_INFO,
    audio_info=AUDIO_INFO,

)
daemon.connect()

# Initialize robot controller (GERRI)
# - If the model in ROBOT_INFO is predefined, the base controller will auto-select it.
# - Otherwise,manually specify a sub-controller as shown below.

# from gerri.robot.examples.construction_vr.manipulator_vr_base_controller import ManipulatorVRBaseController
# from gerri.robot.examples.construction_vr.doosan_vr_sub_controller import DoosanSubController
# from gerri.robot.interface.vr_controller import VRController
#
#
# vr_ctrl_interface = VRController()
# sample_base_controller = ManipulatorVRBaseController(ROBOT_INFO, interface=vr_ctrl_interface)
# sample_sub_controller = DoosanSubController(ROBOT_INFO['ip'], ROBOT_INFO['port'])
# sample_base_controller.sub_controller = sample_sub_controller
# sample_sub_controller.base_controller = sample_base_controller
# sample_base_controller.connect()

from gerri.robot.interface.vr_controller import VRController
from gerri.robot.examples.rtc2kng.zoom_test_base_controller import ZoomBaseController

vr_ctrl_interface = VRController()
sample_base_controller = ZoomBaseController(ROBOT_INFO, interface=vr_ctrl_interface)
sample_base_controller.connect()


# Keep process alive
while True:
    time.sleep(1)
