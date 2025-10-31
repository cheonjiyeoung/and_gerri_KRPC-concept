import os, sys
import time

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))


from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.construction_vr.construction_vr_config import ROBOT_INFO, LEFT_ROBOT_INFO, RIGHT_ROBOT_INFO, \
    PAN_TILT_INFO, VIDEO_INFO, AUDIO_INFO, SERVER_INFO, GRIPPER_INFO

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





from gerri.robot.examples.construction_vr.doosan_vr_base_controller import DoosanVRBaseController
from gerri.robot.examples.construction_vr.doosan_vr_sub_controller import DoosanVRSubController
from gerri.robot.interface.vr_controller import VRController

from gerri.robot.examples.pantilt_2kng.pan_tilt_base_controller import PanTiltBaseController
from gerri.robot.examples.pantilt_2kng.pan_tilt_sub_controller import PanTiltSubController

from gerri.robot.examples.zimmer.zimmer_base_controller import ZimmerBaseController

pan_tilt = PanTiltBaseController(PAN_TILT_INFO)
pan_tilt.sub_controller = PanTiltSubController(PAN_TILT_INFO['port'])
pan_tilt.connect()


vr_ctrl_interface = VRController()
left_base_controller = DoosanVRBaseController(LEFT_ROBOT_INFO, interface=vr_ctrl_interface)
left_sub_controller = DoosanVRSubController(LEFT_ROBOT_INFO['ip'], LEFT_ROBOT_INFO['port'], debug=False)
left_base_controller.sub_controller = left_sub_controller
left_sub_controller.base_controller = left_base_controller
left_base_controller.connect()

right_base_controller = DoosanVRBaseController(RIGHT_ROBOT_INFO, interface=vr_ctrl_interface)
right_sub_controller = DoosanVRSubController(RIGHT_ROBOT_INFO['ip'], RIGHT_ROBOT_INFO['port'], debug=False)
right_base_controller.sub_controller = right_sub_controller
right_sub_controller.base_controller = right_base_controller
right_base_controller.connect()

zimmer_base_controller = ZimmerBaseController(GRIPPER_INFO, interface=vr_ctrl_interface)
zimmer_base_controller.connect()

# from gerri.robot.interface.vr_controller import VRController
# from gerri.robot.examples.rtc2kng.zoom_test_base_controller import ZoomBaseController
# from gerri.robot.examples.sample_robot.sample_sub_controller import SampleSubController
# vr_ctrl_interface = VRController()
# sample_base_controller = ZoomBaseController(ROBOT_INFO, interface=vr_ctrl_interface)
# sample_sub_controller = SampleSubController()
# sample_base_controller.sub_controller = sample_sub_controller
# sample_sub_controller.base_controller = sample_base_controller
# sample_base_controller.connect()
#

# Keep process alive
while True:
    time.sleep(1)
