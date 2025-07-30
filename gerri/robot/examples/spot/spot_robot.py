import os, sys
import time
import asyncio
from aiortc import RTCConfiguration
import threading

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.spot.spot_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO, SPOT_INFO

# Initialize robot controller (GERRI)
# - If the model in ROBOT_INFO is predefined, the base controller will auto-select it.
# - Otherwise, manually specify a sub-controller as shown below.

from gerri.robot.examples.spot.spot_base_controller import SpotBaseController
from gerri.robot.examples.spot.spot_sub_controller import SpotSubController
from gerri.robot.examples.spot.spot_utils.spot_ptz_cam_receiver import SpotWebRTCClient

spot_base_controller = SpotBaseController(ROBOT_INFO)
spot_sub_controller = SpotSubController(SPOT_INFO)
spot_base_controller.sub_controller = spot_sub_controller
spot_sub_controller.base_controller = spot_base_controller
spot_base_controller.connect()

config = RTCConfiguration(iceServers=[])
webrtc_client = SpotWebRTCClient(hostname="192.168.50.3",
                    sdp_port=31102,
                    sdp_filename="h264.sdp",
                    cam_ssl_cert=False,
                    token=spot_sub_controller.robot_controller.robot.user_token,
                    rtc_config=config
)

loop = asyncio.get_event_loop()
loop.create_task(webrtc_client.start())

spot_rest_cam = spot_sub_controller.robot_controller.spot_camera

# Initialize communication module (AND)
VIDEO_INFO["ptz_cam"]["source"] = webrtc_client
VIDEO_INFO["rest_cam"]["source"] = spot_rest_cam
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video=VIDEO_INFO,
    # audio=AUDIO_INFO,
)
daemon.connect()

while True:
    time.sleep(1)