import os, sys
import time
import asyncio
from aiortc import RTCConfiguration
import threading
import logging

# 로그 레벨을 ERROR로 설정하여 WARNING 이하의 로그를 표시하지 않음
logging.basicConfig(level=logging.ERROR)
# logging.getLogger("ketirtc").setLevel(logging.ERROR)
# logging.getLogger("bosdyn.mission").setLevel(logging.DEBUG)
# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.spot.spot_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO, SPOT_INFO, GERRI_ROBOT_INFO, GERRI_VIDEO_INFO

# Initialize robot controller (GERRI)
# - If the model in ROBOT_INFO is predefined, the base controller will auto-select it.
# - Otherwise, manually specify a sub-controller as shown below.

from gerri.robot.examples.spot.spot_base_controller import SpotBaseController
from gerri.robot.examples.spot.spot_sub_controller import SpotSubController
from gerri.robot.examples.spot.spot_utils.spot_ptz_cam_receiver import SpotWebRTCClient
from gerri.robot.raas_dataset_builder import RaasDatasetBuilder

def main():
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
    daemon = AdaptiveNetworkDaemon(
        robot_info=ROBOT_INFO,
        network='ketirtc',
        command="command",
        video=VIDEO_INFO,
        audio=AUDIO_INFO,
    )
    daemon.connect()
    dataset_builder_camera_info = {}
    for key in GERRI_VIDEO_INFO.keys():
        cam_info = GERRI_VIDEO_INFO[key]
        if type(cam_info["source"]) == int:
            dataset_builder_camera_info[key]=cam_info

    # seperate gerri channels
    GERRI_VIDEO_INFO["ptz_cam"]["source"] = webrtc_client
    GERRI_VIDEO_INFO["rest_cam"]["source"] = spot_rest_cam
    gerri_daemon = AdaptiveNetworkDaemon(
        robot_info=GERRI_ROBOT_INFO,
        network='ketirtc',
        command="command",
        video=GERRI_VIDEO_INFO,
        audio=None
    )
    gerri_daemon.connect()

    dataset_builder = RaasDatasetBuilder(controller=spot_sub_controller,
                                        camera_info=dataset_builder_camera_info,
                                        )

    from gerri.robot.examples.spot.spot_utils.mission_callback_receiver import MissionCallbackReceiver
    spot_callback_reciever = MissionCallbackReceiver()
    th_callback_reciever = threading.Thread(target=spot_callback_reciever.open_socket,daemon=True)
    th_callback_reciever.start()

if __name__ == "__main__":
    try:
        main()
        while True:
            time.sleep(1)
    except Exception as ex:
        print(f"Error happened: {ex}")
    finally:
        print("Program exited")
