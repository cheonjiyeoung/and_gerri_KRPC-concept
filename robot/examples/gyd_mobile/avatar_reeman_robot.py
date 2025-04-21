import os
import sys

from pubsub import pub

CURRENT_FILE = os.path.abspath(__file__)
VENV_DIR = os.path.dirname(sys.executable)
PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
sys.path.insert(0, PROJECT_ROOT)

from avatar_darm.robot.network_tools.webrtc.webrtc_command_channel import WebRtcCommandChannel
from avatar_darm.robot.network_tools.webrtc.webrtc_video_channel import WebRtcVideoChannel
from avatar_darm.robot.network_tools.webrtc.webrtc_audio_channel import WebRtcAudioChannel
import asyncio
import time

from avatar_darm.robot.network_tools.webrtc.cam_manager_stream import CV2StreamCamera

from avatar_darm.robot.robot_tools.reeman.config_reeman_robot import *

from avatar_darm.robot.robot_tools.mobile_controller import MobileController
from avatar_darm.robot.robot_tools.dummy_robot_controller import DummyController
from avatar_darm.robot.robot_tools.rlds_dataset_builder import RldsDatasetBuilder


SERVER_HOST = "175.126.123.199"
SERVER_PORT = "8180"

reeman = DummyController()
# right_piper = ManipulatorController(robot_name=PUPPET_ARM_NAME_RIGHT, robot_model=ROBOT_MODEL)
# pan_tilt = PanTiltController(robot_name=PAN_TILT_NAME, use_port=PAN_TILT_USB)

# robot_group = {
#             PUPPET_ARM_NAME_LEFT: left_piper,
#             PUPPET_ARM_NAME_RIGHT: right_piper,
#             # PAN_TILT_NAME: pan_tilt,
# }

robot_group = {ROBOT_ID: reeman}

command_channel = WebRtcCommandChannel(robot_id=ROBOT_ID + "_command",
                                   password="",
                                   server_host=SERVER_HOST,
                                   server_port=SERVER_PORT,
                                   # camera=DummyCamera(),
                                   loop=asyncio.get_event_loop()
                                       )


camera_streamers = {}
video_channel_group = {}

for cam_key, cam_info in CAMERA_INFO.items():
    # ✅ 카메라 스트리머 생성
    streamer = CV2StreamCamera(
        camera_name=cam_info['name'],
        camera_index=cam_info['index'],
        width=cam_info['width'],
        height=cam_info['height'],
        fps=cam_info['fps']
    )
    camera_streamers[cam_key] = streamer

    # ✅ WebRTC Video Channel 생성
    stream_id = f"{ROBOT_ID}_{cam_info['name']}"
    video_channel_group[cam_key] = WebRtcVideoChannel(
        robot_id=stream_id,
        password="",
        server_host=SERVER_HOST,
        server_port=SERVER_PORT,
        camera=streamer
    )

audio_channel = None

class AvatarRobot:
    def __init__(self, robot, command_channel=None, video_channel=None, audio_channel=None):
        self.controller = robot
        self.command_channel = command_channel
        self.video_channel = video_channel
        self.audio_channel = audio_channel



    def connect(self):
        try:
            for controller in self.controller.values():
                controller.connect()
                self.dataset_builder = RldsDatasetBuilder(controller, CAMERA_INFO, interval=0.1,
                                                      save_dir="/media/dev2kng/SanDisk/RLDS_dataset")

        except Exception as e:
            print(e)

        if self.video_channel is not None:
            try:
                for video_channel in self.video_channel.values():
                    video_channel.connect()
            except Exception as e:
                print(e)
            # self.video_channel.connect()

        if self.audio_channel is not None:
            self.audio_channel.connect()

        if self.command_channel is not None:
            self.command_channel.connect()



if __name__ == "__main__":
    robot = AvatarRobot(robot_group,command_channel,video_channel_group,audio_channel)
    robot.connect()

    # print("Robot Connected")
    #
    # time.sleep(3)
    # # ✅ 에피소드 시작
    # pub.sendMessage("start_episode", episode_name="test_episode")
    #
    # print("Robot Started")
    #
    # time.sleep(10)  # 10초 동안 데이터 수집
    #
    # pub.sendMessage("fail_episode")
    # time.sleep(5)
    #
    # print("Robot Stopped")
    #
    #
    # time.sleep(10)  # 10초 동안 데이터 수집
    # print("Robot Stopped")
    #
    #
    # # ✅ 에피소드 종료
    # pub.sendMessage("end_episode")

    while True:
        time.sleep(1)