import sys
import os
import asyncio
import time

CURRENT_FILE = os.path.abspath(__file__)
VENV_DIR = os.path.dirname(sys.executable)
PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
sys.path.insert(0, PROJECT_ROOT)

from avatar_darm.robot.network_tools.webrtc.webrtc_command_channel import WebRtcCommandChannel
from avatar_darm.robot.network_tools.webrtc.webrtc_video_channel import WebRtcVideoChannel
from avatar_darm.robot.network_tools.webrtc.webrtc_audio_channel import WebRtcAudioChannel

from avatar_darm.robot.network_tools.zeromq.zeromq_bridge import ZeroMQBridge
from avatar_darm.robot.robot_tools.manipulator_controller import ManipulatorController
from avatar_darm.robot.robot_tools.pantilt.pantilt_controller import PanTiltController

from avatar_darm.robot.robot_tools.piper.avatar_piper_config import *

SERVER_HOST = "175.126.123.199"
SERVER_PORT = "8180"


left_piper = ManipulatorController(robot_name=PUPPET_ARM_NAME_LEFT, robot_model=ROBOT_MODEL, use_bridge=True)
right_piper = ManipulatorController(robot_name=PUPPET_ARM_NAME_RIGHT, robot_model=ROBOT_MODEL, use_bridge=True)
pan_tilt = PanTiltController(robot_name=PAN_TILT_NAME, use_port=PAN_TILT_USB)


robot_group = {
            PUPPET_ARM_NAME_LEFT: left_piper,
            PUPPET_ARM_NAME_RIGHT: right_piper,
            PAN_TILT_NAME: pan_tilt,
}

command_channel = WebRtcCommandChannel(robot_id=ROBOT_ID + "_command",
                                   password="",
                                   server_host=SERVER_HOST,
                                   server_port=SERVER_PORT,
                                   # camera=DummyCamera(),
                                   loop=asyncio.get_event_loop()
                                       )

video_channel = WebRtcVideoChannel(
        robot_id=ROBOT_ID + "_video",
        password="",
        server_host="175.126.123.199",
        server_port="8180"
    )

audio_channel = WebRtcAudioChannel(
    robot_id=ROBOT_ID + "_audio",
    password="",
    server_host="175.126.123.199",
    server_port="8180"
)

bridge = ZeroMQBridge(target_ip=BRIDGE_CLIENT_IP, pub_port=BRIDGE_CLIENT_PUB_PORT, sub_port=BRIDGE_CLIENT_SUB_PORT)

class AvatarRobot:
    def __init__(self, robot, command_channel=None, video_channel=None, audio_channel=None, bridge=None):
        self.controller = robot
        self.command_channel = command_channel
        self.video_channel = video_channel
        self.audio_channel = audio_channel
        self.bridge = bridge


    def connect(self):
        try:
            for controller in self.controller.values():
                controller.connect()
        except Exception as e:
            print(e)

        if self.bridge is not None:
            self.bridge.connect()
            
        if self.video_channel is not None:
            self.video_channel.connect()

        if self.audio_channel is not None:
            self.audio_channel.connect()

        if self.command_channel is not None:
            self.command_channel.connect()




if __name__ == "__main__":
    robot = AvatarRobot(robot_group,command_channel,video_channel,audio_channel,bridge)
    robot.connect()

    while True:
        time.sleep(1)