import sys
import os
import asyncio
import time


CURRENT_FILE = os.path.abspath(__file__)
VENV_DIR = os.path.dirname(sys.executable)
PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
sys.path.insert(0, PROJECT_ROOT)

from avatar_darm.robot.robot_tools.manipulator_controller import ManipulatorController
from avatar_darm.robot.network_tools.zeromq.zeromq_robot import ZeroMQRobot
from avatar_darm.robot.robot_tools.piper.avatar_piper_config import *



SERVER_HOST = "175.126.123.199"
SERVER_PORT = "8180"

left_piper = ManipulatorController(robot_name=PUPPET_ARM_NAME_LEFT, robot_model=ROBOT_MODEL)
right_piper = ManipulatorController(robot_name=PUPPET_ARM_NAME_RIGHT, robot_model=ROBOT_MODEL)
# pan_tilt = PanTiltController(robot_name=PAN_TILT_NAME, use_port=PAN_TILT_USB)

robot_group = {
            PUPPET_ARM_NAME_LEFT: left_piper,
            PUPPET_ARM_NAME_RIGHT: right_piper,
}

# robot_group = None

command_channel = ZeroMQRobot(target_ip=BRIDGE_SERVER_IP, pub_port=BRIDGE_SERVER_PUB_PORT, sub_port=BRIDGE_SERVER_SUB_PORT)

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

        if self.video_channel is not None:
            self.video_channel.connect()

        if self.audio_channel is not None:
            self.audio_channel.connect()

        if self.command_channel is not None:
            self.command_channel.connect()

        if self.bridge is not None:
            self.bridge.connect()



if __name__ == "__main__":
    robot = AvatarRobot(robot_group,command_channel)
    robot.connect()

    while True:
        time.sleep(1)