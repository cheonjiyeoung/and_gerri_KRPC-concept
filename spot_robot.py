import logging
from gerri.robot.utils.krpc_robot_controller import KRPCRobotController
from gerri.robot.examples.spot.spot_sub_controller import SpotSubController

# 로그 레벨을 ERROR로 설정하여 WARNING 이하의 로그를 표시하지 않음
logging.basicConfig(level=logging.ERROR)

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.spot.spot_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO, SPOT_INFO
# Initialize robot controller (GERRI)
robot_interface = SpotSubController(SPOT_INFO)
robot_controller = KRPCRobotController(robot_info=ROBOT_INFO, robot_interface=robot_interface)
robot_controller.connect()

# Initialize communication module (AND)
daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network='ketirtc',
    command="command",
    video=VIDEO_INFO,
    audio=AUDIO_INFO,
)
# daemon.connect()

