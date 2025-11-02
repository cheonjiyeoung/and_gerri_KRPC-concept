# from _and_ ~~~
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.utils.robot_sub_controller import RobotSubController
from gerri.robot.examples.Spot.Spot_controller import SpotController


# Test
# 1. create robot_controller. (기존 sub_controller + base_controller -> 명칭 변경 예정)
sub_controller = RobotSubController(robot_controller=SpotController())
# sub_controller.connect()

# 2. exac command test. 실제로는 command_handler호출이 아닌 webrtc를 통해 받은 커맨드 처리
command1 = {"topic": "move", "value": {"vx": 2.0, "vy": 1.0, "vth": 0.5}}
sub_controller.command_handler(command1)
command2 = {"topic": "move_waypoint", "value": {"poi": [1.0, 2.0, 0.0]}}
sub_controller.command_handler(command2)
command3 = {"topic": "test_method", "value": {}}
sub_controller.command_handler(command3)
