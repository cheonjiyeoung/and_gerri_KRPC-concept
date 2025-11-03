# from _and_ ~~~
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.utils.krpc_robot_controller import KRPCRobotController
from multi_robot_case_test.multiple_robot.torso_arm import TorsoArm
from multi_robot_case_test.multiple_robot.mobile import MobileRobot
from multi_robot_case_test.multiple_robot.robot_config import ROBOT_INFO

from pubsub import pub

torso_left = TorsoArm("left")
torso_right = TorsoArm("right")
mobile_1 = MobileRobot("1")
mobile_2 = MobileRobot("2")

robot_interfaces = {
    "torso_left" : torso_left,
    "torso_right" : torso_right,
    "mobile_1" : mobile_1,
    "mobile_2" : mobile_2
}

krpc_controller = KRPCRobotController(ROBOT_INFO, robot_interfaces)

# test1 send same method another robots
command = {"topic":"move", "value":{"j1":1,"j2":2,"j3":3,"j4":4,"j5":5,"j6":6},"target":"torso_left"}
pub.sendMessage("receive_message",message=command)

command = {"topic":"move","value":{"j1":2,"j2":3,"j3":2,"j4":3,"j5":2,"j6":3},"target":"torso_right"}
pub.sendMessage("receive_message",message=command)

command = {"topic":"move","value":{"vx":0.0,"vy":0.5},"target":"mobile_1"}
pub.sendMessage("receive_message",message=command)

command = {"topic":"move","value":{"j1":2,"j2":3,"j3":2,"j4":3,"j5":2,"j6":3},"target":"mobile_2"}
pub.sendMessage("receive_message",message=command)


