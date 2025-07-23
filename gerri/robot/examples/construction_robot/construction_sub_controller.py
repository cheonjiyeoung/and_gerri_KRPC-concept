import os
import sys
import time
import math


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.construction_robot import doosanM_left_controller
from gerri.robot.examples.construction_robot import doosanM_right_controller
from pymodbus.client import ModbusTcpClient
import time
import threading

class ConstructionSubController:
    def __init__(self):
        """
        ROBOT INIT CODE
        """
        self.base_controller = None
        self.left_robot_state = 'idle'
        self.left_pose = {
            'position': [0, 0, 0],
            'orientation': [0, 0, 0],
        }
        self.left_joint_state = {
            'name': ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
            'position': [0., 0., 0., 0., 0., 0.],
            'velocity': [0, 0, 0, 0],
            'effort': [0, 0, 0, 0],
        }

        self.right_robot_state = 'idle'
        self.right_pose = {
            'position': [0, 0, 0],
            'orientation': [0, 0, 0],
        }
        self.right_joint_state = {
            'name': ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
            'position': [0., 0., 0., 0., 0., 0.],
            'velocity': [0, 0, 0, 0],
            'effort': [0, 0, 0, 0],
        }

        self.doosanM_left_controller = doosanM_left_controller.DoosanMLeftController()
        self.left_doosan_robot_ip = '192.168.10.118'
        self.left_doosan_robot_port = 502

        self.doosanM_right_controller = doosanM_right_controller.DoosanMRightController()
        self.right_doosan_robot_ip = '192.168.10.119'
        self.right_doosan_robot_port = 502

    def init_base_controller(self, base_controller):
        self.base_controller = base_controller

    def connect(self):
        """
        CONNECT TO ROBOT
        """
        self.doosanM_left_controller.connect(self.left_doosan_robot_ip, self.left_doosan_robot_port)
        self.doosanM_right_controller.connect(self.right_doosan_robot_ip, self.right_doosan_robot_port)


    def disconnect(self):
        """
        DISCONNECT FROM ROBOT
        """

    def update_status(self):
        self.doosanM_left_controller.update_status()
        self.left_robot_state = self.doosanM_left_controller.robot_state
        self.left_pose = self.doosanM_left_controller.pose
        self.left_joint_state = self.doosanM_left_controller.joint_state

        self.doosanM_right_controller.update_status()
        self.right_robot_state = self.doosanM_right_controller.robot_state
        self.right_pose = self.doosanM_right_controller.pose
        self.right_joint_state = self.doosanM_right_controller.joint_state

    def hello_universe(self, value):
        print('hello_universe')
        return 'hello_universe_by_robot'

    def left_stop_robot(self, value):
        self.doosanM_left_controller.stop_robot(value)

    def left_joint_ctrl(self, value):
        self.doosanM_left_controller.joint_ctrl(value)

    def left_movel(self, value):
        self.doosanM_left_controller.movel(value)

    def left_joint_ctrl_master(self, value):
        self.doosanM_left_controller.joint_ctrl_master(value)

    def left_gripper_ctrl(self, value):
        self.doosanM_left_controller.gripper_ctrl(value)

    def right_stop_robot(self, value):
        self.doosanM_right_controller.stop_robot(value)

    def right_joint_ctrl(self, value):
        self.doosanM_right_controller.joint_ctrl(value)

    def right_movel(self, value):
        self.doosanM_right_controller.movel(value)

    def right_joint_ctrl_master(self, value):
        self.doosanM_right_controller.joint_ctrl_master(value)

    def right_gripper_ctrl(self, value):
        self.doosanM_right_controller.gripper_ctrl(value)

    def custom_command(self, command):
        return command

    def send_message(self, message):
        self.base_controller.send_message(message)

    @staticmethod
    def clamp(value, min_value, max_value, absolute_limit=None):
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """
        특정 값을 주어진 범위 내에서 다른 범위로 매핑하는 함수.

        :param value: 매핑할 값
        :param in_min: 매핑 전 최소값
        :param in_max: 매핑 전 최대값
        :param out_min: 매핑 후 최소값
        :param out_max: 매핑 후 최대값
        :return: 매핑된 값
        """
        map_value = self.clamp((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max)

        return map_value
