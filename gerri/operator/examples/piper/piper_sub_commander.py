from PySide6.QtCore import Qt
import pickle
from pubsub import pub

import time

from gerri.operator.interface.master_arm.master_arm import MasterArm
from gerri.operator.examples.piper.piper_dual_operator_config import (
    MASTER_ARM_USB_LEFT, MASTER_ARM_USB_RIGHT,
    PUPPET_ARM_NAME_LEFT, PUPPET_ARM_NAME_RIGHT,
    JOINT_LIMIT_LEFT, JOINT_LIMIT_RIGHT)

MASTER_ARM_PORT_L = MASTER_ARM_USB_LEFT
MASTER_ARM_PORT_R = MASTER_ARM_USB_RIGHT

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

class PiperSubCommander:
    def __init__(self, **kwargs):
        self.control_target = 'all'
        self.use_master_arm = False
        self.master_control = False
        self.base_commander = None
        self.puppet = [PUPPET_ARM_NAME_LEFT, PUPPET_ARM_NAME_RIGHT]
        self.app = kwargs.get('app')

        pub.subscribe(self.key_mouse_control, 'key_mouse_control')
        pub.subscribe(self.ui_signal, 'ui_signal')

    """
    Initializes the connection for the sub-function (e.g., hardware setup, activation).
    """
    def connect(self):
        self.master_arm_left = MasterArm(7, MASTER_ARM_PORT_L)
        self.master_arm_right = MasterArm(7, MASTER_ARM_PORT_R)
        self.initialize()


    """
    Handles cleanup and shutdown for the sub-function (if applicable).
    """
    def disconnect(self):
        pass
        ### TODO : ADD DISCONNECT FUNCTION


    def initialize(self):
        if self.master_arm_left is not None:
            self.master_arm_left.updateDefaultPosCnt()

        if self.master_arm_right is not None:
            self.master_arm_right.updateDefaultPosCnt()

    def enable_master_arm(self):
        self.initialize()

        time.sleep(1)

        self.master_control = True


    def get_status(self, value):
        pass

    def ui_signal(self,signal):
        vx=0
        vy=0
        vth=0
        if signal == "clicked_W":
            vx=1
        if signal == "clicked_A":
            vy=-1
        if signal == "clicked_S":
            vx=-1
        if signal == "clicked_D":
            vy=1
        if signal == "clicked_Q":
            vth=-1
        if signal == "clicked_E":
            vth=1
        value = {"vx":vx,"vy":vy,"vth":vth}
        self.base_commander.hello_universe("hello")

    """
    Clamp a given value to a specified min-max range.
    Optionally constrain further with an absolute limit.
    """
    def clamp(self, value, min_value, max_value, absolute_limit=None):
        """
        값을 주어진 범위로 제한 (클램핑).

        :param value: 제한할 값
        :param min_value: 최소값
        :param max_value: 최대값
        :param absolute_limit: 절대 제한 (튜플, optional)
        :return: 제한된 값
        """
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))

    """
    Maps a value from one range into another using linear scaling.
    """
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

    """
    Handles key and mouse input events and maps them to robot commands.
    """

    def key_mouse_control(self, command):
        # print(command)
        key = command['key_control']
        mouse_d_move = command['mouse_d_move']
        mouse_d_wheel = command['mouse_d_wheel']
        mouse_click = command['mouse_click']


        if "RETURN" in key:
            self.base_commander.get_robot_status(target=self.control_target)


        if "TAB" in key:
            current_index = self.puppet.index(self.control_target)
            next_index = (current_index + 1) % len(self.puppet)
            self.control_target = self.puppet[next_index]
            print(f"Switched control target to: {self.control_target}")

        if "SHIFT" in key:
            if mouse_d_move[0] or mouse_d_move[1] or mouse_d_wheel[0]:
                self.base_commander.joint_ctrl_step(joint_angle_step=[-mouse_d_move[0] * 20, mouse_d_wheel[0] * 20, -mouse_d_wheel[0] * 20, 0, mouse_d_move[1] * 20, 0], target=self.control_target)
            if mouse_click[0]:
                 self.base_commander.gripper_ctrl(gripper_width=0, target=self.control_target)
            if mouse_click[2]:
                self.base_commander.gripper_ctrl(gripper_width=100000000, target=self.control_target)

        if "F10" in key:
            self.base_commander.connect_robot(target=self.control_target)

        if "F4" in key:
            self.base_commander.set_master_joint(target=self.control_target)

        if "F5" in key:
            self.enable_master_arm()

        if "F6" in key:
            self.disable_master_arm()

        if "F1" in key:
            self.base_commander.joint_preset(preset_name='home', target=self.control_target)
        if "F2" in key:
            self.base_commander.joint_preset(preset_name='ready', target=self.control_target)

        if 'CONTROL' in key:
            if '1' in key:
                joint_command = self.base_commander.joint_preset(preset_name='1', target='puppet_right')

            if '2' in key:
                joint_command = self.base_commander.joint_preset(preset_name='2', target='puppet_right')

            if '3' in key:
                self.base_commander.joint_preset(preset_name='3', target='puppet_right')
            if '4' in key:
                self.base_commander.joint_preset(preset_name='4', target='puppet_right')

            if '5' in key:
                self.base_commander.joint_preset(preset_name='5', target='puppet_left')


            if '6' in key:
                self.base_commander.joint_preset(preset_name='6', target='puppet_left')



        if self.master_control:
            joint_value_left = self.master_arm_left.get_position_deg()
            if self.limit_e_stop(joint_value_left, JOINT_LIMIT_LEFT) is False:
                self.base_commander.joint_ctrl_master(joint_value_left[:6], target=PUPPET_ARM_NAME_LEFT)
                self.base_commander.gripper_ctrl_master(master_gripper_width=joint_value_left[6], target=PUPPET_ARM_NAME_LEFT)


            joint_value_right = self.master_arm_right.get_position_deg()
            if self.limit_e_stop(joint_value_right, JOINT_LIMIT_RIGHT) is False:
                self.base_commander.joint_ctrl_master(joint_value_right[:6], target=PUPPET_ARM_NAME_RIGHT)
                self.base_commander.gripper_ctrl_master(master_gripper_width=joint_value_right[6], target=PUPPET_ARM_NAME_RIGHT)


    def limit_e_stop(self, joint_value, joint_limit):
        """
        조인트 값이 리미트를 초과하면 긴급 정지.

        :param joint_value: 조인트 값 리스트
        :param joint_limit: 리미트 범위 리스트
        :return: True (긴급 정지 필요), False (정상 범위)
        """
        # for idx, value in enumerate(joint_value):
        #     min_limit, max_limit = joint_limit[idx]
        #     if value < min_limit:
        #         value = min_limit
        #     elif value > max_limit:
        #         value = max_limit
        #         # self.disable_master_arm()
        #         print(
        #             f"JOINT LIMIT EXCEEDED: Joint {idx + 1} | Value: {value} | Limits: [{min_limit}, {max_limit}]")
        #         # return True
        return False
