import numpy as np
import time
from piper_sdk import * # ROBOT CONTROLLER
import math
from gerri.robot.examples.piper.piper_status import RobotStatus
import threading

# Piper 인터페이스 초기화 및 연결
FACTOR_DEGREE = 1000 # 1도는 1000펄스임

class PiperSubController:
    def __init__(self, can_port):
        self.robot = C_PiperInterface(can_port)
        self.factor_degree = FACTOR_DEGREE
        self.status = None
        self.joint_preset = {'home': [0, 80, -160, 0, 0, 0],
                             'ready': [0, 90, -90, 0, 0, 0],
                             'master': [0, 80, -160, 0, 0, 0]}


        self.default_joint_angle = self.joint_preset['ready']
        self.last_joint_angle = self.default_joint_angle

        self.status.joint_state = {
            'name': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
            'position': self.deg2rad(self.get_joint_angles())
        }

    def init_base_controller(self, base_controller):
        self.status = RobotStatus(robot_id=base_controller.robot_info["id"],
                                  model=base_controller.robot_info["model"],
                                  category=base_controller.robot_info["category"],)

        self._status_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._lock = threading.Lock()
        self._status_thread.start()


    def _update_loop(self):
        while True:
            self._lock.acquire()
            ### STATUS UPDATE
            self.status.joint_state = self.get_joint_angles()
            self._lock.release()
            time.sleep(0.1)

    def initialize(self):
        self.joint_preset['master'] = self.last_joint_angle

    def get_joint_angles(self):
        # ArmJoint 객체에서 직접 joint_state 속성에 접근
        joint_msg = self.robot.GetArmJointMsgs()

        # ArmMsgJointFeedBack 속성에서 각도를 리스트로 저장
        self.last_joint_angle = [
            joint_msg.joint_state.joint_1 * 0.001,  # 0.001도를 실제 도로 변환
            joint_msg.joint_state.joint_2 * 0.001,
            joint_msg.joint_state.joint_3 * 0.001,
            joint_msg.joint_state.joint_4 * 0.001,
            joint_msg.joint_state.joint_5 * 0.001,
            joint_msg.joint_state.joint_6 * 0.001,
        ]
        print(self.last_joint_angle)
        return self.last_joint_angle


    def connect(self):
        self.robot.connect()
        self.get_joint_angles()
        time.sleep(1)
        self.joint_ctrl(self.joint_preset['ready'])
        self.get_joint_angles()
        time.sleep(1)
        self.joint_ctrl(self.joint_preset['home'])
        self.get_joint_angles()

    def disconnect(self):
        self.robot.disconnect()


    def joint_ctrl(self, joint_angles: list):
        scaled_value = [round(joint_angle * self.factor_degree) for joint_angle in joint_angles]
        print(scaled_value)
        self.robot.JointCtrl(*scaled_value)

    def joint_ctrl_step(self, joint_angles: list):
        """
        각 조인트에 스텝 값을 추가하여 제어
        Args:
            joint_angles (list): 각 조인트에 추가할 스텝 값 (예: [+10, -10, +5, 0, 0, -5])
        """
        # 현재 조인트 값 가져오기 (0.001도 단위로 되어 있으므로 변환 필요)
        current_joint_angles = self.get_joint_angles()  # 현재 값: [0.0, 0.0, ...]

        # 스텝 값 추가 및 스케일 적용
        scaled_value = [
            round((current + step) * self.factor_degree)
            for current, step in zip(current_joint_angles, joint_angles)
        ]

        # 조정된 값을 Joint Control 함수에 전달
        self.robot.JointCtrl(*scaled_value)

    def joint_ctrl_puppet(self, master_joint_angles):
        scaled_value = [
            round((self.joint_preset['master'][0] + master_joint_angles[0]) * self.factor_degree),
            round((self.joint_preset['master'][1] + master_joint_angles[1]) * self.factor_degree),
            round((self.joint_preset['master'][2] + master_joint_angles[2]) * self.factor_degree),
            round((self.joint_preset['master'][3] + master_joint_angles[3]) * self.factor_degree),
            round((self.joint_preset['master'][4] + master_joint_angles[4]) * self.factor_degree),
            round((self.joint_preset['master'][5] + master_joint_angles[5]) * self.factor_degree),
        ]

        # 조정된 값을 Joint Control 함수에 전달
        self.robot.JointCtrl(*scaled_value)

    def gripper_ctrl(self, gripper_angle):
        self.robot.GripperCtrl(gripper_angle,  1000, 0x01, 0)

    def gripper_ctrl_puppet(self, gripper_angle):
        gripper_scale_value = round(self.map_value(gripper_angle, -50, 0, 0, 100)*self.factor_degree)
        print(gripper_scale_value)
        self.robot.GripperCtrl(gripper_scale_value,  1000, 0x01, 0)

    def send_command(self, command):
        pass

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


    def deg2rad(self, degrees):
        """리스트든 단일 값이든 degree(도)를 radian(라디안)으로 변환"""
        if isinstance(degrees, (int, float)):  # 단일 값
            return degrees * (math.pi / 180)
        elif isinstance(degrees, (list, tuple, set)):  # 여러 개의 값 (배열)
            return [deg * (math.pi / 180) for deg in degrees]
        else:
            raise TypeError("Input must be an int, float, list, tuple, or set")


    def rad2deg(self, radians):
        """리스트든 단일 값이든 radian(라디안)을 degree(도)로 변환"""
        if isinstance(radians, (int, float)):  # 단일 값
            return radians * (180 / math.pi)
        elif isinstance(radians, (list, tuple, set)):  # 여러 개의 값 (배열)
            return [rad * (180 / math.pi) for rad in radians]
        else:
            raise TypeError("Input must be an int, float, list, tuple, or set")

    def custom_command(self, command):
        print("CUSTOM COMMAND")


if __name__ == '__main__':
    piper = PiperSubController()
    piper.initialize()