import numpy as np
import time
from piper_sdk import *
import math

# Piper 인터페이스 초기화 및 연결
FACTOR_DEGREE = 1000 # 1도는 1000펄스임

class PiperController:
    def __init__(self, can_port):
        self.robot = C_PiperInterface(can_port)
        self.factor_degree = FACTOR_DEGREE

        self.joint_preset = {'home': [0, 0, 0, 0, 0, 0],
                             'ready': [0, 30, -30, 0, 0, 0],
                             'master': [0, 30, -30, 0, 0, 0]}

        self.default_joint_angle = self.joint_preset['ready']
        self.last_joint_angle = self.default_joint_angle

        self.joint_state = {
            'name': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
            'position': self.deg2rad(self.get_joint_angles())
        }
        self.robot.ConnectPort()



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
        self.robot.ConnectPort()
        time.sleep(1)
        self.robot.EnableArm(7)
        time.sleep(1)
        # self.robot.MotionCtrl_1(0x02, 0, 0)
        self.robot.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        time.sleep(1)
        self.robot.status = 'connect'
        self.get_joint_angles()
        time.sleep(1)
        self.joint_ctrl(self.joint_preset['ready'])
        self.get_joint_angles()
        time.sleep(1)
        self.joint_ctrl(self.joint_preset['home'])
        self.get_joint_angles()

    def disconnect(self):
        self.robot.MotionCtrl_1(0x02, 0, 0)
        self.robot.MotionCtrl_2(0, 0, 0, 0x00)
        self.robot.status = 'disconnect'

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
            round((self.joint_preset['master'][4] - master_joint_angles[4]) * self.factor_degree),
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

    def update_status(self):
        self.joint_state['position'] = [round(rad, 3) for rad in self.deg2rad(self.get_joint_angles())]

'''
1. piper.MotionCtrl_1

주요 목적:
로봇의 운동 상태(Trajectory Control, Emergency Stop, Drag-Teach)를 설정합니다.

사용 예시:
긴급 정지, 경로 초기화, 또는 teach mode(드래그로 경로 생성) 진입/종료.

매개변수:
piper.MotionCtrl_1(emergency_stop, track_ctrl, grag_teach_ctrl)
emergency_stop: 긴급 정지 명령.
0x01: 긴급 정지.
0x02: 정지 해제.
track_ctrl: 경로 명령.
0x00: 꺼짐.
0x01~0x08: 경로 일시 정지, 재개, 초기화 등.
grag_teach_ctrl: Teach Mode 명령.
0x01: Teach Mode 시작.
0x02: Teach Mode 종료.
용도:
로봇의 전체적인 상태를 설정하며, 특정 운동을 실행하기보다 모드 전환과 시스템 초기화에 초점.


2. piper.MotionCtrl_2
주요 목적:
로봇의 운동 방식 및 속도를 제어합니다.

사용 예시:
조인트(Joint), 직선(Linear), 원형(Circular) 경로 등 특정 운동 제어.

매개변수:
piper.MotionCtrl_2(ctrl_mode, move_mode, move_spd_rate_ctrl, is_mit_mode=0x00)
ctrl_mode: 제어 모드.
0x00: 대기 모드.
0x01: CAN 명령 제어.
0x02: Teach 모드.
0x03: 이더넷 제어.
move_mode: 이동 모드.
0x00: MOVE P(Position).
0x01: MOVE J(Joint).
0x02: MOVE L(Linear).
0x03: MOVE C(Circular).
move_spd_rate_ctrl: 속도 제어 (0~100%).
is_mit_mode: MIT 모드 설정.

용도:
로봇의 실제 운동(조인트/직선/원형 이동)을 명령하며, 실행 단계에서 운동 궤적을 직접 관리.

3. piper.JointCtrl
주요 목적:
각 조인트의 목표 각도를 개별적으로 설정합니다.

사용 예시:
특정 각도로 로봇팔 조인트 이동.

매개변수:
piper.JointCtrl(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
joint_1 ~ joint_6: 각 조인트의 목표 각도 (단위: 0.001°).

용도:
특정 각도로 정밀 제어를 수행하며, MotionCtrl 명령들과 달리 궤적 계산 없이 즉시 목표 값으로 이동.

'''
'''
gripper_angle (int)

설명: 그리퍼의 각도를 설정합니다. 단위는 0.001°로, 소수점 표현 대신 정수 값으로 다룹니다.
예시:
10000 → 10.000° (10도)
-5000 → -5.000° (-5도, 반대 방향)

gripper_effort (int)

설명: 그리퍼의 토크(힘)를 설정합니다. 단위는 0.001 N/m로, 소수점 표현 대신 정수 값을 사용합니다.
예시:
500 → 0.5 N/m
10000 → 10 N/m

gripper_code (int)

설명: 그리퍼의 작동 상태를 제어하는 코드입니다. 아래의 값을 사용합니다:
0x00: 비활성화 (Disable)
0x01: 활성화 (Enable)
0x03: 활성화 후 에러 클리어 (Enable _and_ Clear Error)
0x02: 비활성화 후 에러 클리어 (Disable _and_ Clear Error)

set_zero (int)

설명: 현재 위치를 기준점(제로 포인트)으로 설정할지 여부를 제어합니다.
값:
0x00: 제로 설정 없음
0xAE: 현재 위치를 제로 포인트로 설정
'''