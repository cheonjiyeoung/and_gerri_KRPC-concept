import os, sys

import numpy as np


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.construction_vr.robot_status import RobotStatus
from gerri.robot.examples.construction_vr.doosan_controller import DoosanController
import threading
import random
import time
import pinocchio as pin


from gerri.robot.function.tf_helper import *
from gerri.robot.function.ik_solver import IKSolver

urdf_path = '/home/orin2kng01/dev/and_gerri/gerri/robot/examples/construction_vr/m1509.urdf'

class DoosanSubController:
    def __init__(self, ip, port):
        self.robot = DoosanController(ip, port)
        self.base_controller = None
        self._lock = threading.Lock()

        self.ik_solver = IKSolver(urdf_path, 'joint_6')

        self.status = None

        self.last_robot_SE3_pose = None


        self.joint_preset = {'home': [-90.00, 0.00, 90.00, 0.00, -45.00, -60.00]}


    def connect(self):
        self.robot.connect()
        self.status = RobotStatus(robot_id=self.base_controller.robot_id,
                                  model=self.base_controller.robot_model,
                                  category=self.base_controller.robot_category)
        threading.Thread(target=self._update_loop,daemon=True).start()

    def _update_loop(self):
        while True:
            self._lock.acquire()
            # For examples #
            self.robot.update_status()
            self.status.joint_state =self.robot.joint_state
            self.status.pose = self.robot.pose
            ###
            self._lock.release()
            time.sleep(0.1)

    def disconnect(self):
        """
        DISCONNECT FROM ROBOT
        """

    def joint_ctrl(self, value, option):
        """
        ROBOT JOINT CONTROL BY DEGREE
        """
        print("Joint Ctrl in Sub", value, option)

    def joint_ctrl_step(self, value, option):
        """
        ROBOT JOINT CONTROL BY DEGREE
        변화량 만큼 이동하는 코드 예 (0,0,0,0,30,0) 5번축만 +30도 회전
        """
        print("joint_ctrl_step in Sub")

    def joint_ctrl_puppet(self, value, option):
        """
        마스터 컨트롤러가 있을 떄 조인트정보를받고 움직이는 코드

        """
        print("joint_ctrl_puppet in Sub", value, option)

    def end_pose_ctrl(self, end_pose):
        print("end_pose_ctrl")

    def end_pose_ctrl_step(self, end_pose_step):
        print("end_pose_step_ctrl")

    def joint_ctrl_vel_stop(self):
        self.robot.joint_ctrl_vel_stop()


    def joint_ctrl_vel_delta(self, start_pose: pin.SE3, delta_pose: pin.SE3, acc=250, dt=0.01):
        """
        BaseController로부터 받은 명령을 수행하는 핵심 함수.
        기준 자세(start_pose)에 변화량(delta_pose)을 적용해 목표 지점으로 이동.
        """
        # 1. 최종 목표 지점 계산
        target_pose = start_pose * delta_pose

        # 2. 현재 관절 각도 가져오기
        current_q_rad = np.deg2rad(self.status.joint_state['position'])

        # 3. IK 솔버를 이용해 관절 속도(dq) 계산
        #    이때는 절대 목표를 추종하는 clik 함수를 사용
        dq = self.ik_solver.clik(current_q_rad, target_pose)

        self.robot.joint_ctrl_vel(dq, acc, dt)

        # 4. 실제 로봇에 속도 명령 전달
        self.robot.joint_ctrl_vel(dq)

    def get_current_SE3_pose(self):
        current_joint_rad = np.deg2rad(self.status.joint_state['position'])
        return self.ik_solver.fk(current_joint_rad)


    def gripper_ctrl(self, value, option):
        """
        페러럴 그리퍼 컨트롤
        """
        print("gripper_ctrl in Sub", value, option)

    def gripper_ctrl_puppet(self, value, option):
        """
        페러럴 그리퍼 컨트롤
        """
        print("gripper_ctrl in Sub", value, option)

    def move(self, value, option):
        print("move", value, option)

    def hello_universe(self, value):
        print('hello_universe')
        self.send_message(value)
        return 'hello_universe_by_robot'

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
