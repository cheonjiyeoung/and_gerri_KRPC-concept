import os, sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.construction_vr.robot_status import RobotStatus
from gerri.robot.examples.sample_robot.sample_robot import SampleRobotController
from gerri.robot.examples.construction_vr.doosan_controller import DoosanController
from gerri.robot.examples.construction_vr.manipulator_ik_solver import ManipulatorIKSolver
import threading
import random
import time
import pinocchio as pin

urdf_path = '/home/orin2kng01/dev/and_gerri/gerri/robot/examples/construction_vr/m1509.urdf'

class DoosanSubController:
    def __init__(self, ip, port):
        self.robot = DoosanController(ip, port)
        self.base_controller = None
        self._lock = threading.Lock()

        self.ik_solver = ManipulatorIKSolver(urdf_path, 'joint_6')
        self.status = None

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

    def clik_ctrl(self, delta_pose):
        current_joint_rad = np.deg2rad(self.status.joint_state['position'])
        # current_end_position_mm = self.status.pose['position']
        # current_end_rotation_deg = self.status.pose['orientation']
        # current_end_position_m = np.array(current_end_position_mm) / 1000.0
        # current_end_rotation_rad = np.deg2rad(current_end_rotation_deg)
        #
        # # 회전 행렬 생성
        # current_R = pin.rpy.rpyToMatrix(current_end_rotation_rad[0],
        #                                 current_end_rotation_rad[1],
        #                                 current_end_rotation_rad[2])
        # # SE3 객체 생성
        # current_robot_pose = pin.SE3(current_R, current_end_position_m)

        joint_vel = self.ik_solver.clik_delta(current_joint_rad, delta_pose)
        print(joint_vel)
        self.robot.joint_ctrl_velocity(joint_vel)


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
