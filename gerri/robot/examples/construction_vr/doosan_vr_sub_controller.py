import os, sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.construction_vr.robot_status import RobotStatus
import threading
import random
import time
import pinocchio as pin


from gerri.robot.function.tf_helper import *
from gerri.robot.function.ik_solver import IKSolver

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../.."))

# 프로젝트 루트를 기준으로 URDF 파일의 전체 경로를 조합합니다.
URDF_PATH = os.path.join(PROJECT_ROOT, 'gerri/robot/examples/construction_vr/m1509_urdf/m1509.urdf')

JOINT_LIMIT_MIN = np.deg2rad([-360, -150, -135, -360, -135, -360])
JOINT_LIMIT_MAX = np.deg2rad([360, 150, 135, 360, 135, 360])

CONTROL_INTERVAL = 0.05

class DoosanVRSubController:
    def __init__(self, ip, port, joint_limit_degree=None, control_interval=CONTROL_INTERVAL, debug=False):
        self.base_controller = None
        if joint_limit_degree:
            self.joint_limit = np.deg2rad(joint_limit_degree)
        else:
            self.joint_limit = [JOINT_LIMIT_MIN, JOINT_LIMIT_MAX]

        self.control_interval = control_interval


        self._lock = threading.Lock()
        self.debug = debug

        self.default_ref = 'base'

        self.T_world_base = tf_from_offset_zyz_deg(a_z_deg=0, b_y_deg=45, c_z_deg=-90)
        self.T_correction = tf_from_offset_zyz_deg(c_z_deg=-90)


        # --- URDF 및 IK 솔버 초기화 (공통) ---
        PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../.."))
        URDF_PATH = os.path.join(PROJECT_ROOT, 'gerri', 'robot', 'examples', 'construction_vr', 'm1509_urdf',
                                 'm1509.urdf')
        self.ik_solver = IKSolver(URDF_PATH, 'joint_6')

        self.joint_preset = {'home': [-90.00, 0.00, 90.00, 0.00, -45.00, 0.00]}

        # --- debug 플래그에 따라 모드 분기 ---
        if self.debug:
            try:
                from pinocchio.visualize import MeshcatVisualizer
                import meshcat.geometry as g
                MESHCAT_AVAILABLE = True
            except ImportError:
                MESHCAT_AVAILABLE = False
            # --- 디버그 (시각화) 모드 ---
            if not MESHCAT_AVAILABLE:
                raise ImportError("디버그 모드를 위해 'meshcat' 라이브러리를 설치해주세요.")

            # 가상 로봇의 상태
            from gerri.robot.examples.construction_vr.doosan_debug_controller import VisualizerController

            self.robot = VisualizerController()
            self.q_current = np.deg2rad(self.joint_preset['home'])
            self.status = RobotStatus(robot_id="virtual_robot", model="virtual_model", category="manipulator")
            self.status.joint_state['position'] = self.joint_preset['home']

        else:
            from gerri.robot.examples.construction_vr.doosan_controller import DoosanController
            self.robot = DoosanController(ip, port)
            self.status = None  # connect()에서 생성됨
            self.viz = None

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

    def joint_ctrl(self, value):
        """
        ROBOT JOINT CONTROL BY DEGREE
        """
        self.robot.joint_ctrl(value)

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

    def end_pose_ctrl(self, target_pose, vel=100, acc=200, dt=0.1):
        """
        최종 목표 자세(SE3)를 받아 로봇을 제어합니다.
        """
        # pin.SE3 객체를 로봇이 이해하는 [x,y,z,rx,ry,rz] 리스트로 변환
        pose_list = se3_to_pose(target_pose, pos_unit='mm', rot_unit='deg')
        # 로봇 드라이버에 명령 전송
        self.robot.end_pose_ctrl(pose_list, vel, acc, dt)

    def end_pose_ctrl_delta(self, start_pose, delta_pose, vel=100, acc=100, dt=0.1):
        target_pose = start_pose * delta_pose
        # 변환 함수를 호출하여 [x,y,z,rx,ry,rz] 리스트를 얻음
        # 로봇이 mm 단위를 쓴다면 pos_unit='mm' 추가
        pose_list = se3_to_pose(target_pose, pos_unit='mm', rot_unit='deg')
        # print(f"Current pose: {self.status.pose}")
        # print(f"Target Pose List: {pose_list}")
        self.robot.end_pose_ctrl(pose_list)

    def joint_ctrl_vel(self, target_pose: pin.SE3, acc=250, dt=0.05, tolerance=0):
        """
        최종 목표 자세(target_pose)를 받아 해당 지점으로 이동하기 위한
        관절 속도를 계산하고 로봇을 제어합니다.
        """
        # 1. 현재 관절 각도 가져오기
        current_q_rad = np.deg2rad(self.status.joint_state['position'])

        # 2. IK 솔버를 이용해 관절 속도(dq) 계산
        dq = self.ik_solver.clik(current_q_rad, target_pose, tolerance)

        # 만약 dq가 0 벡터라면 (목표 도달)
        if np.linalg.norm(dq) < 1e-5: # dq가 거의 0이면
            return

        q_next = pin.integrate(self.ik_solver.model, current_q_rad, dq * self.control_interval)
        # q_next = pin.integrate(self.ik_solver.model, current_q_rad, dq * self.control_interval)

        # 1. 각 안전 함수는 이제 위험 여부(True/False)만 반환합니다.
        is_limit_exceeded = self.check_joint_limits(q_next)
        is_collision = self.check_self_collision(q_next)

        # 2. 위험이 하나라도 감지되면 '정지', 모두 안전하면 '원본 dq'를 전송합니다.
        if is_collision or is_limit_exceeded:
            if is_collision:
                print("🚨 충돌 감지!")
            if is_limit_exceeded:
                print("🚨 관절 범위 초과!")
            # self.robot.joint_ctrl_vel(dq, acc, dt)

        else:
            # 안전하므로, IK가 계산한 원본 dq를 그대로 사용합니다.
            self.robot.joint_ctrl_vel(dq, acc, dt)


    def check_joint_limits(self, q_next):
        """
        예상 관절 각도가 제한을 초과하는지 확인하고,
        초과하는 경우 해당 관절의 속도를 0으로 만드는 안전 함수.
        """
        for i in range(self.ik_solver.model.nv):
            if not (JOINT_LIMIT_MIN[i] < q_next[i] < JOINT_LIMIT_MAX[i]):
                return True  # 하나라도 초과하면 즉시 True 반환
        return False # 모든 관절이 안전하면 False 반환

    def check_self_collision(self, q: np.ndarray) -> bool:
        """
        [신규 함수] Pinocchio를 이용해 자기 충돌 여부를 확인하는 안전 함수.
        :param q: 확인할 관절 각도 (radian)
        :return: 충돌 시 True, 아니면 False
        """
        # 1. 충돌 모델의 지오메트리 위치를 q 자세에 맞게 업데이트
        pin.updateGeometryPlacements(self.ik_solver.model, self.ik_solver.data,
                                     self.ik_solver.collision_model, self.ik_solver.collision_data, q)

        # 2. 모든 지오메트리 쌍에 대해 충돌 여부 계산
        #    computeCollisions의 첫 번째 인자가 True이면 충돌 즉시 계산 중단 (빠름)
        is_collision = pin.computeCollisions(self.ik_solver.collision_model, self.ik_solver.collision_data, True)

        return is_collision

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

        # 4. 실제 로봇에 속도 명령 전달
        self.robot.joint_ctrl_vel(dq, acc, dt)


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
