import os, sys, time
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g


class VisualizerController:
    def __init__(self):
        # ... (초기화 부분은 동일) ...
        print("Initializing VisualizerController (Simulation Mode)...")

        PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../.."))
        URDF_PATH = os.path.join(PROJECT_ROOT, 'gerri', 'robot', 'examples', 'construction_vr', 'm1509_urdf',
                                 'm1509.urdf')

        self.model = pin.buildModelFromUrdf(URDF_PATH)
        self.data = self.model.createData()

        self.joint_state = {'position': [-90., 0., 90., 0., -45., 0.]}
        self.pose = {'position': [0, 0, 0], 'orientation': [0, 0, 0]}

        try:
            # ... (뷰어 초기화 부분) ...
            mesh_dir = os.path.dirname(URDF_PATH)
            visual_model = pin.buildGeomFromUrdf(self.model, URDF_PATH, pin.GeometryType.VISUAL, package_dirs=mesh_dir)
            self.viz = MeshcatVisualizer(self.model, pin.GeometryModel(), visual_model)
            self.viz.initViewer(open=True)
            self.viz.loadViewerModel(rootNodeName="doosan_robot")

            # ... (베이스 오프셋 적용 부분은 동일) ...
            yaw_rad = np.deg2rad(-90)
            pitch_rad = np.deg2rad(45)
            R_yaw = pin.rpy.rpyToMatrix(0, 0, yaw_rad)
            R_pitch = pin.rpy.rpyToMatrix(0, pitch_rad, 0)
            R_world_base = R_pitch @ R_yaw
            self.T_world_base = pin.SE3(R_world_base, np.zeros(3))
            self.viz.viewer["doosan_robot"].set_transform(self.T_world_base.homogeneous)
            print("✅ Robot base offset Yaw(-90) -> Pitch(45) applied.")

            ### --- BUG FIX: 엔드 이펙터 좌표계 '부착' --- ###
            # 1. 로봇 모델 내부의 올바른 주소 지정
            ee_frame_path = "doosan_robot/joint_6"
            # 2. 해당 주소에 triad(좌표계) 모양을 '추가'
            self.viz.viewer[ee_frame_path].set_object(g.triad(0.1))
            print(f"✅ End-effector frame attached to '{ee_frame_path}'.")

            # 초기 자세 표시
            self.viz.display(self.get_q_rad())

            # --- 정적 프레임 그리기 ---
            self._draw_static_frames()

        except Exception as e:
            self.viz = None
            print(f"❌ VisualizerController: 뷰어 초기화 실패: {e}")

    ### --- BUG FIX: 함수 역할 분리 및 수정 --- ###
    def _draw_static_frames(self):
        """월드, 베이스 등 움직이지 않는 좌표계만 그립니다."""
        if not self.viz: return
        self.viz.viewer["world"].set_object(g.triad(0.2))
        self.viz.viewer["base"].set_object(g.triad(0.15))
        self.viz.viewer["base"].set_transform(self.T_world_base.homogeneous)

    # _update_frames 함수는 이제 필요 없으므로 삭제하거나 아래 함수로 대체
    def _update_frames(self, q=None):
        """(이전 버전 호환용) 이제 이 함수는 아무것도 하지 않습니다."""
        pass  # 엔드 이펙터 프레임은 viz.display()에 의해 자동으로 업데이트됩니다.

    def get_q_rad(self):
        return np.deg2rad(self.joint_state['position'])

    def connect(self):
        print("✅ VisualizerController Connected.")

    def update_status(self):
        """가상 로봇의 상태를 FK를 이용해 계산하여 업데이트합니다."""
        q = self.get_q_rad()
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        ee_frame_id = self.model.getFrameId('joint_6')
        # T_base_ee가 아닌, 월드 기준의 T_world_ee를 계산해야 합니다.
        # oMf는 월드 기준의 프레임 자세를 제공합니다.
        ee_pose_world = self.data.oMf[ee_frame_id]

        self.pose['position'] = ee_pose_world.translation.tolist()
        rpy_rad = pin.rpy.matrixToRpy(ee_pose_world.rotation)
        self.pose['orientation'] = np.rad2deg(rpy_rad).tolist()

    def joint_ctrl(self, value, vel=120, acc=120):
        """가상 로봇을 특정 관절 각도로 즉시 이동"""
        self.joint_state['position'] = value
        q_rad = self.get_q_rad()
        self.viz.display(q_rad)  # 이 한 줄이 로봇 모델과 부착된 ee 좌표계를 모두 업데이트합니다.

    def joint_ctrl_vel(self, value, acc=250, dt=0.01):
        """전달받은 관절 속도(dq)로 가상 로봇의 상태를 업데이트하고 뷰어에 표시"""
        if not self.viz: return

        dq_rad = np.array(value)
        q_current_rad = self.get_q_rad()
        q_next_rad = pin.integrate(self.model, q_current_rad, dq_rad * dt)
        self.joint_state['position'] = np.rad2deg(q_next_rad).tolist()

        self.viz.display(q_next_rad)  # 이 한 줄이 로봇 모델과 부착된 ee 좌표계를 모두 업데이트합니다.
        time.sleep(dt)

    def joint_ctrl_vel_stop(self):
        pass