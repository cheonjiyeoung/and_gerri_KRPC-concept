import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import time
import meshcat.geometry as g
import os, sys

# --- 기존 모듈 Import ---
from tf_helper import tf_from_rpy_deg, tf_from_axis_map
# --- ✨ 개선점 1: IK Solver 추가 ---
from ik_solver import IKSolver

# --- 1. 설정 및 초기화 ---
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../.."))
URDF_PATH = os.path.join(PROJECT_ROOT, 'gerri', 'robot', 'examples', 'construction_vr', 'm1509_urdf', 'm1509.urdf')

# --- ✨ 개선점 2: 가독성을 위한 상수 정의 ---
EE_FRAME_NAME = 'joint_6'
DT = 0.02  # 시뮬레이션 시간 간격

try:
    mesh_dir = os.path.dirname(URDF_PATH)
    model = pin.buildModelFromUrdf(URDF_PATH)
    visual_model = pin.buildGeomFromUrdf(model, URDF_PATH, pin.GeometryType.VISUAL, package_dirs=mesh_dir)

    # --- ✨ 개선점 1 (계속): IK 솔버 인스턴스 생성 ---
    ik_solver = IKSolver(URDF_PATH, EE_FRAME_NAME)

    viz = MeshcatVisualizer(model, pin.GeometryModel(), visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel(rootNodeName="doosan_robot")
    print("✅ MeshCat 뷰어 및 IK 솔버 초기화 성공!")

except Exception as e:
    print(f"❌ 초기화 실패: {e}")
    exit()

# --- 2. 좌표계 정의 (기존과 동일) ---
T_world_base = tf_from_rpy_deg([0, 45, -90]).inverse()
T_world_vr = tf_from_axis_map(['-y', '-x', 'z']).inverse()
T_world_ctrl = tf_from_rpy_deg([0, 0, -90]).inverse()

# --- 3. 시뮬레이션을 위한 초기 상태 정의 ---
q_current = np.deg2rad([-90, 0, 90, 0, -45, 0])  # 현재 로봇 관절 상태 (계속 업데이트됨)
viz.display(q_current)  # 초기 자세 표시
time.sleep(1)  # 뷰어가 로드될 시간을 줌

pin.forwardKinematics(model, viz.data, q_current)
pin.updateFramePlacements(model, viz.data)
start_pose_base = viz.data.oMf[model.getFrameId(EE_FRAME_NAME)]

# --- 4. MeshCat에 좌표계 및 피드백 객체 추가 ---
viz.viewer["world"].set_object(g.triad(0.1))
viz.viewer["robot_base"].set_object(g.triad(0.3))
viz.viewer["robot_base"].set_transform(T_world_base.np)
viz.viewer["vr_origin"].set_object(g.triad(0.3))
viz.viewer["vr_origin"].set_transform(T_world_vr.np)
viz.viewer["vr_controller"].set_object(g.triad(0.2))

# 목표 지점 (빨간색 구)
viz.viewer["end_effector_target"].set_object(g.Sphere(0.03), g.MeshLambertMaterial(color=0xff0000, reflectivity=0.8))

# --- ✨ 개선점 3: 시각적 피드백 강화 ---
# 실제 로봇 EE 위치 (초록색 구)
viz.viewer["end_effector_actual"].set_object(g.Sphere(0.02), g.MeshLambertMaterial(color=0x00ff00, reflectivity=0.8))
# 목표와 실제 위치 사이의 에러를 표시할 선
viz.viewer["error_line"].set_object(g.Line(g.PointsGeometry(np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]).T), g.LineBasicMaterial(color=0x888888)))

# --- 5. 실시간 시뮬레이션 루프 ---
print("\n🚀 시뮬레이션을 시작합니다. 브라우저에서 확인하세요.")
t = 0.0
while True:
    try:
        # 가상의 VR 컨트롤러 움직임 (기존과 동일)
        dx = 0.1 * np.sin(t * 1.5)
        dy = 0.1 * np.cos(t * 1.5)
        dz = 0.05 * np.sin(t * 3)
        delta_pos = np.array([dx, dy, dz])
        angle = 0.5 * np.sin(t)
        delta_rot = pin.rpy.rpyToMatrix(0, 0, angle)
        delta_vr = pin.SE3(delta_rot, delta_pos)

        # BaseController의 목표 자세 계산 로직 (기존과 동일)
        delta_world = T_world_ctrl * delta_vr
        start_pose_world = T_world_base * start_pose_base
        target_pose_world = start_pose_world * delta_world
        target_pose_base = T_world_base.inverse() * target_pose_world

        # --- ✨ 개선점 1 (계속): IK를 이용해 로봇 움직임 계산 ---
        # 1. IK로 목표 지점까지 가기 위한 관절 속도(dq) 계산
        dq = ik_solver.clik(q_current, target_pose_base)

        # 2. 관절 속도를 적분하여 다음 스텝의 관절 각도(q) 계산
        q_current = pin.integrate(model, q_current, dq * DT)

        # --- ✨ 개선점 3 (계속): 시각화 업데이트 ---
        # 3. 계산된 관절 각도로 로봇 모델의 자세를 업데이트
        viz.display(q_current)

        # 4. 업데이트된 로봇의 실제 EE 위치를 계산하여 초록 구 업데이트
        pin.forwardKinematics(model, viz.data, q_current)
        pin.updateFramePlacements(model, viz.data)
        actual_pose_base = viz.data.oMf[model.getFrameId(EE_FRAME_NAME)]
        actual_pose_world = T_world_base * actual_pose_base
        viz.viewer["end_effector_actual"].set_transform(actual_pose_world.np)

        # 5. 목표(빨간 구)와 실제(초록 구) 사이에 선 그리기
        p_target = target_pose_world.translation
        p_actual = actual_pose_world.translation
        viz.viewer["error_line"].set_object(
            g.Line(g.PointsGeometry(np.array([p_target, p_actual]).T), g.LineBasicMaterial(color=0x888888)))

        # 나머지 객체들 업데이트
        controller_pose_world = T_world_vr * delta_vr
        viz.viewer["vr_controller"].set_transform(controller_pose_world.np)
        viz.viewer["end_effector_target"].set_transform(target_pose_world.np)

        time.sleep(DT)
        t += DT

    except KeyboardInterrupt:
        break
print("\n시뮬레이션을 종료합니다.")