import pinocchio as pin
import numpy as np
import os


class IKSolver:
    """
    Pinocchio를 이용해 CLIK(Closed-loop Inverse Kinematics) 기반의
    관절 속도를 계산하는 범용 솔버.
    """

    def __init__(self, urdf_path, end_effector_frame_name, max_joint_vel_deg=180.0):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        mesh_dir = os.path.dirname(urdf_path)

        try:
            self.collision_model = pin.buildGeomFromUrdf(self.model, urdf_path,
                                                         pin.GeometryType.COLLISION,
                                                         package_dirs=[mesh_dir])
            self.collision_data = self.collision_model.createData()

            print(f"✅ 충돌 모델 로드 성공. (Geometries: {len(self.collision_model.geometryObjects)})")
        except ValueError as e:
            print(f"❌ 충돌 모델 로드 실패: {e}. URDF 파일 내 <collision> 태그의 경로를 확인하세요.")
            self.collision_model = None
            self.collision_data = None


        if not self.model.existFrame(end_effector_frame_name):
            raise ValueError(f"URDF에 '{end_effector_frame_name}' 프레임이 존재하지 않습니다.")
        self.frame_id = self.model.getFrameId(end_effector_frame_name)

        # 제어 파라미터
        self.Kp = 30.0       # 비례 이득. 목표에 얼마나 빨리 도달할지 결정. 너무 크면 진동 발생.
        self.damping = 1e-4  # 감쇠 계수. 특이점(singularity) 근처에서 발산을 막는 핵심 안정화 파라미터.
        self.MAX_JOINT_VELOCITY = np.deg2rad(max_joint_vel_deg)
        print(f">> IKSolver 초기화: Kp={self.Kp}, Damping={self.damping}, MaxVel={max_joint_vel_deg} deg/s")

    def clik(self, q: np.ndarray, target_pose: pin.SE3, tolerance=0):
        """
        [기본 함수] 현재 관절 각도(q)와 '절대적인 목표 자세(target_pose)'로부터
        필요한 관절 속도(dq)를 계산합니다.
        """
        current_pose = self.fk(q)
        distance_to_goal = np.linalg.norm(current_pose.translation - target_pose.translation)

        if distance_to_goal <= tolerance / 1000.0:
            return np.zeros(self.model.nv), 0.0  # (dq, manipulability) 형태에 맞게 반환

        # --- 자코비안 계산 (단 한 번!) ---
        J = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id)

        # --- 1. 조작성(Manipulability) 계산 ---
        JJ_T = J @ J.T
        determinant = np.linalg.det(JJ_T)
        manipulability = np.sqrt(determinant) if determinant > 1e-12 else 0.0

        # --- 2. 관절 속도(dq) 계산 ---
        error = pin.log6(current_pose.inverse() * target_pose).vector
        J_T = J.T
        lambda_sq = self.damping * self.damping
        inv_term = np.linalg.inv(J @ J_T + lambda_sq * np.identity(6))
        dq = J_T @ inv_term @ (self.Kp * error)

        dq_magnitude = np.linalg.norm(dq)
        if dq_magnitude > self.MAX_JOINT_VELOCITY:
            dq = dq * (self.MAX_JOINT_VELOCITY / dq_magnitude)

        # --- 3. 두 가지 결과 함께 반환 ---
        return dq, manipulability

    def clik_delta(self, q: np.ndarray, delta_pose: pin.SE3) -> np.ndarray:
        """
        [헬퍼 함수] 현재 관절 각도(q)를 기준으로 '상대적인 움직임(delta_pose)'만큼
        이동하기 위한 관절 속도(dq)를 계산합니다.
        """
        # 1. 순기구학으로 현재 자세 계산
        current_pose = self.fk(q)

        # 2. 다음 목표 자세 계산
        next_target_pose = current_pose * delta_pose

        # 2. 오차 계산
        error = pin.log6(current_pose.inverse() * next_target_pose).vector

        # 3. 자코비안 계산
        J = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id)

        # 4. 감쇠 최소 자승법으로 dq 계산
        J_T = J.T
        lambda_sq = self.damping * self.damping
        inv_term = np.linalg.inv(J @ J_T + lambda_sq * np.identity(6))
        dq = J_T @ inv_term @ (self.Kp * error)

        return dq


    def fk(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.frame_id)
        return self.data.oMf[self.frame_id].copy()