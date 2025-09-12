import pinocchio as pin
import numpy as np


class IKSolver:
    """
    Pinocchio를 이용해 CLIK(Closed-loop Inverse Kinematics) 기반의
    관절 속도를 계산하는 범용 솔버.
    """

    def __init__(self, urdf_path, end_effector_frame_name):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        if not self.model.existFrame(end_effector_frame_name):
            raise ValueError(f"URDF에 '{end_effector_frame_name}' 프레임이 존재하지 않습니다.")
        self.frame_id = self.model.getFrameId(end_effector_frame_name)

        # 제어 파라미터
        self.Kp = 20.0
        self.damping = 1e-4

    def clik(self, q: np.ndarray, target_pose: pin.SE3) -> np.ndarray:
        """
        [기본 함수] 현재 관절 각도(q)와 '절대적인 목표 자세(target_pose)'로부터
        필요한 관절 속도(dq)를 계산합니다.
        """
        # 1. 순기구학으로 현재 자세 계산
        current_pose = self.fk(q)
        # 2. 오차 계산
        error = pin.log6(current_pose.inverse() * target_pose).vector

        # 3. 자코비안 계산
        J = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id)

        # 4. 감쇠 최소 자승법으로 dq 계산
        J_T = J.T
        lambda_sq = self.damping * self.damping
        inv_term = np.linalg.inv(J @ J_T + lambda_sq * np.identity(6))
        dq = J_T @ inv_term @ (self.Kp * error)

        return dq

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