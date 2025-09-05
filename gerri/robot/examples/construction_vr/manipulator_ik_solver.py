import pinocchio as pin
import numpy as np


class ManipulatorIKSolver:
    """
    Pinocchio를 이용해 CLIK(Closed-loop Inverse Kinematics) 기반의
    관절 속도를 계산하는 범용 솔버.
    """

    def __init__(self, robot_model, end_effector_frame_name):
        self.model = robot_model
        self.data = self.model.createData()

        if not self.model.existFrame(end_effector_frame_name):
            raise ValueError(f"URDF에 '{end_effector_frame_name}' 프레임이 존재하지 않습니다.")
        self.frame_id = self.model.getFrameId(end_effector_frame_name)

        # 제어 파라미터
        self.Kp = 5.0
        self.damping = 1e-4

    def compute_joint_velocities(self, q: np.ndarray, target_pose: pin.SE3) -> np.ndarray:
        """
        현재 관절 각도(q)와 목표 자세(target_pose)로부터
        필요한 관절 속도(dq)를 계산합니다.
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.frame_id)

        current_pose = self.data.oMf[self.frame_id]
        error = pin.log6(current_pose.inverse() * target_pose).vector

        J = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id)

        # 감쇠 최소 자승법 (Damped Least-Squares)
        J_T = J.T
        lambda_sq = self.damping * self.damping
        inv_term = np.linalg.inv(J @ J_T + lambda_sq * np.identity(6))

        dq = J_T @ inv_term @ (self.Kp * error)

        return dq