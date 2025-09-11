import numpy as np
import pinocchio as pin

class ManipulatorTFHelper:
    """
    좌표 변환(TF)을 생성하고 합성하는 유틸리티 클래스.
    """
    def __init__(self):
        # 자주 사용하는 변환들을 미리 생성하여 프리셋으로 관리
        self.tf_preset = {
            'vr_to_robot': self.tf_from_axis_map(['-y', '-x', 'z']),
            'vr_world': self.tf_from_axis_map(['z', 'x', '-y']),
            'gcdual_base_right': self.tf_from_rpy_deg([0, 45, -90])
        }

    @staticmethod
    def tf_from_axis_map(params):
        """axis_map으로부터 회전 행렬 생성"""
        # ... (내부 로직은 동일) ...
        axis_map_dict = {'x': [1,0,0], 'y': [0,1,0], 'z': [0,0,1],
                         '-x': [-1,0,0], '-y': [0,-1,0], '-z': [0,0,-1]}
        new_x, new_y, new_z = (np.array(axis_map_dict[p]) for p in params)
        R = np.array([new_x, new_y, new_z]).T
        return pin.SE3(R, np.zeros(3))

    @staticmethod
    def tf_from_rpy_deg(params):
        """rpy 각도(degree)로부터 회전 행렬 생성"""
        # ... (내부 로직은 동일) ...
        rpy_rad = np.deg2rad(params)
        R = pin.rpy.rpyToMatrix(rpy_rad[0], rpy_rad[1], rpy_rad[2])
        return pin.SE3(R, np.zeros(3))

    @staticmethod
    def tf_from_offset_rpy_deg(params):
        """위치 오프셋(m)과 rpy 각도(degree)로부터 변환 행렬 생성"""
        # ... (내부 로직은 동일) ...
        offset = np.array(params.get('offset', [0,0,0]))
        rpy_deg = params.get('rpy', [0,0,0])
        rpy_rad = np.deg2rad(rpy_deg)
        R = pin.rpy.rpyToMatrix(rpy_rad[0], rpy_rad[1], rpy_rad[2])
        return pin.SE3(R, offset)

    @staticmethod
    def compose(se3_list: list) -> pin.SE3:
        """SE3 리스트를 순서대로 곱하여 최종 변환 행렬을 반환합니다."""
        final_transform = pin.SE3.Identity()
        for T_step in se3_list:
            final_transform = T_step * final_transform
        return final_transform