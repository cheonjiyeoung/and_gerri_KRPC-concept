import numpy as np
import pinocchio as pin


def tf_from_axis_map(params):
    """axis_map으로부터 회전 행렬 생성"""
    # ... (내부 로직은 동일) ...
    axis_map_dict = {'x': [1,0,0], 'y': [0,1,0], 'z': [0,0,1],
                     '-x': [-1,0,0], '-y': [0,-1,0], '-z': [0,0,-1]}
    new_x, new_y, new_z = (np.array(axis_map_dict[p]) for p in params)
    R = np.array([new_x, new_y, new_z]).T
    return pin.SE3(R, np.zeros(3))

def tf_from_rpy_deg(params):
    """rpy 각도(degree)로부터 회전 행렬 생성"""
    # ... (내부 로직은 동일) ...
    rpy_rad = np.deg2rad(params)
    R = pin.rpy.rpyToMatrix(rpy_rad[0], rpy_rad[1], rpy_rad[2])
    return pin.SE3(R, np.zeros(3))

def tf_from_offset_rpy_deg(params):
    """위치 오프셋(m)과 rpy 각도(degree)로부터 변환 행렬 생성"""
    # ... (내부 로직은 동일) ...
    offset = np.array(params.get('offset', [0,0,0]))
    rpy_deg = params.get('rpy', [0,0,0])
    rpy_rad = np.deg2rad(rpy_deg)
    R = pin.rpy.rpyToMatrix(rpy_rad[0], rpy_rad[1], rpy_rad[2])
    return pin.SE3(R, offset)


def se3_to_pose(pose: pin.SE3, pos_unit='m', rot_unit='deg'):
    """pin.SE3 객체를 [x, y, z, rx, ry, rz] 리스트로 변환합니다."""

    # 1. 위치 추출 및 단위 변환
    position = pose.translation
    if pos_unit == 'mm':
        position = position * 1000.0

    # 2. 회전 추출 및 단위 변환
    rotation_matrix = pose.rotation
    rpy = pin.rpy.matrixToRpy(rotation_matrix)  # 기본 단위는 라디안
    if rot_unit == 'deg':
        rpy = np.rad2deg(rpy)

    # 3. 두 리스트를 합쳐서 반환
    return np.concatenate([position, rpy])

def tf_compose(se3_list: list) -> pin.SE3:
    """SE3 리스트를 순서대로 곱하여 최종 변환 행렬을 반환합니다."""
    final_transform = pin.SE3.Identity()
    for T_step in se3_list:
        final_transform = T_step * final_transform
    return final_transform

def convert_lh_to_rh(pose_lh: pin.SE3) -> pin.SE3:
    """
    Z축을 뒤집어 왼손 좌표계(LH) SE3를 오른손 좌표계(RH) SE3로 변환합니다.
    """
    # 1. Z축을 뒤집는 반전 행렬 정의
    S = np.diag([1, 1, -1])

    # 2. 왼손 좌표계의 위치와 회전 행렬 추출
    p_lh = pose_lh.translation
    R_lh = pose_lh.rotation

    # 3. 오른손 좌표계의 위치와 회전 행렬 계산
    p_rh = np.array([p_lh[0], p_lh[1], -p_lh[2]])
    R_rh = S @ R_lh @ S

    # 4. 오른손 좌표계 SE3 객체를 새로 만들어 반환
    return pin.SE3(R_rh, p_rh)