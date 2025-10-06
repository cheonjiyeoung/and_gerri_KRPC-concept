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


def convert_by_frame_alignment(delta_vr: pin.SE3) -> pin.SE3:
    """
    Spot 컨트롤러의 좌표계 변환 규칙('기저 변환')을 Pinocchio로 구현합니다.
    이것이 Spot의 코드와 100% 동일하게 동작하는 최종 버전입니다.
    """
    # 1. 변환 규칙을 담은 '번역기' SE3 객체
    T_robot_vr = pin.SE3(pin.Quaternion(-0.5, -0.5, 0.5, 0.5), np.zeros(3))  # Pinocchio: x,y,z,w

    # 2. 기저 변환 공식 적용
    delta_robot = T_robot_vr * delta_vr * T_robot_vr.inverse()

    return delta_robot


def convert_lh_to_rh_physical(pose_lh: pin.SE3) -> pin.SE3:
    R_lh_to_rh = np.array([[-1, 0, 0],
                           [0, 0, 1],
                           [0, -1, 0]])
    return pin.SE3(R_lh_to_rh, np.zeros(3)) * pose_lh


# tf_helper.py 파일의 함수를 아래 최종 버전으로 교체하세요.

def vr_ee_converter(delta_vr: pin.SE3) -> pin.SE3:
    """
    VR 모션을 로봇 EE 모션으로 변환하는 최종 분리 제어(decoupled) 함수.
    - 위치: 고정된 행렬로 변환
    - 회전: VR의 rx, ry, rz를 분해하여 로봇의 각 축에 재조립.
    """
    # --- 1. 위치 변환 (기존과 동일) ---
    R_pos_fixed = np.array([[-1, 0, 0],
                            [0, 0, 1],
                            [0, -1, 0]])
    p_final = R_pos_fixed @ delta_vr.translation

    # --- 2. 회전 변환 (분리 제어 방식) ---
    # (a) VR 컨트롤러의 회전 변화량을 [rx, ry, rz] 각도로 분해합니다.
    rpy_vr = pin.rpy.matrixToRpy(delta_vr.rotation)

    # (b) ✨✨ 축 매핑: VR의 각 축 회전을 로봇의 어떤 축 회전으로 쓸지 결정합니다. ✨✨
    # 예시: VR의 rx(0) -> 로봇의 rx, VR의 ry(1) -> 로봇의 ry, VR의 rz(2) -> 로봇의 rz
    # 이 순서를 바꾸면 조작계를 완전히 커스터마이징할 수 있습니다.
    ee_rx = rpy_vr[0]  # 로봇의 Roll은 VR의 Roll 값을 사용
    ee_ry = rpy_vr[1]  # 로봇의 Pitch는 VR의 Pitch 값을 사용
    ee_rz = rpy_vr[2]  # 로봇의 Yaw는 VR의 Yaw 값을 사용

    # (c) 위에서 매핑한 각도들을 기준으로 로봇 EE의 최종 회전 행렬을 '재조립'합니다.
    R_final = pin.rpy.rpyToMatrix(ee_rx, ee_ry, ee_rz)

    # --- 3. 최종 결과 조합 ---
    return pin.SE3(R_final, p_final)