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
    return np.round(np.concatenate([position, rpy]))

def tf_compose(se3_list: list) -> pin.SE3:
    """SE3 리스트를 순서대로 곱하여 최종 변환 행렬을 반환합니다."""
    final_transform = pin.SE3.Identity()
    for T_step in se3_list:
        final_transform = T_step * final_transform
    return final_transform


def vr_ee_converter(delta_vr: pin.SE3, scale_factor = 1) -> pin.SE3:
    """
    VR 모션을 로봇 EE 모션으로 변환하는 최종 분리 제어(decoupled) 함수.
    - 위치: 고정된 행렬로 변환
    - 회전: VR의 rx, ry, rz를 분해하여 로봇의 각 축에 재조립.
    """
    # --- 1. 위치 변환  ---
    # vr to ee => x to -x, y to z, z to -y
    R_pos_fixed = np.array([[-1, 0, 0],  # +x to -x
                            [0, 0, 1],   # +y to +z
                            [0, -1, 0]]) # +z to -y
    p_final = R_pos_fixed @ delta_vr.translation * scale_factor

    # --- 2. 회전 변환 (분리 제어 방식) ---
    # (a) VR 컨트롤러의 회전 변화량을 [rx, ry, rz] 각도로 분해합니다.
    rpy_vr = pin.rpy.matrixToRpy(delta_vr.rotation)

    # (b) 축 매핑: VR의 각 축 회전을 로봇의 어떤 축 회전으로 쓸지 결정합니다.
    # 예시: VR의 rx(0) -> 로봇의 rx, VR의 ry(1) -> 로봇의 ry, VR의 rz(2) -> 로봇의 rz
    # 이 순서를 바꾸면 조작계를 완전히 커스터마이징할 수 있습니다.
    # 회전 방향은 반대이므로 - 를 항상 포함하고 계산
    ee_rx = rpy_vr[0] * scale_factor  # 로봇의 Roll은 VR의 Roll 값을 사용
    ee_ry = -rpy_vr[2] * scale_factor  # 로봇의 Pitch는 VR의 Pitch 값을 사용
    ee_rz = rpy_vr[1] * scale_factor  # 로봇의 Yaw는 VR의 Yaw 값을 사용

    # (c) 위에서 매핑한 각도들을 기준으로 로봇 EE의 최종 회전 행렬을 '재조립'합니다.
    R_final = pin.rpy.rpyToMatrix(ee_rx, ee_ry, ee_rz)

    # --- 3. 최종 결과 조합 ---
    return pin.SE3(R_final, p_final)


def tf_from_offset_zyx_deg(x=0., y=0., z=0., roll=0., pitch=0., yaw=0.) -> pin.SE3:
    """
    위치(offset)와 ZYX 오일러 각도(degree)로부터 SE3 변환(TF)을 생성합니다.
    - 회전 순서: 고정 축 기준 Z -> Y -> X 순서로 적용
    - 행렬 곱셈 순서: Rx @ Ry @ Rz

    :param x: x위치 오프셋
    :param y: y위치 오프셋
    :param z: z위치 오프셋
    :param roll: X축 기준 회전각 (Roll)
    :param pitch: Y축 기준 회전각 (Pitch)
    :param yaw: Z축 기준 회전각 (Yaw)
    :return: pin.SE3 객체
    """
    # 1. 위치(translation) 벡터를 numpy 배열로 변환
    p_final = np.array([x, y, z])

    # 2. 모든 각도를 라디안(radian)으로 변환
    roll_rad = np.deg2rad(roll)
    pitch_rad = np.deg2rad(pitch)
    yaw_rad = np.deg2rad(yaw)

    # 3. 각 축에 대한 순수 회전 행렬을 생성
    Rx = pin.rpy.rpyToMatrix(roll_rad, 0, 0)
    Ry = pin.rpy.rpyToMatrix(0, pitch_rad, 0)
    Rz = pin.rpy.rpyToMatrix(0, 0, yaw_rad)

    # 4. Z -> Y -> X 순서에 맞게 Rx @ Ry @ Rz 로 곱합니다.
    R_final = Rx @ Ry @ Rz

    # 5. 최종 SE3 객체 반환
    return pin.SE3(R_final, p_final)


def tf_from_offset_zyz_deg(x=0., y=0., z=0., a_z_deg=0., b_y_deg=0., c_z_deg=0.) -> pin.SE3:
    """
    두산 로봇 표준(Z-Y-Z 오일러 각)에 따라 SE3 변환(TF)을 생성하는 최종 범용 함수입니다.
    위치를 x, y, z 개별 인자로 받습니다.

    :param x: x축 위치 오프셋
    :param y: y축 위치 오프셋
    :param z: z축 위치 오프셋
    :param a_z_deg: 첫 번째 Z축 회전 각도 (A)
    :param b_y_deg: 새로운 Y'축 기준 회전 각도 (B)
    :param c_z_deg: 가장 새로운 Z''축 기준 회전 각도 (C)
    :return: pin.SE3 객체
    """
    # 1. 위치(translation) 벡터 설정
    p_final = np.array([x, y, z])

    # 2. 모든 각도를 라디안(radian)으로 변환
    a_rad = np.deg2rad(a_z_deg)
    b_rad = np.deg2rad(b_y_deg)
    c_rad = np.deg2rad(c_z_deg)

    # 3. Z-Y-Z 각 축에 대한 순수 회전 행렬 생성
    Rz_A = pin.rpy.rpyToMatrix(0, 0, a_rad)
    Ry_B = pin.rpy.rpyToMatrix(0, b_rad, 0)
    Rz_C = pin.rpy.rpyToMatrix(0, 0, c_rad)

    # 4. Z-Y-Z 내재적(Intrinsic) 회전 순서에 따라 행렬 곱셈 수행
    # R_final = Rz(A) @ Ry(B) @ Rz(C)
    R_final = Rz_A @ Ry_B @ Rz_C

    # 5. 최종 SE3 객체 반환
    return pin.SE3(R_final, p_final)