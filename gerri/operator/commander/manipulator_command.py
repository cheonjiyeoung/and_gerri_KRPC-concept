"""
[Command 구조 및 사용 지침]
- `topic`과 `value`는 모든 로봇에 통용될 수 있는 값을 선호
- 추가적인 값들은 `option` 파라미터로 처리
- 모든 함수는 목적에 맞게 변경 가능
- `option`에 추가된 파라미터는 로봇 쪽에서도 대응 동작을 설정해야 함
"""

def joint_ctrl(joint_angle, target=None, **option):
    """
    각 조인트별 지정 각도로 로봇을 이동하는 함수.

    Parameters:
    - joint_angle (list): 각 조인트별 목표 각도 (degree)
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'joint_ctrl',
        'value': joint_angle
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def joint_ctrl_step(joint_angle_step, target=None, **option):
    """
    각 조인트별로 지정된 변화량만큼 이동하는 함수.

    Parameters:
    - joint_angle_step (list): 각 조인트별 이동할 step 크기 (degree)
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'joint_ctrl_step',
        'value': joint_angle_step
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def joint_ctrl_master(master_joint_angle, target=None, **option):
    """
    Master arm의 각도를 기반으로 로봇을 제어하는 함수.

    Parameters:
    - master_joint_angle (list): Master arm의 조인트 각도 (degree)
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'joint_ctrl_master',
        'value': master_joint_angle
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def joint_preset(preset_name, target=None, **option):
    """
    미리 정의된 프리셋 각도로 이동하는 함수.

    Parameters:
    - preset_name (str): 사용할 프리셋 이름
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'joint_preset',
        'value': preset_name
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def gripper_ctrl(gripper_width, target=None, **option):
    """
    그리퍼를 제어하는 함수.

    Parameters:
    - gripper_width (float): 그리퍼 너비 (상대적 비율)
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: force, duration 등)


    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'gripper_ctrl',
        'value': gripper_width
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def gripper_ctrl_master(master_gripper_width, target=None, **option):
    """
    master 로봇을 통해 그리퍼를 제어하는 함수.

    Parameters:
    - gripper_width (float): 그리퍼 너비 (상대적 비율)
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: force, duration 등)


    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'gripper_ctrl_master',
        'value': master_gripper_width
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def connect_robot(target=None, **option):
    """
    로봇에 연결하는 함수.

    Parameters:
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: force, duration 등)


    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'connect_robot',
        'value': None
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def get_robot_status(target=None, **option):
    """
    로봇의 상태를 가져오는 함수.

    Parameters:
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: force, duration 등)


    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'get_robot_status',
        'value': None
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command

def set_master_joint(joint_angle=None, target=None, **option):
    """
    로봇의 master joint를 세팅하는 함수.

    Parameters:
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: force, duration 등)


    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'set_master_joint',
        'value': None
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command