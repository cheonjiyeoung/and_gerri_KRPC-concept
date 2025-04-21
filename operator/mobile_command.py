"""
[Command 구조 및 사용 지침]
- `topic`과 `value`는 모든 로봇에 통용될 수 있는 값을 선호
- 추가적인 값들은 `option` 파라미터로 처리
- 모든 함수는 목적에 맞게 변경 가능
- `option`에 추가된 파라미터는 로봇 쪽에서도 대응 동작을 설정해야 함
"""

def move(pan_tilt_angle, target=None, **option):
    """
    지정 각도로 pan_tilt 를 설정하는 함수.

    Parameters:
    - pan_tilt_angle (list): [pan, tilt] 각도 (degree)
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'pan_tilt',
        'value': pan_tilt_angle
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command

def pan_tilt_step(pan_tilt_angle_step, target=None, **option):
    """
    현재 각도에서 지정 step 만큼 pan_tilt 를 설정하는 함수.

    Parameters:
    - pan_tilt_angle_step (list): 현재 각도에서 추가할 [pan, tilt] 각도 (degree)
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'pan_tilt_step',
        'value': pan_tilt_angle_step
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command
