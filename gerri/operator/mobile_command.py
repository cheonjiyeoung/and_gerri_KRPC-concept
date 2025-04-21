"""
[Command 구조 및 사용 지침]
- `topic`과 `value`는 모든 로봇에 통용될 수 있는 값을 선호
- 추가적인 값들은 `option` 파라미터로 처리
- 모든 함수는 목적에 맞게 변경 가능
- `option`에 추가된 파라미터는 로봇 쪽에서도 대응 동작을 설정해야 함
"""

def move_waypoint(waypoint, target=None, **option):
    """
    move to waypoint

    Parameters:
    - waypoint (str): waypoint name
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'move_waypoint',
        'value': waypoint
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command

def move_coord(coord, target=None, **option):
    """
    move to coord

    Parameters:
    - coord (list): [x, y, theta] coord
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'move_coord',
        'value': coord
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command

def dock(dock, target=None, **option):
    """
    docking to docking station.

    Parameters:
    - dock (str): docking station name
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)
      // gyde mobile need type of docking

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    print(f"**option = {option}")
    command = {
        'topic': 'dock',
        'value': dock
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command

def joy(joy, target=None, **option):
    """
    move to joy(cmd_vel).
    
    Parameters:
    - joy (list): joy params ex)vx,vth
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'joy',
        'value': joy
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command


def change_mode(mode, target=None, **option):
    """
    move to joy(cmd_vel).
    
    Parameters:
    - joy (list): joy params ex)vx,vth
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)

    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'change_mode',
        'value': mode
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command

def change_map(name, target=None, **option):
    command = {
        'topic': 'map_change',
        'value': name
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command

def relocate(waypoint, target=None, **option):
    """
    robot pose relocate.

    Parameters:
    - waypoint (str): docking station name
    - target (str, optional): 특정 로봇을 지정 (기본값: None)
    - option (dict, optional): 추가 설정값 (예: speed, acceleration 등)
    Returns:
    - dict: 로봇 명령을 포함한 command 객체
    """
    command = {
        'topic': 'relocate',
        'value': waypoint
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command

