"""
[Command 구조 및 사용 지침]
- `topic`과 `value`는 모든 로봇에 통용될 수 있는 값을 선호
- 추가적인 값들은 `option` 파라미터로 처리
- 모든 함수는 목적에 맞게 변경 가능
- `option`에 추가된 파라미터는 로봇 쪽에서도 대응 동작을 설정해야 함
"""

################################################
# SPOT PTZ
def ptz_initialize(value=None, target=None, **option):
    command = {
        'topic': 'ptz_initialize',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command   

def ptz_relative(value=None, target=None, **option):
    command = {
        'topic': 'ptz_relative',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command   
    
def ptz_absolute(value=None, target=None, **option):
    command = {
        'topic': 'ptz_absolute',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command   
    
def focus(value=None, target=None, **option):
    command = {
        'topic': 'set_focus',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command   

def focus_relative(value=None, target=None, **option):
    command = {
        'topic': 'focus_relative',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command   
    
def set_compositor(value=None, target=None, **option):
    command = {
        'topic': 'set_compositor',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command   

#########################################################

#########################################################
# SPOT 공통
def toggle_power(value=None, target=None, **option):
    command = {
        'topic': 'toggle_power',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command   

def set_stance(value=None, target=None, **option):
    """value = {"yaw":yaw,"roll":roll,"pitch":pitch,"reset":bool}"""
    command = {
        'topic': 'set_stance',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command 

def sit(value=None, target=None, **option):
    command = {
        'topic': 'sit',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command 

def stand(value=None, target=None, **option):
    command = {
        'topic': 'stand',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command 

def action_stop(value=None, target=None, **option):
    command = {
        'topic': 'action_stop',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command 

def walk(value=None, target=None, **option):
    command = {
        'topic': 'walk',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command 

def run_mission(value=None, target=None, **option):
    command = {
        'topic': 'run_mission',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command    

def stop_mission(value=None, target=None, **option):
    command = {
        'topic': 'stop_mission',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command   

def pause_mission(value=None, target=None, **option):
    command = {
        'topic': 'pause_mission',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command  

def restart_mission(value=None, target=None, **option):
    command = {
        'topic': 'resume_mission',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command    

def dock(value=None, target=None, **option):
    command = {
        'topic': 'dock',
        'value': value
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command    



##########################################################################

##########################################################################
# SPOT Arm controll



