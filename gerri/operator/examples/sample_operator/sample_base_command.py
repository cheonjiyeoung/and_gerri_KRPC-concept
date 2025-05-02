"""
[Command 구조 및 사용 지침]
- `topic`과 `value`는 모든 로봇에 통용될 수 있는 값을 선호
- 추가적인 값들은 `option` 파라미터로 처리
- 모든 함수는 목적에 맞게 변경 가능
- `option`에 추가된 파라미터는 로봇 쪽에서도 대응 동작을 설정해야 함
"""


def hello_universe(message, target=None, **option):
    """
    Args:
    message (str): Message to send with the command
    target (str, optional): Specific robot target ID
    **option: Additional optional parameters Hello Universe!
    target:
    **option:

    Returns:
    dict: Formatted command dictionary with topic, value, and optional target/option
    """

    command = {
        'topic': 'hello_universe',
        'value': message
    }
    if target:
        command['target'] = target
    if option:
        command['option'] = option
    return command
