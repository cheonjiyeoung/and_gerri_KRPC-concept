import inspect

def _is_callable(method, value):
    params = inspect.signature(method).parameters
    for param, attr in params.items():
        if attr.default is inspect.Parameter.empty and param not in value:
            return False
    return True

class RobotSubController:
    def __init__(self, robot_controller=None):
        self.robot_controller = robot_controller

    def command_handler(self, command):
        # topic = 메서드명
        # value = 파라미터 딕셔너리 형태로 전달
        # ex) def move(vx=0.0, vy=0.0, vth=0.0) -> {'topic':'move', 'value':{"vx": 1.0, "vy":0.0, "vth":0.5}}
        topic = command.get("topic")
        value = command.get("value")

        # 해당 topic에 맞는 메서드 호출
        method = getattr(self.robot_controller, topic, None)
        if callable(method):
            # 파라미터 확인
            params = inspect.signature(method).parameters

            # 파라미터가 없으면 그냥 호출
            if len(params) == 0:
                method()
                return
            
            # 파라미터 유효성 검사
            else:
                if _is_callable(method, value):
                    method(**value)
                else:
                    print(f"Error: Missing required parameters for method '{topic}'\n Expected parameters: {list(params.keys())}, Provided parameters: {list(value.keys())}")



if __name__ == "__main__":
    rc = RobotController()

    command1 = {"topic": "move", "value": {"vx": 5.0, "vy": 0.5}}
    rc.command_handler(command1)

    command2 = {"topic": "move_waypoint", "value": {}}
    rc.command_handler(command2)

    command3 = {"topic": "test_method"}
    rc.command_handler(command3)