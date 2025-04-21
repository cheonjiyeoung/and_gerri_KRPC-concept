import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from avatar_darm.robot.robot_tools.status_manager import StatusManager
from pubsub import pub
import asyncio


class MobileController:
    def __init__(self, robot_name, robot_model, **params):
        self.robot_name = robot_name
        self.robot_category = 'mobile'
        self.robot_model = robot_model

        self.controller = self._initialize_robot(robot_model, **params)
        self.status_manager = StatusManager(self.robot_name, self.robot_category, self.robot_model, self.controller)
        pub.subscribe(self.message_handler,"receive_message")

    def _initialize_robot(self, robot_model, **params):
        """
        로봇 모델에 따라 적절한 컨트롤러를 초기화.

        :param robot_model: 로봇 모델
        :param kwargs: 추가적인 파라미터
        :return: 특정 모델 컨트롤러 인스턴스
        """                                     
        if robot_model == 'gyd_mobile':
            from avatar_darm.robot.robot_tools.gyd.gyd_controller import GydMobileController
            return GydMobileController(port="/dev/ttyUSB0",baudrate=115200,timeout=1)
        else:
            raise ValueError(f"Unsupported robot model: {robot_model}")

    def message_handler(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_name or message['target'] == 'all':
                    if topic == 'change_mode':
                        self.controller.change_mode(value)
                    elif topic == 'move_waypoint':
                        self.controller.move_waypoint(value)
                    elif topic == 'move_coord':
                        self.controller.move_coord(value)
                    elif topic == 'dock':
                        self.controller.dock(value)
                    elif topic == 'joy':
                        self.controller.joy(value)
                    elif topic == 'relocate':
                        self.controller.reloc_absolute(value)
                    elif topic == 'move_floor':
                        self.controller.move_floor(value)
                    elif topic == 'map_change':
                        self.controller.map_change(value)
            else:
                print('command not defined')


    def connect(self):
        self.controller.connect()

    def disconnect(self):
        self.controller.disconnect()
