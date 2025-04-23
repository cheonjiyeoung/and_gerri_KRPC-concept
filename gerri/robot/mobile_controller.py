import asyncio
from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.status_manager import StatusManager


class MobileController:
    def __init__(self, robot_info, **params):
        self.robot_name = robot_info['name']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        if 'use_bridge' in params and params['use_bridge']:
            self.bridge = True
        else:
            self.bridge = False

        self.controller = self._initialize_robot(self.robot_model, **params)
        self.status_manager = StatusManager(robot_info, self.controller)
        pub.subscribe(self.message_handler,"receive_message")

    def _initialize_robot(self, robot_model, **params):
        """
        로봇 모델에 따라 적절한 컨트롤러를 초기화.

        :param robot_model: 로봇 모델
        :param kwargs: 추가적인 파라미터
        :return: 특정 모델 컨트롤러 인스턴스
        """                                     
        if robot_model == 'gyd_mobile':
            from gerri.robot.examples.gyd_mobile.gyd_mobile_controller import GydMobileController
            return GydMobileController(port="/dev/ttyUSB0",baudrate=115200,timeout=1)
        else:
            raise ValueError(f"Unsupported robot model: {robot_model}")

    def message_handler(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_name or message['target'] == 'all':
                    try:
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
                            self.controller.relocate(value)
                        elif topic == 'move_floor':
                            self.controller.move_floor(value)
                        elif topic == 'map_change':
                            self.controller.map_change(value)
                        elif topic == 'get_robot_status':
                            pub.sendMessage('send_message', message=self.controller.update_status())
                        elif topic == 'connect_robot':
                            self.connect()
                        else:
                            print(f"⚠️ Unknown topic received: {topic}")
                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")



    def connect(self):
        self.controller.connect()

    def disconnect(self):
        self.controller.disconnect()
