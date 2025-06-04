import asyncio
from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.status_manager import StatusManager


class MobileController:
    def __init__(self, robot_info, **params):
        self.robot_id = robot_info['name']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        if 'use_bridge' in params and params['use_bridge']:
            self.bridge = True
        else:
            self.bridge = False

        if 'sub_controller' in params:
            self.sub_controller = params['sub_controller']
        else:
            self.sub_controller = self.init_sub_controller(**params)

        if hasattr(self.sub_controller, 'init_base_controller'):
            self.sub_controller.init_base_controller(self)

        # self.sub_controller = self.init_sub_controller(self.robot_model, **params)
        self.status_manager = StatusManager(robot_info, self.sub_controller)
        pub.subscribe(self.receive_message,"receive_message")

    def init_sub_controller(self, robot_model, **params):
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

    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == 'change_mode':
                            self.sub_controller.change_mode(value)
                        elif topic == 'move_waypoint':
                            self.sub_controller.move_waypoint(value)
                        elif topic == 'move_coord':
                            self.sub_controller.move_coord(value)
                        elif topic == 'dock':
                            self.sub_controller.dock(value)
                        elif topic == 'joy':
                            self.sub_controller.joy(value)
                        elif topic == 'relocate':
                            self.sub_controller.relocate(value)
                        elif topic == 'move_floor':
                            self.sub_controller.move_floor(value)
                        elif topic == 'map_change':
                            self.sub_controller.map_change(value)
                        elif topic == 'get_robot_status':
                            pub.sendMessage('send_message', message=self.sub_controller.update_status())
                        elif topic == 'connect_robot':
                            self.connect()
                        elif topic == 'custom_command':
                            self.sub_controller.custom_command(value)
                        else:
                            print(f"⚠️ Unknown topic received: {topic}")
                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")



    def connect(self):
        self.sub_controller.connect()

    def disconnect(self):
        self.sub_controller.disconnect()
