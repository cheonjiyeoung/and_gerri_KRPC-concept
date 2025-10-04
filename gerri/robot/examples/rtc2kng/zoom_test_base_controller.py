from pubsub import pub
import copy

import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.status_manager import StatusManager


class ZoomBaseController:
    def __init__(self, robot_info, **params):
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        self.zoom_factor = 1.0

        if 'use_bridge' in params and params['use_bridge']:
            self.bridge = True
        else:
            self.bridge = False

        if 'sub_controller' in params:
            self.sub_controller = params['sub_controller']
        else:
            self.sub_controller = self.init_sub_controller(**params)

        if hasattr(self.sub_controller, 'init_base_controller'):
            self.sub_controller.init_base_controller(base_controller=self)

        self.status_manager = StatusManager(robot_info, self.sub_controller)



        if 'interface' in params:
            self.interface = params['interface']
            self.last_interface_value = copy.deepcopy(self.interface)
        else:
            self.interface = None
            self.last_interface_value = None

        pub.subscribe(self.receive_message,"receive_message")
        pub.subscribe(self.set_zoom_level, "zoom_level")


    def init_sub_controller(self, **params):
        """
        로봇 모델에 따라 적절한 컨트롤러를 초기화.

        :param robot_model: 로봇 모델
        :param kwargs: 추가적인 파라미터
        :return: 특정 모델 컨트롤러 인스턴스
        """                                     
        if self.robot_model == 'gerri':
            from gerri.robot.examples.sample_robot.sample_sub_controller import SampleSubController
            return SampleSubController()
        else:
            raise ValueError(f"Unsupported robot model: {self.robot_model}")

    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == self.interface.name:
                            self.interface.update(value)
                            if self.interface.button_left_menu :
                                pub.sendMessage('zoom_step_control', step=self.interface.right_axis_Y)

                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")

    def set_zoom_level(self, level):
        self.zoom_level = level
        print("zoom_level:", self.zoom_level)


    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def connect(self):
        self.sub_controller.connect()

    def disconnect(self):
        self.sub_controller.disconnect()
