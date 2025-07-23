import asyncio
from pubsub import pub

import os, sys

from pubsub.pub import sendMessage

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.status_manager import StatusManager

class ConstructionBaseController:
    def __init__(self, robot_info, **params):
        self.robot_id = robot_info['id']
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

        self.sub_controller = self.init_sub_controller(**params)
        self.status_manager = StatusManager(robot_info, self.sub_controller)
        pub.subscribe(self.receive_message,"receive_message")
        self.master_start = False

    def init_sub_controller(self, **params):
        """
        로봇 모델에 따라 적절한 컨트롤러를 초기화.

        :param robot_model: 로봇 모델
        :param kwargs: 추가적인 파라미터
        :return: 특정 모델 컨트롤러 인스턴스
        """                                     
        if self.robot_model == 'doosanM1509':
            from gerri.robot.examples.construction_robot.construction_sub_controller import ConstructionSubController
            return ConstructionSubController()
        else:
            raise ValueError(f"Unsupported robot model: {self.robot_model}")

    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] == 'left_arm':
                    try:
                        if topic == 'hello_universe':
                            self.sub_controller.hello_universe(value)
                        if topic == 'joint_ctrl':
                                self.sub_controller.left_joint_ctrl(value)
                        if topic == 'moveL':
                                self.sub_controller.left_movel(value)
                        if topic == 'stop_robot':
                            self.sub_controller.left_stop_robot(value)
                        if topic == 'joint_ctrl_master':
                            self.sub_controller.left_joint_ctrl_master(value)
                        if topic == 'gripper_ctrl':
                            self.sub_controller.left_gripper_ctrl(value)
                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")

                if message['target'] == 'right_arm':
                    try:
                        if topic == 'hello_universe':
                            self.sub_controller.hello_universe(value)
                        if topic == 'joint_ctrl':
                            self.sub_controller.right_joint_ctrl(value)
                        if topic == 'moveL':
                            self.sub_controller.right_movel(value)
                        if topic == 'stop_robot':
                            self.sub_controller.right_stop_robot(value)
                        if topic == 'joint_ctrl_master':
                            self.sub_controller.right_joint_ctrl_master(value)
                        if topic == 'gripper_ctrl':
                            self.sub_controller.right_gripper_ctrl(value)
                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                    # except Exception as e:
                    #     print(f"❌ Error processing topic '{topic}': {e}")


    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def connect(self):
        self.sub_controller.connect()

    def disconnect(self):
        self.sub_controller.disconnect()
