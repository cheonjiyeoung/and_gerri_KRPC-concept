import sys
import os
from pubsub import pub
from gerri.robot.status_manager import StatusManager

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

class ManipulatorController:
    def __init__(self, robot_name, robot_model, **params):
        self.robot_name = robot_name

        self.robot_category = 'manipulator'
        self.robot_model = robot_model

        if 'use_bridge' in params and params['use_bridge']:
            self.bridge = True
        else:
            self.bridge = False

        self.controller = self._initialize_robot(**params)
        self.status_manager = StatusManager(self.robot_name, self.robot_category, self.robot_model, self.controller)

        pub.subscribe(self.message_handler, 'receive_message')

    def _initialize_robot(self, **params):
        """
        로봇 모델에 따라 적절한 컨트롤러를 초기화.

        :param robot_model: 로봇 모델
        :param kwargs: 추가적인 파라미터
        :return: 특정 모델 컨트롤러 인스턴스
        """
        if self.bridge:
            from robot_bridge.robot_bridge import RobotBridge
            return RobotBridge(self.robot_name)
        if self.robot_model == 'piper':
            from piper.piper_controller import PiperController
            return PiperController(can_port=self.robot_name)
        else:
            raise ValueError(f"Unsupported robot model: {self.robot_model}")

    def message_handler(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_name or message['target'] == 'all':
                    if self.bridge:
                        pub.sendMessage('send_message_bridge', message=message)
                    else:
                        if topic == 'joint_ctrl':
                            self.controller.joint_ctrl(value)
                        elif topic == 'joint_ctrl_step':
                            self.controller.joint_ctrl_step(value)
                        elif topic == 'gripper_ctrl':
                            self.controller.gripper_ctrl(value)
                        elif topic == 'joint_ctrl_master':
                            self.controller.joint_ctrl_puppet(value)
                        elif topic == 'gripper_ctrl_master':
                            self.controller.gripper_ctrl_puppet(value)
                        elif topic == 'joint_preset':
                            if value in self.controller.joint_preset.keys():
                                self.controller.joint_ctrl(self.controller.joint_preset[value])
                        elif topic == 'connect_robot':
                            self.connect()
                        elif topic == 'get_robot_status':
                            pub.sendMessage('send_message', message=self.controller.update_status())


    def connect(self):
        self.controller.connect()

    def disconnect(self):
        self.controller.disconnect()
