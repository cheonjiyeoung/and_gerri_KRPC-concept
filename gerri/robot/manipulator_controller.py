from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.status_manager import StatusManager


class ManipulatorController:
    def __init__(self, robot_info, **params):
        self.robot_name = robot_info['name']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        if 'use_bridge' in params and params['use_bridge']:
            self.bridge = True
        else:
            self.bridge = False
        self.controller = self._initialize_robot(**params)
        self.status_manager = StatusManager(robot_info, self.controller)

        pub.subscribe(self.message_handler, 'receive_message')

    def _initialize_robot(self, **params):
        """
        로봇 모델에 따라 적절한 컨트롤러를 초기화.

        :param robot_model: 로봇 모델
        :param kwargs: 추가적인 파라미터
        :return: 특정 모델 컨트롤러 인스턴스
        """
        if self.bridge:
            from gerri.robot.examples.robot_bridge.robot_bridge import RobotBridge
            return RobotBridge(self.robot_name)
        if self.robot_model == 'piper':
            from gerri.robot.examples.piper.piper_controller import PiperController
            return PiperController(can_port=self.robot_name)
        elif self.robot_model == 'dummy':
            from gerri.robot.examples.sample_robot.sample_robot_controller import SampleRobotController
            return SampleRobotController()
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
                        try:
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
                                if value in self.controller.joint_preset:
                                    self.controller.joint_ctrl(self.controller.joint_preset[value])
                                else:
                                    print(f"⚠️ Unknown joint_preset value: {value}")
                            elif topic == 'connect_robot':
                                self.connect()
                            elif topic == 'get_robot_status':
                                pub.sendMessage('send_message', message=self.controller.update_status())
                            else:
                                print(f"⚠️ Unsupported topic: {topic}")
                        except AttributeError as e:
                            print(f"❌ Command '{topic}' not supported by controller: {e}")
                        except Exception as e:
                            print(f"❌ Error while handling topic '{topic}': {e}")


    def send_message(self, topic, value, target='all'):
        pub.sendMessage('send_message', message={'topic': topic, 'value': value, 'target': target})

    def connect(self):
        self.controller.connect()

    def disconnect(self):
        self.controller.disconnect()
