import asyncio
from pubsub import pub

import os, sys

from gerri.robot.function.manipulator_function import ManipulatorFunction
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.status_manager import StatusManager


class ManipulatorVRBaseController:
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.sub_controller = None

        if 'interface' in params:
            self.interface = params['interface']
        else:
            self.interface = None

        pub.subscribe(self.receive_message,"receive_message")

    def connect(self):
        self.sub_controller.connect()
        self.status_manager = StatusManager(self.robot_info, self.sub_controller)

    def disconnect(self):
        self.sub_controller.disconnect()

    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def receive_message(self, message):
        ManipulatorFunction.receive_message(self,message=message)
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == self.interface.name:
                            self.interface.update(value)
                            # --- 팔 제어 로직 (오른쪽 그립) ---
                            if self.interface.button_right_grip:
                                print(self.interface.right_current_pose, self.interface.right_delta_pose)
                                # self.sub_controller.joint_ctrl_vel()

                                # 그리퍼 제어 (오른쪽 트리거)
                                gripper_value = round(1 - self.interface.right_trigger, 1)
                                self.sub_controller.gripper_ctrl(gripper_value)
                        elif topic == 'connect_robot':
                            self.connect()
                        elif topic == 'get_robot_status':
                            pub.sendMessage('send_message', message=self.sub_controller.update_status())
                        elif topic == 'custom_command':
                            self.sub_controller.custom_command(value)

                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                        pass
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")
                        pass
