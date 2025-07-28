from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

class ManipulatorFunction:
    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            option = message.get('option')
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == 'joint_ctrl':
                            self.sub_controller.joint_ctrl(value,option)
                        elif topic == 'joint_ctrl_step':
                            self.sub_controller.joint_ctrl_step(value,option)
                        elif topic == 'gripper_ctrl':
                            self.sub_controller.gripper_ctrl(value,option)
                        elif topic == 'joint_ctrl_master':
                            self.sub_controller.joint_ctrl_puppet(value,option)
                        elif topic == 'gripper_ctrl_master':
                            self.sub_controller.gripper_ctrl_puppet(value,option)
                        elif topic == 'joint_preset':
                            if value in self.sub_controller.joint_preset:
                                self.sub_controller.joint_ctrl(self.sub_controller.joint_preset[value])
                            else:
                                print(f"⚠️ Unknown joint_preset value: {value}")
                    except AttributeError as e:
                        print(f"❌ Command '{topic}' not supported by controller: {e}")
                    except Exception as e:
                        print(f"❌ Error while handling topic '{topic}': {e}")