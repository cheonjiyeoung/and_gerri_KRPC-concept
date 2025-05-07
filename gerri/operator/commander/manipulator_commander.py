from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.commander import manipulator_command

class ManipulatorCommander:
    def __init__(self, robot_info, **params):
        self.robot_name = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        if 'sub_commander' in params:
            self.sub_commander = params['sub_commander']
        else:
            self.sub_commander = self.init_sub_commander(**params)

        if hasattr(self.sub_commander, 'init_base_commander'):
            self.sub_commander.init_base_commander(self)

        pub.subscribe(self.receive_message, 'received_message')

    def init_sub_commander(self, **params):
        if self.robot_model == 'piper':
            from gerri.operator.examples.piper.piper_commander import PiperCommander
            return PiperCommander(self)
        else:
            raise ValueError(f"Unsupported robot model: {self.robot_model}")

    def connect(self):
        self.sub_commander.connect()

    def disconnect(self):
        self.sub_commander.disconnect()

    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                target = message['target']

    def joint_ctrl(self, joint_angles, target='all'):
        command = manipulator_command.joint_ctrl(joint_angles, target=target)
        self.send_message(command)

    def joint_ctrl_step(self, joint_steps, target='all'):
        command = manipulator_command.joint_ctrl_step(joint_steps, target=target)
        self.send_message(command)

    def joint_ctrl_master(self, master_joint_angle, target='all'):
        command = manipulator_command.joint_ctrl_master(master_joint_angle, target=target)
        self.send_message(command)

    def gripper_ctrl(self, gripper_value, target='all'):
        command = manipulator_command.gripper_ctrl(gripper_value, target=target)
        self.send_message(command)

    def get_robot_status(self, target='all'):
        command = manipulator_command.get_robot_status(target=target)
        self.send_message(command)

    def connect_robot(self, target='all'):
        command = manipulator_command.connect_robot(target=target)
        self.send_message(command)

    def set_master_joint(self, target='all'):
        command = manipulator_command.set_master_joint(target=target)
        self.send_message(command)
