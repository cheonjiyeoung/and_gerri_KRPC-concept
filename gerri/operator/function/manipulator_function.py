# manifulator_commander.py
from pubsub import pub

import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.function import manipulator_command


class ManipulatorFunction:
    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def joint_ctrl(self, joint_angles, option = None, target='all'):
        command = manipulator_command.joint_ctrl(joint_angles, option=option, target=target)
        self.send_message(command)

    def arm_joint_ctrl(self, joint_angle, option = None):
        command = manipulator_command.joint_ctrl(joint_angle, option=option, target='all')
        self.send_message(command)

    def joint_ctrl_step(self, joint_steps, option = None, target='all'):
        command = manipulator_command.joint_ctrl_step(joint_steps, option=option, target=target)
        self.send_message(command)

    def joint_ctrl_master(self, master_joint_angle, option = None, target='all'):
        command = manipulator_command.joint_ctrl_master(master_joint_angle, option=option, target=target)
        self.send_message(command)

    def gripper_ctrl(self, gripper_angle, option = None, target='all'):
        command = manipulator_command.gripper_ctrl(gripper_angle, option=option, target=target)
        self.send_message(command)

    def get_robot_status(self, option = None, target='all'):
        command = manipulator_command.get_robot_status(option=option, target=target)
        self.send_message(command)

    def connect_robot(self, option = None, target='all'):
        command = manipulator_command.connect_robot(option=option, target=target)
        self.send_message(command)

    def set_master_joint(self, option = None, target='all'):
        command = manipulator_command.set_master_joint(option=option, target=target)
        self.send_message(command)