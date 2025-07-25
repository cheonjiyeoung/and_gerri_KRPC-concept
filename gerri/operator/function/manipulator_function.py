# manifulator_commander.py
from pubsub import pub

import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.function import manipulator_command


class ManipulatorCommander:
    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def joint_ctrl(self, joint_angles, target='all'):
        command = manipulator_command.joint_ctrl(joint_angles, target=target)
        self.send_message(command)

    def arm_joint_ctrl(self, joint_angle, option):
        command = manipulator_command.joint_ctrl(joint_angle, target='all', option=option)
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