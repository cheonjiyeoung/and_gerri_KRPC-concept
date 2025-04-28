# manipulator_commander.py (대분류, 명령 실행 + 소분류 커맨더 관리)

from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.commander import manipulator_command

class ManipulatorCommander:
    def __init__(self, robot_info):
        self.robot_name = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.command = manipulator_command
        self.sub_commander = self._initialize_commander()

        pub.subscribe(self.received_message, 'received_message')

    def _initialize_commander(self):
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

    def received_message(self, message):
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


# piper_commander.py (소분류, 입력 해석 및 base 호출)

class PiperCommander:
    def __init__(self, base_commander):
        self.base = base_commander
        self.control_target = self.base.control_target

    def key_mouse_control(self, command):
        key = command['key_control']
        mouse_d_move = command['mouse_d_move']
        mouse_d_wheel = command['mouse_d_wheel']
        mouse_click = command['mouse_click']

        if "RETURN" in key:
            self.base.request_status()

        if "SHIFT" in key:
            self.handle_shift_movement(mouse_d_move, mouse_d_wheel, mouse_click)

        if "F1" in key:
            self.base.move_joint([0, 0, 0, 0, 0, 0])

        if "F2" in key:
            self.base.move_joint([0, 30, -30, 0, 0, 0])

    def handle_shift_movement(self, mouse_d_move, mouse_d_wheel, mouse_click):
        step = [-mouse_d_move[0] * 20, mouse_d_wheel[0] * 20, -mouse_d_wheel[0] * 20, 0, mouse_d_move[1] * 20, 0]
        self.base.move_joint_step(step)

        if mouse_click[0]:
            self.base.move_gripper(0)
        if mouse_click[2]:
            self.base.move_gripper(100000000)
