
from pubsub import pub

import time


import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

class PiperSubCommander:
    def __init__(self, master_arm = None, **kwargs):
        self.base_commander = None

        if master_arm:
            self.use_master_arm = True
            self.master_arm_info  = master_arm
            self.control_target = self.master_arm_info['target']
            self.joint_limit = self.master_arm_info['joint_limit']
        else:
            self.use_master_arm = False
            self.master_arm_info  = None
            self.control_target = None
            self.joint_limit = None

        self.control_target = 'all'
        self.master_control = False
        self.master_arm = None

        pub.subscribe(self.key_mouse_control, 'key_mouse_control')
        pub.subscribe(self.ui_signal, 'ui_signal')

    """
    Initializes the connection for the sub-function (e.g., hardware setup, activation).
    """
    def connect(self):
        if self.use_master_arm:
            if self.master_arm_info['master_model'] == 'piper':
                from gerri.robot.examples.piper.piper_sub_controller import PiperSubController
                self.master_arm = PiperSubController(self.master_arm_info['id'])

            elif self.master_arm_info['master_model'] == 'dynamixel':
                from gerri.operator.interface.master_arm.master_arm import MasterArm
                self.master_arm = MasterArm(n_dxls=self.master_arm_info['n_dxl'],
                                            port=self.master_arm_info['port'],
                                            baudrate=self.master_arm_info['baudrate'])
            self.master_arm.initialize()


    """
    Handles cleanup and shutdown for the sub-function (if applicable).
    """
    def disconnect(self):
        pass
        ### TODO : ADD DISCONNECT FUNCTION

    def enable_master_arm(self):
        self.master_arm.initialize()
        time.sleep(1)
        self.master_control = True

    def disable_master_arm(self):
        self.master_control = False


    def get_status(self, value):
        pass

    def ui_signal(self,signal):
        vx=0
        vy=0
        vth=0
        if signal == "clicked_W":
            vx=1
        if signal == "clicked_A":
            vy=-1
        if signal == "clicked_S":
            vx=-1
        if signal == "clicked_D":
            vy=1
        if signal == "clicked_Q":
            vth=-1
        if signal == "clicked_E":
            vth=1
        value = {"vx":vx,"vy":vy,"vth":vth}
        self.base_commander.hello_universe("hello")

    """
    Clamp a given value to a specified min-max range.
    Optionally constrain further with an absolute limit.
    """
    def clamp(self, value, min_value, max_value, absolute_limit=None):
        """
        값을 주어진 범위로 제한 (클램핑).

        :param value: 제한할 값
        :param min_value: 최소값
        :param max_value: 최대값
        :param absolute_limit: 절대 제한 (튜플, optional)
        :return: 제한된 값
        """
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))

    """
    Maps a value from one range into another using linear scaling.
    """
    def map_value(self, value, in_min, in_max, out_min, out_max):
        """
        특정 값을 주어진 범위 내에서 다른 범위로 매핑하는 함수.

        :param value: 매핑할 값
        :param in_min: 매핑 전 최소값
        :param in_max: 매핑 전 최대값
        :param out_min: 매핑 후 최소값
        :param out_max: 매핑 후 최대값
        :return: 매핑된 값
        """
        map_value = self.clamp((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max)

        return map_value

    """
    Handles key and mouse input events and maps them to robot commands.
    """

    def key_mouse_control(self, command):
        # print(command)
        key = command['key_control']
        mouse_d_move = command['mouse_d_move']
        mouse_d_wheel = command['mouse_d_wheel']
        mouse_click = command['mouse_click']


        if "RETURN" in key:
            self.base_commander.get_robot_status(target=self.control_target)

        if "TAB" in key:
            pass

        if "SHIFT" in key:
            if mouse_d_move[0] or mouse_d_move[1] or mouse_d_wheel[0]:
                self.base_commander.joint_ctrl_step(joint_angle_step=[-mouse_d_move[0] * 20, mouse_d_wheel[0] * 20, -mouse_d_wheel[0] * 20, 0, mouse_d_move[1] * 20, 0], target=self.control_target)
            if mouse_click[0]:
                 self.base_commander.gripper_ctrl(gripper_width=0, target=self.control_target)
            if mouse_click[2]:
                self.base_commander.gripper_ctrl(gripper_width=100000000, target=self.control_target)

        if "F10" in key:
            self.base_commander.connect_robot(target=self.control_target)

        if "F4" in key:
            self.base_commander.set_master_joint(target=self.control_target)

        if "F5" in key:
            if self.use_master_arm:
                self.enable_master_arm()

        if "F6" in key:
            if self.use_master_arm:
                self.disable_master_arm()

        if "F1" in key:
            self.base_commander.joint_preset(preset_name='home', target=self.control_target)
        if "F2" in key:
            self.base_commander.joint_preset(preset_name='ready', target=self.control_target)

        if 'CONTROL' in key:
            if '1' in key:
                self.base_commander.joint_preset(preset_name='1', target=self.control_target)

            if '2' in key:
                self.base_commander.joint_preset(preset_name='2', target=self.control_target)

            if '3' in key:
                self.base_commander.joint_preset(preset_name='3', target=self.control_target)

            if '4' in key:
                self.base_commander.joint_preset(preset_name='4', target=self.control_target)

            if '5' in key:
                self.base_commander.joint_preset(preset_name='5', target=self.control_target)

            if '6' in key:
                self.base_commander.joint_preset(preset_name='6', target=self.control_target)

        if "ALT" in key:
            self.base_commander.pan_tilt_step(pan_tilt_angle_step=mouse_d_move)

        if self.master_control:
            joint_value = self.master_arm.get_joint_angles()
            if self.check_joint_safety_limits(joint_value):
                if self.master_arm_info['master_model'] == 'dynamixel':
                    self.base_commander.joint_ctrl_master(joint_value[:6], target=self.control_target)
                    self.base_commander.gripper_ctrl_master(master_gripper_width=joint_value[6], target=self.control_target)
                elif self.master_arm_info['master_model'] == 'piper':
                    self.base_commander.joint_ctrl(joint_value[:6], target=self.control_target)
                    self.base_commander.gripper_ctrl(gripper_width=joint_value[6], target=self.control_target)

    def check_joint_safety_limits(self, joint_value):
        """
        Check if each joint value is within its safety limit.

        :param joint_value: List of joint values (deg or rad)
        :return: True if all values are within limits, False otherwise
        """
        if self.joint_limit:
            for idx, value in enumerate(joint_value):
                min_limit, max_limit = self.master_arm_info['joint_limit'][idx]
                if value < min_limit or value > max_limit:
                    print(
                        f"[SAFETY] Joint {idx + 1} limit exceeded: Value={value:.2f}, Limit=[{min_limit}, {max_limit}]"
                    )
                    return False
        return True