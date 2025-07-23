import sys
import os
import time
import json
import numpy as np
import threading
import math

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.interface.master_arm.master_arm import MasterArm
from pubsub import pub


class DoosanMLeftCommander:
    def __init__(self):
        self.joint_names = ["left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6"]
        self.current_joint_position = [0.] * 6
        self.current_point = [0.] * 3
        self.current_angle = [0.] * 3

        self.target_posj = [0.] * 6
        self.target_velj = 0.
        self.target_accj = 0.

        self.target_point = [0.] * 3
        self.target_angle = [0.] * 3
        self.target_vell = 0.
        self.target_accl = 0.

        self.master_start = False
        self.readMaster = False
        self.gripper_active = False
        self.go_back_joint = [0.] * 6
        self.going_back = False

        self.dxls = None
        self.Master_System_thread = None

        self.key_set = set()
        self.command_dict = {}

        self.enable_keyboard = False
        self._lock = threading.Lock()

    def stop(self):
        pub.sendMessage('left_stop_robot', pause_type='SSTOP')

    def moveJ(self, j1, j2, j3, j4, j5, j6, vel, acc):
        self.target_posj[0] = j1
        self.target_posj[1] = j2
        self.target_posj[2] = j3
        self.target_posj[3] = j4
        self.target_posj[4] = j5
        self.target_posj[5] = j6
        self.target_velj = vel
        self.target_accj = acc
        pub.sendMessage('left_movej_message', joint_angle=self.target_posj, joint_velocity=self.target_velj, joint_acceleration=self.target_accj)

    def moveL(self, x, y, z, rx, ry, rz, vel, acc):
        self.target_point[0] = x
        self.target_point[1] = y
        self.target_point[2] = z
        self.target_angle[0] = rx
        self.target_angle[1] = ry
        self.target_angle[2] = rz
        self.target_vell = vel
        self.target_accl = acc
        pub.sendMessage('left_movel_message', point=self.target_point, angle=self.target_angle, velocity=self.target_vell, acceleration=self.target_accl)

    def master_read_start(self):
        if self.dxls is None:
            self.dxls = MasterArm(n_dxls=6, port='/dev/ttyUSB0')  # Initialize the MasterArm with the given number of joints
            self.dxls.updateDefaultPosCnt([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            time.sleep(1)
        else:
            pass
        if not self.master_start:
            self.readMaster = True
            self.master_start = True
            pub.sendMessage('left_set_master_joint', master_start=self.master_start)
            self.dxls.updateDefaultPosCnt(self.current_joint_position)
            time.sleep(0.3)
            self.Master_System_thread = threading.Thread(target=self.read_master_val)
            self.Master_System_thread.daemon = True
            self.Master_System_thread.start()
    
    def master_stop(self):
        self.master_start = False
        self.readMaster = False
        self.gripper_active = False
        pub.sendMessage('left_set_master_joint', master_start=self.master_start)
        self.stop()

    def read_master_val(self):
        while True:
            if self.master_start:
                joint_ctrl_master = self.dxls.get_position_deg()
                master_moving = [0.] * 6
                for i in range(5):
                    master_moving[i] = joint_ctrl_master[i]
                gripper_moving = joint_ctrl_master[5]
                if gripper_moving <= -45.:
                    if self.gripper_active == False:
                        self.activating_gripper()
                    else:
                        self.move_gripper()
                else:
                    if self.gripper_active == True:
                        if self.going_back == False:
                            self.stop()
                        else:
                            if self.go_back_complete() == True:
                                self.gripper_active = False
                                self.going_back = False
                                self.dxls.updateDefaultPosCnt(self.current_joint_position)
                                time.sleep(1)
                    else:
                        pub.sendMessage('left_joint_ctrl_master', joint_ctrl_master=joint_ctrl_master, joint_velocity=150, joint_acceleration=150, round=200)
            time.sleep(0.3)

    def activating_gripper(self):
        self.gripper_active = True
        self.stop()
        self.go_back_joint = self.current_joint_position
    
    def move_gripper(self):
        pub.sendMessage('left_moving_gripper', moving_offset=5.)

    def master_go_back(self):
        self.going_back = True
        pub.sendMessage('left_movej_message', joint_angle=self.go_back_joint, joint_velocity=20, joint_acceleration=20)

    def go_back_complete(self):
        return all(abs(x - y) < 1 for x, y in zip(self.go_back_joint, self.current_joint_position))

    ## Test Drill1
    def reload_1_btn(self):
        # print("reload1")
        self.target_posj[0] = 69.9
        self.target_posj[1] = 103.1
        self.target_posj[2] = -116.4
        self.target_posj[3] = 339.3
        self.target_posj[4] = -35.7
        self.target_posj[5] = 39.2
        self.target_velj = 10.
        self.target_accj = 10.
        print(self.target_posj)
        pub.sendMessage('left_movej_message', joint_angle=self.target_posj, joint_velocity=self.target_velj,
                        joint_acceleration=self.target_accj)

    ## Test Drill2
    def reload_2_btn(self):
        # print("reload2")
        self.target_posj[0] = 70.3
        self.target_posj[1] = 101.5
        self.target_posj[2] = -116.6
        self.target_posj[3] = 337.8
        self.target_posj[4] = -34.1
        self.target_posj[5] = 40.6
        self.target_velj = 10.
        self.target_accj = 10.
        print(self.target_posj)
        pub.sendMessage('left_movej_message', joint_angle=self.target_posj, joint_velocity=self.target_velj,
                        joint_acceleration=self.target_accj)

    ## Test Drill3
    def reload_3_btn(self):
        # print("reload3")
        self.target_posj[0] = 71.2
        self.target_posj[1] = 100.1
        self.target_posj[2] = -113.0
        self.target_posj[3] = 337.7
        self.target_posj[4] = -36.0
        self.target_posj[5] = 39.6
        self.target_velj = 10.
        self.target_accj = 10.
        print(self.target_posj)
        pub.sendMessage('left_movej_message', joint_angle=self.target_posj, joint_velocity=self.target_velj,
                        joint_acceleration=self.target_accj)

    def keyPressEvent(self, event):
        if self.enable_keyboard:
            self.handle_key_event(event.key(), pressed=True)

    def keyReleaseEvent(self, event):
        if self.enable_keyboard:
            self.handle_key_event(event.key(), pressed=False)

    def get_key_name(self, key):
        """Qt 키 코드를 문자로 변환합니다."""
        # 알파벳 키 처리 (A~Z)
        if Qt.Key.Key_A <= key <= Qt.Key.Key_Z:
            return chr(key)

        # 숫자 키 처리 (0~9)
        elif Qt.Key.Key_0 <= key <= Qt.Key.Key_9:
            return chr(key)

        # MAPPING FOR OTHER KEYS (MANUAL DEFINITION)
        key_map = {
            Qt.Key.Key_Space: 'SPACE',
            Qt.Key.Key_Up: 'UP ARROW',
            Qt.Key.Key_Down: 'DOWN ARROW',
            Qt.Key.Key_Left: 'LEFT ARROW',
            Qt.Key.Key_Right: 'RIGHT ARROW',
            Qt.Key.Key_Enter: 'ENTER',
            Qt.Key.Key_Return: 'RETURN',
            Qt.Key.Key_Escape: 'ESC',
            Qt.Key.Key_Control: 'CONTROL',
            Qt.Key.Key_Shift: 'SHIFT',
            Qt.Key.Key_Alt: 'ALT',
            Qt.Key.Key_Tab: 'TAB',
            Qt.Key.Key_Backspace: 'BACKSPACE',
            Qt.Key.Key_Delete: 'DELETE',
            Qt.Key.Key_F1: 'F1',
            Qt.Key.Key_F2: 'F2',
            Qt.Key.Key_F3: 'F3',
            Qt.Key.Key_F4: 'F4',
            Qt.Key.Key_F5: 'F5',
            Qt.Key.Key_F6: 'F6',
            Qt.Key.Key_F7: 'F7',
            Qt.Key.Key_F8: 'F8',
            Qt.Key.Key_F9: 'F9',
            Qt.Key.Key_F10: 'F10',
            Qt.Key.Key_F11: 'F11',
            Qt.Key.Key_F12: 'F12',
            Qt.Key.Key_F13: 'F13',
            0: 'SHIFT'
        }

        if key in key_map:
            return key_map[key]
        else:
            return f"Unknown({key})"

    def handle_key_event(self, key, pressed):
        key_name = self.get_key_name(key)
        if pressed:
            self.key_set.add(key_name)  # 누른 키를 추가
            # print(f"Pressed: {key_name}")
        else:
            self.key_set.discard(key_name)  # 뗀 키를 제거
            # print(f"Released: {key_name}")
        self.command_dict['key_control'] = self.key_set
        self.key_mouse_control(self.command_dict)

    def key_mouse_control(self, command):
        key = command['key_control']

        if 'W' in key or 'S' in key or 'A' in key or 'D' in key or 'Q' in key or 'E' in key:
            # print("Move")
            straight_walk = 0
            side_walk = 0
            rotation_walk = 0
            ### 앞 뒤 조작
            if 'W' in key:
                straight_walk = -1
            elif 'S' in key:
                straight_walk = 1

            ### 좌 우 조작
            if 'A' in key and 'D' in key:
                side_walk = 0
            elif 'A' in key:
                side_walk = -1
            elif 'D' in key:
                side_walk = 1

            ### 방향 조작
            if 'Q' in key and 'E' in key:
                rotation_walk = 0
            elif 'Q' in key:
                rotation_walk = -1
            elif 'E' in key:
                rotation_walk = 1

            example_single_move_command = robot_command(straight_walk)
            example_move_array_command = robot_array_command(straight_walk, side_walk, rotation_walk)

            # print(move_command)
            pub.sendMessage('send_message', message=example_single_move_command)
            pub.sendMessage('send_message', message=example_move_array_command)


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
    # def key_mouse_control(self, command):
    #     # print(command)
    #     key = command['key_control']
    #     mouse_d_move = command['mouse_d_move']
    #     mouse_d_wheel = command['mouse_d_wheel']
    #     mouse_click = command['mouse_click']

    #     if self.base_commander:

    #         if "RETURN" in key:
    #             self.base_commander.hello_universe(message='Hello Universe!')
