import os
import sys
import time
import math


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from pymodbus.client import ModbusTcpClient
import time
import threading

class DoosanMRightController:
    def __init__(self):
        """
        ROBOT INIT CODE
        """
        self.base_controller = None
        self.robot_state = 'idle'
        self.pose = {
            'position': [0, 0, 0],
            'orientation': [0, 0, 0],
        }
        self.joint_state = {
            'name': ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
            'position': [-90., 30., -120., -90., 0., 0.],
            'velocity': [0, 0, 0, 0],
            'effort': [0, 0, 0, 0],
        }

        self.set_robot_mode = 0

        self.read_state_register_address = 259
        self.read_joint_register_address = 270
        self.read_joint_register_count = 6
        self.read_pose_register_address = 400
        self.read_pose_register_count = 6
        self.read_pose_tool_register_address = 420
        self.read_pose_tool_register_count = 6
        self.stop_reading = False  # 스레드 종료 플래그

        self.write_register_address_0 = 129
        ### joint
        self.write_register_address_j1 = 130
        self.write_register_address_j2 = 131
        self.write_register_address_j3 = 132
        self.write_register_address_j4 = 133
        self.write_register_address_j5 = 134
        self.write_register_address_j6 = 135
        self.write_register_address_jvel = 136
        self.write_register_address_jacc = 137

        #### pose
        self.write_register_address_x = 138
        self.write_register_address_y = 139
        self.write_register_address_z = 140
        self.write_register_address_rx = 141
        self.write_register_address_ry = 142
        self.write_register_address_rz = 143
        self.write_register_address_lvel = 144
        self.write_register_address_lacc = 145
        self.write_register_address_rel = 146

        self.client = None

        self.test = 1.
        self.test_90 = False
        self.test_120 = False

    def connect(self, ip, port):
        """
        CONNECT TO ROBOT
        """
        while True:
            try:
                self.client = ModbusTcpClient(ip, port=port)
                connection = False
                connection = self.client.connect()

                if connection:
                    print("############ Connect to Doosan M1509 right ###############")
                    return self.client
                else:
                    print(f"Doosan right로봇에 연결 실패. {self.retry_delay}초 후 다시 시도합니다.")
            except Exception as e:
                print(f"right 연결 중 오류 발생: {e}. {self.retry_delay}초 후 다시 시도합니다.")

            time.sleep(5)

    def disconnect(self):
        """
        DISCONNECT FROM ROBOT
        """

    # def update_status_test(self):
    #     if self.joint_state['position'][2] == -120.:
    #         self.test_120 = True
    #         self.test_90 = False
    #     if self.joint_state['position'][2] == -90.:
    #         self.test_120 = False
    #         self.test_90 = True
    #
    #     if self.test_120 == True and self.test_90 == False:
    #         self.joint_state['position'][2] = self.joint_state['position'][2] + self.test
    #     if self.test_90 == True and self.test_120 == False:
    #         self.joint_state['position'][2] = self.joint_state['position'][2] - self.test

    def update_status(self):
        if self.client:
            result_robot_state = self.client.read_holding_registers(self.read_state_register_address, count=1)
            result_joint = self.client.read_holding_registers(self.read_joint_register_address, count=self.read_joint_register_count)
            result_pose = self.client.read_holding_registers(self.read_pose_register_address, count=self.read_pose_register_count)
            # offset_result_pose = self.client.read_holding_registers(self.read_pose_tool_register_address, count=self.read_pose_tool_register_count)
            if not result_robot_state.isError():
                if result_robot_state.status == 0:
                    self.robot_state = "INITIALIZING"
                elif result_robot_state.status == 1:
                    self.robot_state = "STANDBY"
                elif result_robot_state.status == 2:
                    self.robot_state = "OPERATIING"
                elif result_robot_state.status == 3:
                    self.robot_state = "SAFE OFF"
                elif result_robot_state.status == 4:
                    self.robot_state = "TEACHING"
                elif result_robot_state.status == 5:
                    self.robot_state = "SAFE STOP"
                elif result_robot_state.status == 6:
                    self.robot_state = "EMERGENCY STOP"
                elif result_robot_state.status == 7:
                    self.robot_state = "HOMING"
                elif result_robot_state.status == 8:
                    self.robot_state = "RECOVERY"
                elif result_robot_state.status == 9:
                    self.robot_state = "SAFE STOP2"
                elif result_robot_state.status == 10:
                    self.robot_state = "SAFE OFF2"
                elif result_robot_state.status == 15:
                    self.robot_state = "NOT READY"
                else:
                    self.robot_state = "UNKWON"
                # print(self.robot_state, result_robot_state.status )
            else:
                print("state read fail")

            if not result_joint.isError():
                # print("[READ] Joint values:")
                for i in range(self.read_joint_register_count):
                    self.joint_state['position'][i] = self.convert_to_int_joint(result_joint.registers[i])
            else:
                print("joint state read fail")

            if not result_pose.isError():
                # print("[READ] Joint values:")
                for i in range(self.read_pose_register_count):
                    if i < 3:
                        self.pose['position'][i] = self.convert_to_int_pose(result_pose.registers[i])
                    else:
                        self.pose['orientation'][i-3] = self.convert_to_int_joint(result_pose.registers[i])
                # for j in range(6):
                #     print(j, ": ", self.convert_to_int_joint(offset_result_pose.registers[j]))
            else:
                print("pose state read fail")
        return 'Doosan Arm right State'

    def convert_to_int_joint(self, val):
        if val >= 0x8000:
            val = val - 0x10000
        if -3600 <= val <= 3600:
            return val / 10.
        else:
            return val

    def convert_to_int_pose(self, val):
        if val >= 0x8000:
            val = val - 0x10000
        if -9000 <= val <= 9000:
            return val / 10.
        else:
            return val

    def convert_to_hex_joint(self, value):
        scaled = int(round(value * 10))
        if not -3600 <= scaled <= 3600:
            raise ValueError("over Value")
        return scaled & 0xFFFF

    def convert_to_hex_pose(self, value):
        scaled = int(round(value * 10))
        if not -9000 <= scaled <= 9000:
            raise ValueError("over Value")
        return scaled & 0xFFFF

    def hello_universe(self, value):
        print('hello_universe')
        return 'hello_universe_by_robot'

    def stop_robot(self, value):
        print("stop Robot", value)
        self.set_robot_mode = 0

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
            # drl_script_pause()

    def joint_ctrl(self, value):
        print('joint_ctrl')
        self.set_robot_mode = 1
        j1 = value['joint_angle'][0]
        j1 = max(-360., min(360., j1))
        j2 = value['joint_angle'][1]
        j2 = max(-95., min(95., j2))
        j3 = value['joint_angle'][2]
        j3 = max(-135., min(135., j3))
        j4 = value['joint_angle'][3]
        j4 = max(-360., min(360., j4))
        j5 = value['joint_angle'][4]
        j5 = max(-135., min(135., j5))
        j6 = value['joint_angle'][5]
        j6 = max(-360., min(360., j6))

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_j1, self.convert_to_hex_joint(j1))
        self.client.write_register(self.write_register_address_j2, self.convert_to_hex_joint(j2))
        self.client.write_register(self.write_register_address_j3, self.convert_to_hex_joint(j3))
        self.client.write_register(self.write_register_address_j4, self.convert_to_hex_joint(j4))
        self.client.write_register(self.write_register_address_j5, self.convert_to_hex_joint(j5))
        self.client.write_register(self.write_register_address_j6, self.convert_to_hex_joint(j6))
        self.client.write_register(self.write_register_address_jvel, self.convert_to_hex_joint(value['joint_velocity']))
        self.client.write_register(self.write_register_address_jacc, self.convert_to_hex_joint(value['joint_acceleration']))

    def movel(self, value):
        print("Movel")
        self.set_robot_mode = 2
        x = value['point'][0]
        y = value['point'][1]
        z = value['point'][2]
        rx = value['angle'][0]
        ry = value['angle'][1]
        rz = value['angle'][2]

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_x, self.convert_to_hex_pose(x))
        self.client.write_register(self.write_register_address_y, self.convert_to_hex_pose(y))
        self.client.write_register(self.write_register_address_z, self.convert_to_hex_pose(z))
        self.client.write_register(self.write_register_address_rx, self.convert_to_hex_pose(rx))
        self.client.write_register(self.write_register_address_ry, self.convert_to_hex_pose(ry))
        self.client.write_register(self.write_register_address_rz, self.convert_to_hex_pose(rz))
        self.client.write_register(self.write_register_address_lvel, self.convert_to_hex_pose(value['velocity']))
        self.client.write_register(self.write_register_address_lacc, self.convert_to_hex_pose(value['acceleration']))
        self.client.write_register(self.write_register_address_rel, 0)

    def joint_ctrl_master(self, value):
        self.set_robot_mode = 3
        j1 = value['master_joint_angle'][0]
        j1 = max(-360., min(360., j1))
        j2 = value['master_joint_angle'][1]
        j2 = max(-95., min(95., j2))
        j3 = value['master_joint_angle'][2]
        j3 = max(-135., min(135., j3))
        j4 = value['master_joint_angle'][3]
        j4 = max(-360., min(360., j4))
        j5 = value['master_joint_angle'][4]
        j5 = max(-135., min(135., j5))
        j6 = value['master_joint_angle'][5]
        j6 = max(-360., min(360., j6))

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_j1, self.convert_to_hex_joint(j1))
        self.client.write_register(self.write_register_address_j2, self.convert_to_hex_joint(j2))
        self.client.write_register(self.write_register_address_j3, self.convert_to_hex_joint(j3))
        self.client.write_register(self.write_register_address_j4, self.convert_to_hex_joint(j4))
        self.client.write_register(self.write_register_address_j5, self.convert_to_hex_joint(j5))
        self.client.write_register(self.write_register_address_j6, self.convert_to_hex_joint(j6))
        self.client.write_register(self.write_register_address_jvel, self.convert_to_hex_joint(value['joint_velocity']))
        self.client.write_register(self.write_register_address_jacc, self.convert_to_hex_joint(value['joint_acceleration']))

    def gripper_ctrl(self, value):
        print("Moving Gripper")
        self.set_robot_mode = 2
        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_x, 0)
        self.client.write_register(self.write_register_address_y, 0)
        self.client.write_register(self.write_register_address_z, self.convert_to_hex_pose(value))
        self.client.write_register(self.write_register_address_rx, 0)
        self.client.write_register(self.write_register_address_ry, 0)
        self.client.write_register(self.write_register_address_rz, 0)
        self.client.write_register(self.write_register_address_lvel, 50)
        self.client.write_register(self.write_register_address_lacc, 50)
        self.client.write_register(self.write_register_address_rel, 1)

    def custom_command(self, command):
        return command
