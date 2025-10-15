import os
import sys
import time
import math
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from pymodbus.client import ModbusTcpClient
import time
import threading

class DoosanController:
    def __init__(self, ip, port):
        """
        ROBOT INIT CODE
        """
        self.robot_state = 'idle'
        self.pose = {
            'position': [0, 0, 0],
            'orientation': [0, 0, 0],
        }
        self.joint_state = {
            'name': ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
            'position': [0., 0., 0., 0., 0., 0.],
            'velocity': [0, 0, 0, 0],
            'effort': [0, 0, 0, 0],
        }


        self.ip = ip
        self.port = port

        self.retry_delay = 10

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

        ### pose
        self.write_register_address_x = 138
        self.write_register_address_y = 139
        self.write_register_address_z = 140
        self.write_register_address_rx = 141
        self.write_register_address_ry = 142
        self.write_register_address_rz = 143
        self.write_register_address_lvel = 144
        self.write_register_address_lacc = 145
        self.write_register_address_rel = 146

        ### speed
        self.write_register_address_v1 = 147
        self.write_register_address_v2 = 148
        self.write_register_address_v3 = 149
        self.write_register_address_v4 = 150
        self.write_register_address_v5 = 151
        self.write_register_address_v6 = 152
        self.write_register_address_vacc = 153
        self.write_register_address_vdt = 154

        self.client = None

    def connect(self):
        """
        CONNECT TO ROBOT
        """
        while True:
            try:
                self.client = ModbusTcpClient(self.ip, port=self.port)
                connection = False
                connection = self.client.connect()

                if connection:
                    print("############ Connect to Doosan M1509 ###############")
                    return self.client
                else:
                    print(f"Doosan 로봇에 연결 실패. {self.retry_delay}초 후 다시 시도합니다.")
            except Exception as e:
                print(f"연결 중 오류 발생: {e}. {self.retry_delay}초 후 다시 시도합니다.")

            time.sleep(self.retry_delay)

    def disconnect(self):
        """
        DISCONNECT FROM ROBOT
        """

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
                    self.robot_state = "UNKNOWN"
                # print(self.robot_state, result_robot_state.status )
            else:
                print("state read fail")

            if not result_joint.isError():
                # print("[READ] Joint values:")
                for i in range(self.read_joint_register_count):
                    self.joint_state['position'][i] = self.modbus_decode(result_joint.registers[i])
            else:
                print("joint state read fail")

            if not result_pose.isError():
                # print("[READ] Joint values:")
                for i in range(self.read_pose_register_count):
                    if i < 3:
                        self.pose['position'][i] = self.modbus_decode(result_pose.registers[i])
                    else:
                        self.pose['orientation'][i-3] = self.modbus_decode(result_pose.registers[i])
                # for j in range(6):
                #     print(j, ": ", self.modbus_decode(offset_result_pose.registers[j]))
            else:
                print("pose state read fail")

    # def convert_to_int_joint(self, val):
    #     if val >= 0x8000:
    #         val = val - 0x10000
    #     if -3600 <= val <= 3600:
    #         return val / 10.
    #     else:
    #         return val
    # 
    # def convert_to_int_pose(self, val):
    #     if val >= 0x8000:
    #         val = val - 0x10000
    #     if -9000 <= val <= 9000:
    #         return val / 10.
    #     else:
    #         return val
    # 
    # def convert_to_hex_joint(self, value):
    #     scaled = int(round(value * 10))
    #     if not -3600 <= scaled <= 3600:
    #         raise ValueError("over Value")
    #     return scaled & 0xFFFF
    # 
    # def convert_to_hex_pose(self, value):
    #     scaled = int(round(value * 10))
    #     if not -9000 <= scaled <= 9000:
    #         raise ValueError("over Value")
    #     return scaled & 0xFFFF

    def modbus_encode(self, value, scale = 10):
        scaled_value = int(round(value * scale))
        return scaled_value & 0xFFFF

    def modbus_decode(self, raw_value, scale = 10):
        signed_value = raw_value if raw_value < 0x8000 else raw_value - 0x10000
        return float(signed_value) / scale

    def stop_robot(self, value):
        print("stop Left Robot", value)
        self.set_robot_mode = 0

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
            # drl_script_pause()

    def joint_ctrl(self, value, vel=120, acc=120):
        self.set_robot_mode = 1

        j1 = self.clamp(value[0], min_value = -360, max_value = 360)
        j2 = self.clamp(value[1], min_value = -150, max_value = 150)
        j3 = self.clamp(value[2], min_value = -135, max_value = 135)
        j4 = self.clamp(value[3], min_value = -360, max_value = 360)
        j5 = self.clamp(value[4], min_value = -135, max_value = 135)
        j6 = self.clamp(value[5], min_value = -360, max_value = 360)

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)

        joint_value = [self.modbus_encode(j1, 100),
                     self.modbus_encode(j2, 100),
                     self.modbus_encode(j3, 100),
                     self.modbus_encode(j4, 100),
                     self.modbus_encode(j5, 100),
                     self.modbus_encode(j6, 100),
                     self.modbus_encode(vel, 100),
                     self.modbus_encode(acc, 100)]
        self.client.write_registers(self.write_register_address_j1, joint_value)


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
        self.client.write_register(self.write_register_address_x, self.modbus_encode(x))
        self.client.write_register(self.write_register_address_y, self.modbus_encode(y))
        self.client.write_register(self.write_register_address_z, self.modbus_encode(z))
        self.client.write_register(self.write_register_address_rx, self.modbus_encode(rx))
        self.client.write_register(self.write_register_address_ry, self.modbus_encode(ry))
        self.client.write_register(self.write_register_address_rz, self.modbus_encode(rz))
        self.client.write_register(self.write_register_address_lvel, self.modbus_encode(value['velocity']))
        self.client.write_register(self.write_register_address_lacc, self.modbus_encode(value['acceleration']))
        self.client.write_register(self.write_register_address_rel, 0)

    def joint_ctrl_master(self, value):
        self.set_robot_mode = 3
        j1 = self.clamp(value[0], min_value = -360, max_value = 360)
        j2 = self.clamp(value[1], min_value = -95, max_value = 95)
        j3 = self.clamp(value[2], min_value = -135, max_value = 135)
        j4 = self.clamp(value[3], min_value = -360, max_value = 360)
        j5 = self.clamp(value[4], min_value = -135, max_value = 135)
        j6 = self.clamp(value[5], min_value = -360, max_value = 360)

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_j1, self.modbus_encode(j1))
        self.client.write_register(self.write_register_address_j2, self.modbus_encode(j2))
        self.client.write_register(self.write_register_address_j3, self.modbus_encode(j3))
        self.client.write_register(self.write_register_address_j4, self.modbus_encode(j4))
        self.client.write_register(self.write_register_address_j5, self.modbus_encode(j5))
        self.client.write_register(self.write_register_address_j6, self.modbus_encode(j6))
        self.client.write_register(self.write_register_address_jvel, self.modbus_encode(value['joint_velocity']))
        self.client.write_register(self.write_register_address_jacc, self.modbus_encode(value['joint_acceleration']))

    def end_pose_ctrl(self, pose, vel=100, acc=200, dt=0.1):
        self.set_robot_mode = 4

        x, y, z, rx, ry, rz = pose

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_x, self.modbus_encode(x))
        self.client.write_register(self.write_register_address_y, self.modbus_encode(y))
        self.client.write_register(self.write_register_address_z, self.modbus_encode(z))
        self.client.write_register(self.write_register_address_rx, self.modbus_encode(rx))
        self.client.write_register(self.write_register_address_ry, self.modbus_encode(ry))
        self.client.write_register(self.write_register_address_rz, self.modbus_encode(rz))
        self.client.write_register(self.write_register_address_lvel, self.modbus_encode(vel))
        self.client.write_register(self.write_register_address_lacc, self.modbus_encode(acc))
        self.client.write_register(self.write_register_address_rel, self.modbus_encode(dt))



    def joint_ctrl_vel(self, value, acc=250, dt=0.1):
        self.set_robot_mode = 5
        v1, v2, v3, v4, v5, v6 = value

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)

        vel_value = [self.modbus_encode(v1, 100),
                     self.modbus_encode(v2, 100),
                     self.modbus_encode(v3, 100),
                     self.modbus_encode(v4, 100),
                     self.modbus_encode(v5, 100),
                     self.modbus_encode(v6, 100),
                     self.modbus_encode(acc, 100),
                     self.modbus_encode(dt, 100)]
        self.client.write_registers(self.write_register_address_v1, vel_value)

    def joint_ctrl_vel_stop(self):
        self.set_robot_mode = 0

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_v1, self.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v2, self.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v3, self.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v4, self.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v5, self.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v6, self.modbus_encode(0, 100))


    def gripper_ctrl(self, value):
        print("Moving Gripper")
        self.set_robot_mode = 2
        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_x, 0)
        self.client.write_register(self.write_register_address_y, 0)
        self.client.write_register(self.write_register_address_z, self.modbus_encode(value))
        self.client.write_register(self.write_register_address_rx, 0)
        self.client.write_register(self.write_register_address_ry, 0)
        self.client.write_register(self.write_register_address_rz, 0)
        self.client.write_register(self.write_register_address_lvel, 50)
        self.client.write_register(self.write_register_address_lacc, 50)
        self.client.write_register(self.write_register_address_rel, 1)

    @staticmethod
    def clamp(value, min_value, max_value, absolute_limit=None):
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))


    def custom_command(self, command):
        return command


if __name__ == '__main__':
    robot = DoosanController('172.20.1.247', 502)
    robot.connect()
    time.sleep(1)
    robot.update_status()
    time.sleep(1)
    print(robot.joint_state)