import os
import sys
import time
import math
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from pymodbus.client import ModbusTcpClient
from gerri.robot.function.modbus_helper import ModbusHelper
import time
import threading

modbus_helper = ModbusHelper()

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


        self.write_register_address_tool_ref = 155

        self.write_register_address_key_control= 161

        self.mode_register = 0

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
                    self.joint_state['position'][i] = modbus_helper.modbus_decode(result_joint.registers[i])
            else:
                print("joint state read fail")

            if not result_pose.isError():
                # print("[READ] Joint values:")
                for i in range(self.read_pose_register_count):
                    if i < 3:
                        self.pose['position'][i] = modbus_helper.modbus_decode(result_pose.registers[i])
                    else:
                        self.pose['orientation'][i-3] = modbus_helper.modbus_decode(result_pose.registers[i])
                # for j in range(6):
                #     print(j, ": ", modbus_helper.modbus_decode(offset_result_pose.registers[j]))
            else:
                print("pose state read fail")

    def stop_robot(self, value):
        print("stop Left Robot", value)
        self.set_robot_mode = 0

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
            # drl_script_pause()

    def joint_ctrl(self, value, vel=120, acc=120):
        self.set_robot_mode = 3

        j1 = self.clamp(value[0], min_value = -360, max_value = 360)
        j2 = self.clamp(value[1], min_value = -150, max_value = 150)
        j3 = self.clamp(value[2], min_value = -135, max_value = 135)
        j4 = self.clamp(value[3], min_value = -360, max_value = 360)
        j5 = self.clamp(value[4], min_value = -135, max_value = 135)
        j6 = self.clamp(value[5], min_value = -360, max_value = 360)

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)

        joint_value = [modbus_helper.modbus_encode(j1, 10),
                     modbus_helper.modbus_encode(j2, 10),
                     modbus_helper.modbus_encode(j3, 10),
                     modbus_helper.modbus_encode(j4, 10),
                     modbus_helper.modbus_encode(j5, 10),
                     modbus_helper.modbus_encode(j6, 10),
                     modbus_helper.modbus_encode(vel, 10),
                     modbus_helper.modbus_encode(acc, 10)]
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
        self.client.write_register(self.write_register_address_x, modbus_helper.modbus_encode(x))
        self.client.write_register(self.write_register_address_y, modbus_helper.modbus_encode(y))
        self.client.write_register(self.write_register_address_z, modbus_helper.modbus_encode(z))
        self.client.write_register(self.write_register_address_rx, modbus_helper.modbus_encode(rx))
        self.client.write_register(self.write_register_address_ry, modbus_helper.modbus_encode(ry))
        self.client.write_register(self.write_register_address_rz, modbus_helper.modbus_encode(rz))
        self.client.write_register(self.write_register_address_lvel, modbus_helper.modbus_encode(value['velocity']))
        self.client.write_register(self.write_register_address_lacc, modbus_helper.modbus_encode(value['acceleration']))
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
        self.client.write_register(self.write_register_address_j1, modbus_helper.modbus_encode(j1))
        self.client.write_register(self.write_register_address_j2, modbus_helper.modbus_encode(j2))
        self.client.write_register(self.write_register_address_j3, modbus_helper.modbus_encode(j3))
        self.client.write_register(self.write_register_address_j4, modbus_helper.modbus_encode(j4))
        self.client.write_register(self.write_register_address_j5, modbus_helper.modbus_encode(j5))
        self.client.write_register(self.write_register_address_j6, modbus_helper.modbus_encode(j6))
        self.client.write_register(self.write_register_address_jvel, modbus_helper.modbus_encode(value['joint_velocity']))
        self.client.write_register(self.write_register_address_jacc, modbus_helper.modbus_encode(value['joint_acceleration']))

    def end_pose_ctrl(self, pose, vel=100, acc=200, dt=0.1):
        self.set_robot_mode = 4

        x, y, z, rx, ry, rz = pose

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_x, modbus_helper.modbus_encode(x))
        self.client.write_register(self.write_register_address_y, modbus_helper.modbus_encode(y))
        self.client.write_register(self.write_register_address_z, modbus_helper.modbus_encode(z))
        self.client.write_register(self.write_register_address_rx, modbus_helper.modbus_encode(rx))
        self.client.write_register(self.write_register_address_ry, modbus_helper.modbus_encode(ry))
        self.client.write_register(self.write_register_address_rz, modbus_helper.modbus_encode(rz))
        self.client.write_register(self.write_register_address_lvel, modbus_helper.modbus_encode(vel))
        self.client.write_register(self.write_register_address_lacc, modbus_helper.modbus_encode(acc))
        self.client.write_register(self.write_register_address_rel, modbus_helper.modbus_encode(dt))


    def xyz_move(self, value):
        pose_value = modbus_helper.modbus_encode(value, 100)
        self.client.write_registers(self.write_register_address_key_control, pose_value)


    def joint_ctrl_vel(self, value, acc=250, dt=0.1):
        self.set_robot_mode = 5
        v1, v2, v3, v4, v5, v6 = value

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)

        vel_value = [modbus_helper.modbus_encode(v1, 100),
                     modbus_helper.modbus_encode(v2, 100),
                     modbus_helper.modbus_encode(v3, 100),
                     modbus_helper.modbus_encode(v4, 100),
                     modbus_helper.modbus_encode(v5, 100),
                     modbus_helper.modbus_encode(v6, 100),
                     modbus_helper.modbus_encode(acc, 100),
                     modbus_helper.modbus_encode(dt, 100)]
        self.client.write_registers(self.write_register_address_v1, vel_value)

    def joint_ctrl_vel_stop(self):
        self.set_robot_mode = 0

        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_v1, modbus_helper.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v2, modbus_helper.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v3, modbus_helper.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v4, modbus_helper.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v5, modbus_helper.modbus_encode(0, 100))
        self.client.write_register(self.write_register_address_v6, modbus_helper.modbus_encode(0, 100))


    def gripper_ctrl(self, value):
        print("Moving Gripper")
        self.set_robot_mode = 2
        self.client.write_register(self.write_register_address_0, self.set_robot_mode)
        self.client.write_register(self.write_register_address_x, 0)
        self.client.write_register(self.write_register_address_y, 0)
        self.client.write_register(self.write_register_address_z, modbus_helper.modbus_encode(value))
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


    def change_mode(self, value):
        self.set_robot_mode = value
        self.client.write_register(self.write_register_address_0, self.set_robot_mode)


    def get_robot_mode(self):
        print(self.mode_register)
        return self.mode_register

if __name__ == '__main__':
    robot = DoosanController('172.20.1.247', 502)
    robot.connect()
    time.sleep(1)
    robot.update_status()
    time.sleep(1)
    print(robot.joint_state)