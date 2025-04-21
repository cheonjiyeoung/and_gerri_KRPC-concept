# base_controller.py
from logging import lastResort

from pubsub import pub
import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
PAN_DXL_ID                  = 1                                         # 좌우 모터
TILT_DXL_ID                 = 2                                         # 상하 모터
BAUDRATE                    = 1000000                                   # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'                            # Check which port is being used on your controller
                                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                                         # Value for enabling the torque
TORQUE_DISABLE              = 0                                         # Value for disabling the torque
PAN_MINIMUM_POSITION_VALUE  = 0                                         # Dynamixel will rotate between this value
PAN_MAXIMUM_POSITION_VALUE  = 4096                                      # _and_ this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
TILT_MINIMUM_POSITION_VALUE = 1000                                        # Dynamixel will rotate between this value
TILT_MAXIMUM_POSITION_VALUE = 2500                                     # _and_ this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                                        # Dynamixel moving status threshold
DEFAULT_PAN_POSITION_VALUE = 2048
DEFAULT_TILT_POSITION_VALUE = 2048



# Open port
class PanTiltController:           # 2개의 다이나믹셀을 목처럼 컨트롤하기위해 개발된 코드임
    def __init__(self, robot_name="pan_tilt", use_port=DEVICENAME, pan_motor_id=PAN_DXL_ID, tilt_motor_id=TILT_DXL_ID):
        super().__init__()
        self.robot_name = robot_name
        self.robot_type = "pan_tilt"

        self.portHandler = PortHandler(use_port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.pan_motor_id = pan_motor_id
        self.tilt_motor_id = tilt_motor_id

        self.default_pan_value = 2048
        self.default_tilt_angle = 2048

        self.last_pan_value = 0
        self.last_tilt_value = 0

        pub.subscribe(self.message_handler, 'receive_message')

    @staticmethod
    def clamp(value, min_value, max_value, absolute_limit=None):
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))

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


    def connect(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        # Enable Dynamixel Torque
        print(self.packetHandler.write1ByteTxRx(self.portHandler, PAN_DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE))
        print(self.packetHandler.write1ByteTxRx(self.portHandler, TILT_DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE))
        self.pan_tilt_control(self.default_pan_value, self.default_tilt_angle)


    def message_handler(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if topic == "pan_tilt":
                self.angle_control(pan_angle=value[0], tilt_angle=value[1])
            elif topic == "pan_tilt_step":
                self.step_control(pan_step=int(value[0] * 100), tilt_step=int(value[1] * 100))


    def pan_tilt_control(self, pan_value, tilt_value):
        pan_value = int(self.clamp(pan_value, PAN_MINIMUM_POSITION_VALUE, PAN_MAXIMUM_POSITION_VALUE))
        tilt_value = int(self.clamp(tilt_value, TILT_MINIMUM_POSITION_VALUE, TILT_MAXIMUM_POSITION_VALUE))
        self.packetHandler.write4ByteTxRx(self.portHandler, self.pan_motor_id, ADDR_PRO_GOAL_POSITION, pan_value)
        self.packetHandler.write4ByteTxRx(self.portHandler, self.tilt_motor_id, ADDR_PRO_GOAL_POSITION, tilt_value)
        # if pan_control != COMM_SUCCESS _and_ tilt_control != COMM_SUCCESS:
        #     print("%s" % self.packetHandler.getTxRxResult(pan_control, tilt_control))
        # elif pan_error != 0 _and_ tilt_error != 0:
        #     print("%s" % self.packetHandler.getRxPacketError(pan_error, tilt_error))
        # else:
        self.last_pan_value = pan_value
        self.last_tilt_value = tilt_value
        print(self.last_pan_value, self.last_tilt_value)


    def angle_control(self, pan_angle, tilt_angle):
        pan_value = int(self.map_value(pan_angle, -180, 180, 0, 4096))
        tilt_value = int(self.map_value(tilt_angle, -180, 180, 0, 4096))
        if self.last_pan_value != pan_value or self.last_tilt_value != tilt_value:
            self.pan_tilt_control(pan_value, tilt_value)


    def step_control(self, pan_step = 0, tilt_step = 0):
        if pan_step != 0 or tilt_step != 0:
            self.pan_tilt_control(self.last_pan_value - pan_step, self.last_tilt_value - tilt_step)


    def home_position(self):
        self.pan_tilt_control(self.default_pan_value, self.default_tilt_angle)


    def disconnect(self):
        print(self.packetHandler.write1ByteTxRx(self.portHandler, PAN_DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE))
        print(self.packetHandler.write1ByteTxRx(self.portHandler, TILT_DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE))

        # Close port
        self.portHandler.closePort()

if __name__ == '__main__':
    neck = PanTiltController()
    neck.connect()
    time.sleep(2)
    neck.disconnect()

