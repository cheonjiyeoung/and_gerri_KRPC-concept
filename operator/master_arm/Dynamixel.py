from dynamixel_sdk import *
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


import dx_conf


class Dynamixel:   # This cl]ass handles a bundle of DXs through U2D2. Each class should be assigned to each U2D2.
    def __init__(self, device_name='/dev/ttyUSB0', protocol_version=2.0):
        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(protocol_version)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()
        
        # Set port baudrate
        if self.portHandler.setBaudRate(dx_conf.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, dx_conf.ADDR_GOAL_POSITION, dx_conf.LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, dx_conf.ADDR_PRESENT_POSITION, dx_conf.LEN_PRESENT_POSITION)
        
        # bulk read
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)

    def close_port(self):
        self.portHandler.closePort()

    def reboot(self, dxl_id):
        # Try reboot
        # Dynamixel LED will flicker while it reboots
        dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, dxl_id)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] reboot Succeeded\n" % dxl_id)

    def to_velocity_mode(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_OPERATING_MODE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    def to_position_mode(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_OPERATING_MODE, 3)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    def to_current_mode(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_OPERATING_MODE, 0x00)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            
    def enable_torque(self, dxl_id, flag):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_TORQUE_ENABLE, flag)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("dynamixel has been successfully connected")
        self.enable_led(dxl_id, flag)

    def enable_led(self, dxl_id, flag):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_LED_ENABLE, flag)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("dynamixel has been successfully connected")

    def set_pos_limit(self, dxl_id, pos_min, pos_max):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_MIN_POS_LIMIT, pos_min)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("dynamixel has been successfully connected")

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_MAX_POS_LIMIT, pos_max)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("dynamixel has been successfully connected")


    def set_vel(self, dxl_id, goal_velocity):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_GOAL_VELOCITY, int(goal_velocity))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))    

    def set_pos(self, dxl_id, goal_position):
        # thresholding min, max
        # if goal_position < dx_conf.DXL_MINIMUM_POSITION_VALUE:
        #     goal_position = dx_conf.DXL_MINIMUM_POSITION_VALUE
        # elif goal_position > dx_conf.DXL_MAXIMUM_POSITION_VALUE:
        #     goal_position = dx_conf.DXL_MAXIMUM_POSITION_VALUE

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_GOAL_POSITION, int(goal_position))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def set_pos_sync(self, dxl_ids, goal_positions):
        for id, pos in zip(dxl_ids, goal_positions):
            # Allocate goal position value into byte array
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(int(pos))), DXL_HIBYTE(DXL_LOWORD(int(pos))),
                                   DXL_LOBYTE(DXL_HIWORD(int(pos))), DXL_HIBYTE(DXL_HIWORD(int(pos)))]

            # Add dynamixel#1 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(id, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % id)
                quit()

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

    def set_curr(self, dxl_id, goal_current):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id,
                                                                       dx_conf.ADDR_GOAL_CURRENT,
                                                                       int(goal_current))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def get_moving_status(self, dxl_id):
        dxl_is_moving, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_MOVING)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_is_moving

    def get_pos(self, dxl_id):
        dxl_current_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_current_position

    def set_get_pos_sync(self, dxl_ids):
        for id in dxl_ids:
            # Add parameter storage for dynamixel present position value
            dxl_addparam_result = self.groupSyncRead.addParam(id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % id)
                quit()

    def set_get_pos_bulk(self, dxl_ids):
        for id in dxl_ids:
            dxl_addparam_result = self.groupBulkRead.addParam(id, dx_conf.ADDR_PRESENT_POSITION, dx_conf.LEN_PRESENT_POSITION)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkRead addparam failed" % id)
                quit()    

    def get_pos_sync(self, dxl_ids):
        res = True
        # Syncread present position
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        
        dxl_current_positions = []
        for id in dxl_ids:
            # Check if groupsyncread data of dynamixel is available
            dxl_getdata_result = self.groupSyncRead.isAvailable(id, dx_conf.ADDR_PRESENT_POSITION, dx_conf.LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
                #quit()
                res = False

            # Get the present position value
            dxl_current_positions.append(self.groupSyncRead.getData(id, dx_conf.ADDR_PRESENT_POSITION, dx_conf.LEN_PRESENT_POSITION))

        # Clear syncread parameter storage
        #self.groupSyncRead.clearParam()
        return res, dxl_current_positions


    def get_pos_bulk(self, dxl_ids):
        res = True
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        dxl_current_positions = []
        for id in dxl_ids:
            dxl_getdata_result = self.groupBulkRead.isAvailable(id, dx_conf.ADDR_PRESENT_POSITION, dx_conf.LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % id)
                #quit()
                res = False

            # Get the present position value
            dxl_current_positions.append(self.groupSyncRead.getData(id, dx_conf.ADDR_PRESENT_POSITION, dx_conf.LEN_PRESENT_POSITION))

        # Clear syncread parameter storage
        #self.groupSyncRead.clearParam()
        return res, dxl_current_positions

if __name__ == '__main__':
    # dxls = Dynamixel(device_name='/dev/ttyUSB0')
    dxls = Dynamixel(device_name='COM4')
    for i in range(1, 8):
        dxls.enable_torque(i, False)
        dxls.set_pos_limit(dxl_id=i, pos_min=0, pos_max=4095)
        dxls.enable_torque(i, True)

    time.sleep(1)
    print (dxls.get_pos_sync(dxl_ids=list(range(1, 8))))
    dxls.close_port()
