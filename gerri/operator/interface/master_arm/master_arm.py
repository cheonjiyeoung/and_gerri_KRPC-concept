#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from gerri.operator.interface.master_arm.Dynamixel import *
import random
import numpy as np
import ctypes
import time
import copy
import threading

class MasterArm:
    def __init__(self, n_dxls, port='/dev/ttyUSB0', baudrate=115200):
        """
        Initialize the MasterArm class.
        :param n_dxls: Number of joints (axes) in the arm.
        :param port: Port to connect to the Dynamixel device.
        """
        self.control_mode = 0  # Control mode for the robot arm (default: 0)
        self.n_dxls = n_dxls  # Number of joints (axes) in the robot arm
        self.armDynamixel = Dynamixel(device_name=port, baudrate = baudrate)  # Dynamixel device object initialization
        self.armDynamixel_id = list(range(self.n_dxls))  # Dynamixel IDs for each joint

        # Initialize each Dynamixel motor
        for i in range(len(self.armDynamixel_id)):
            _id = self.armDynamixel_id[i]
            self.armDynamixel.enable_led(_id, True)  # Enable LED for visual feedback
            time.sleep(0.1)
            self.armDynamixel.enable_torque(_id, False)  # Disable torque initially
            time.sleep(0.1)

        self.arm_default_pos_cnt = [0.] * self.n_dxls  # Default position counts for each joint
        self.arm_offsets_deg = [0.] * self.n_dxls  # Offset angles in degrees for each joint
        self.position_rad = [0] * self.n_dxls  # Position of each joint in radians

        self.armDynamixel.set_get_pos_sync(self.armDynamixel_id)  # Set up synchronous position fetching
        self.res = True  # Operation result flag
        self.masterStart = False  # Master start flag
        self.initComplete = False  # Initialization complete flag
        self.readBefore = False  # Flag to indicate if positions were read before
        self.getCurrentArmJoint = False  # Current joint reading flag
        self.gripper_active = False  # Gripper activation flag
        self.flag_init_thread = False  # Thread initialization flag

        self._lock = threading.Lock()  # Thread lock for synchronizing access

        print(f"[Python] Dynamixel ready for {self.n_dxls} axes")

    def disconnect(self):
        """
        Safely disconnect all Dynamixel motors by disabling torque _and_ closing the port.
        """
        # Disable torque for each Dynamixel motor
        for _id in self.armDynamixel_id:
            comm_result, error = self.armDynamixel.enable_torque(_id, False)
            if comm_result != 0:  # Replace '0' with the appropriate COMM_SUCCESS constant
                print(f"Communication error on ID {_id}: {self.armDynamixel.get_comm_result_msg(comm_result)}")
            elif error != 0:
                print(f"Packet error on ID {_id}: {self.armDynamixel.get_packet_error_msg(error)}")
            else:
                print(f"Torque disabled for Dynamixel ID {_id}")

        # Close the port
        self.armDynamixel.close_port()
        print("Dynamixel port closed successfully.")


    def updateDefaultPosCnt(self, default_pos_cnt = None):
        positions_cnt = self.readPosition_cnt()
        if default_pos_cnt:
            self.arm_offsets_deg = default_pos_cnt
        for i in range(self.n_dxls):
            self.arm_default_pos_cnt[i] = positions_cnt[i]
        print("Reset complete:", self.arm_offsets_deg)

    def readPosition_cnt(self):
        # Read the position counts of all joints
        self._lock.acquire()  # Acquire thread lock
        self.res, p1 = self.armDynamixel.get_pos_sync(self.armDynamixel_id)  # Get synchronous position data
        self._lock.release()  # Release thread lock

        positions_cnt = list(p1[:self.n_dxls])

        # Convert position counts to signed integers if necessary
        for i in range(self.n_dxls):
            if positions_cnt[i] > 0x7fffffff:
                positions_cnt[i] -= 4294967296

        return positions_cnt

    def readPosition(self):
        # Read the positions of all joints in degrees _and_ radians
        positions_cnt = self.readPosition_cnt()

        self.position_deg = []
        self.position_rad = [0] * self.n_dxls
        for i in range(self.n_dxls):
            if i == 1 or i == 3:
                _q =  self.arm_offsets_deg[i] - (positions_cnt[i] * 360. / 4096. - self.arm_default_pos_cnt[i] * 360. / 4096.)
            else :
                 _q = positions_cnt[i] * 360. / 4096. - self.arm_default_pos_cnt[i] * 360. / 4096. + self.arm_offsets_deg[i]
            self.position_deg.append(_q)

        # Normalize angles to the range [-360, 360]
        for i in range(self.n_dxls):
            if self.position_deg[i] > 360.:
                _n = int(self.position_deg[i] / 360.) + 1
                self.position_deg[i] -= _n * 360.
            elif self.position_deg[i] < -360.:
                _n = int(self.position_deg[i] / 360.) - 1
                self.position_deg[i] += _n * 360.
            self.position_rad[i] = round(self.position_deg[i] * np.pi / 180., 3)

    def print_deg(self):
        # Print the current positions of all joints in degrees
        self.readPosition()
        _str = 'RobotArm:' + ', '.join(['{:0.2f}'.format(rad * 180 / np.pi) for rad in self.position_rad])
        print(_str)

    def get_position_rad(self):
        # Get the current positions of all joints in radians
        self.readPosition()
        return self.position_rad

    def get_position_deg(self):
        # Get the current positions of all joints in degrees
        self.readPosition()
        return self.position_deg


if __name__ == '__main__':
    # Main function to initialize the robot arm _and_ print joint positions
    n_dxls = int(input("Enter the number of joints (axes) for the robot arm: "))  # Get the number of joints from the user
    dxls = MasterArm(n_dxls=n_dxls, port='COM4')  # Initialize the MasterArm with the given number of joints
    dxls.updateDefaultPosCnt()  # Update the default position counts
    while True:
        dxls.print_deg()  # Continuously print the joint positions in degrees
        time.sleep(0.1)
