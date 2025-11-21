#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#*******************************************************************************
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions _and_
# limitations under the License.
#*******************************************************************************


from dynamixel_sdk import * # Uses dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
# MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V
MY_DXL = 'XL330'        # [WARNING] Operating Voltage : 5.0V

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_OPERATING_MODE         = 11
    ADDR_GOAL_VELOCITY          = 104
    ADDR_MAX_POS_LIMIT          = 48
    ADDR_MIN_POS_LIMIT          = 52
    ADDR_TORQUE_ENABLE          = 64
    ADDR_LED_ENABLE             = 65
    ADDR_GOAL_POSITION          = 116
    ADDR_MOVING                 = 122
    ADDR_GOAL_CURRENT           = 102
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4        # Data Byte Length
    # DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    # DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000   # Default = 57600 bps   1000000
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    LEN_GOAL_POSITION           = 4
    ADDR_PRESENT_POSITION       = 611
    LEN_PRESENT_POSITION        = 4
    # DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    # DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    LEN_GOAL_POSITION           = 4          # Data Byte Length
    ADDR_PRESENT_POSITION       = 580
    LEN_PRESENT_POSITION        = 4          # Data Byte Length
    # DXL_MINIMUM_POSITION_VALUE  = -150000    # Refer to the Minimum Position Limit of product eManual
    # DXL_MAXIMUM_POSITION_VALUE  = 150000     # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000
elif MY_DXL == 'XL330':
    ADDR_OPERATING_MODE         = 11
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4
    ADDR_LED_ENABLE             = 65
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4        # Data Byte Length
    BAUDRATE                    = 115200
    ADDR_TORQUE_ENABLE          = 64
TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # dynamixel moving status threshold


# Initialize PortHandler instance
# Set the port path
# Get methods _and_ members of PortHandlerLinux or PortHandlerWindows
# DEVICENAME = '/dev/ttyUSB0'
DEVICENAME = 'COM4'
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods _and_ members of Protocol1PacketHandler or Protocol2PacketHandler
PROTOCOL_VERSION = 2.0
packetHandler = PacketHandler(PROTOCOL_VERSION)
