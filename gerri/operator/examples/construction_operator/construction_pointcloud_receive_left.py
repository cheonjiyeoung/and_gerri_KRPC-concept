"""
Main entry point for Hello Universe operator UI.
- Connects to the robot using AND
- Sets up command system (GERRI)
- Launches keyboard/mouse input interface
"""

from PySide6.QtWidgets import QApplication
import logging
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
logging.basicConfig(level=logging.WARN)
from _and_.keti_rtc.webrtc_operator import WebRtcOperator
from _and_.keti_rtc.webrtc_operator_video import WebRtcOperatorVideo
from construction_config import OPERATOR_INFO
from PySide6.QtWidgets import QApplication, QMainWindow
import rclpy
from rclpy.node import Node

ROBOT_INFO = {
    "id": "construction_pointcloud_left",
    "model": "doosanM1509",
    "category": "sample",
}

webrtc_daemon = WebRtcOperator(ROBOT_INFO,OPERATOR_INFO)
webrtc_daemon.connect()

rclpy.init()

from gerri.operator.examples.construction_operator.construction_base_pointcloud_receiver_left import ConstructionBasePointcloudReceiver

# gerri_operator = SampleBaseCommander(ROBOT_INFO)
# gerri_operator.connect()

# If the model is not predefined in SampleBaseCommander,
# we explicitly assign a sub-function implementation.
gerri_operator_pointcloud = ConstructionBasePointcloudReceiver(ROBOT_INFO, sub_commander=None)
gerri_operator_pointcloud.connect()


