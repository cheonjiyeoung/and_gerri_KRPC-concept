"""
Main entry point for Hello Universe operator UI.
- Connects to the robot using AND
- Sets up command system (GERRI)
- Launches keyboard/mouse input interface
"""

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.keti_rtc.webrtc_operator import WebRtcOperator
# from _and_.keti_rtc.webrtc_operator_video import WebRtcOperatorVideo
from construction_config import ROBOT_INFO, OPERATOR_INFO, VIDEO_INFO, AUDIO_INFO
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from PySide6.QtGui import QPalette, QColor
import rclpy
from rclpy.node import Node
import time
import logging

logging.basicConfig(level=logging.WARN)
webrtc_daemon = WebRtcOperator(ROBOT_INFO,OPERATOR_INFO)
webrtc_daemon.connect()

time.sleep(1)


app = QApplication(sys.argv)
palette = QPalette()
palette.setColor(QPalette.Window, QColor(50, 50, 50))
palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
palette.setColor(QPalette.Button, QColor(70, 70, 70))
palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
palette.setColor(QPalette.Base, QColor(40, 40, 40))
palette.setColor(QPalette.Text, QColor(230, 230, 230))
app.setPalette(palette)

rclpy.init()

from gerri.operator.examples.construction_operator.construction_base_commander import ConstructionBaseCommander
from gerri.operator.examples.construction_operator.construction_sub_commander import ConstructionSubCommander
# If the model is not predefined in SampleBaseCommander,
# we explicitly assign a sub-commander implementation.
gerri_operator = ConstructionBaseCommander(ROBOT_INFO, sub_commander=ConstructionSubCommander(app=app))
gerri_operator.connect()
sys.exit(app.exec())

