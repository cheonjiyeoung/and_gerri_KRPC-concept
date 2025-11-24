"""
Main entry point for Hello Universe operator UI.
- Connects to the robot using AND
- Sets up command system (GERRI)
- Launches keyboard/mouse input interface
"""

from PySide6.QtWidgets import QApplication

import sys
from and_gerri._and_.keti_rtc.operator.webrtc_operator_command import WebRtcOperatorCommand
from gyd.operator_20251124085840.gyd_config import ROBOT_INFO, OPERATOR_INFO, VIDEO_INFO, AUDIO_INFO
import logging

logging.basicConfig(level=logging.WARNING)
webrtc_daemon = WebRtcOperatorCommand(ROBOT_INFO,OPERATOR_INFO)
webrtc_daemon.connect()

app = QApplication(sys.argv)
from and_gerri.gerri.operator.interface.keyboardmousecontroller import KeyboardMouseController
from and_gerri.gerri.operator.interface.common_qt_ui import CommonQtUI
from and_gerri._and_.keti_rtc.operator.webrtc_operator_media_receiver import OperatorMediaReceiever

front_cam_receiver = OperatorMediaReceiever(robot_info=ROBOT_INFO, operator_info=OPERATOR_INFO,
                                                 channel_info="front_cam")
rear_cam_receiver = OperatorMediaReceiever(robot_info=ROBOT_INFO, operator_info=OPERATOR_INFO,
                                                 channel_info="rear_cam")

front_cam_receiver.connect()
rear_cam_receiver.connect()
channels = {
    "command": webrtc_daemon,   
    "front_cam": front_cam_receiver,
    "rear_cam": rear_cam_receiver
}

from gyd.operator_20251124085840.gyd_commander import GydCommander
# If the model is not predefined in SampleBaseCommander,
# we explicitly assign a sub-function implementation.

gyd = GydCommander(channels=channels, robot_info=ROBOT_INFO, operator_info=OPERATOR_INFO)
gyd.connect()

screen = app.primaryScreen()
size = screen.size()
width = size.width()
height = size.height()
window =CommonQtUI(w=width,h=height,webrtc_channels=channels,commander=gyd)
main_window = KeyboardMouseController(ui=window)
main_window.setWindowTitle("Hello Universe Operator")
main_window.show()

sys.exit(app.exec())