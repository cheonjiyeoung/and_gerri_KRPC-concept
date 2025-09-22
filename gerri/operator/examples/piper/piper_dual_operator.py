"""
Main entry point for Hello Universe operator UI.
- Connects to the robot using AND
- Sets up command system (GERRI)
- Launches keyboard/mouse input interface
"""
import time

from PySide6.QtWidgets import QApplication

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.keti_rtc.operator.webrtc_operator_command import WebRtcOperatorCommand
from gerri.operator.examples.piper.piper_dual_operator_config import ROBOT_INFO, OPERATOR_INFO, MASTER_ARM_LEFT_INFO, MASTER_ARM_RIGHT_INFO

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from PySide6.QtGui import QPalette, QColor

webrtc_daemon = WebRtcOperatorCommand(ROBOT_INFO,OPERATOR_INFO)
webrtc_daemon.connect()

app = QApplication(sys.argv)

from gerri.operator.examples.piper.piper_base_commander import PiperBaseCommander
from gerri.operator.examples.piper.piper_sub_commander import PiperSubCommander

# LEFT ARM SETUP

piper_left_base_commander = PiperBaseCommander(ROBOT_INFO, OPERATOR_INFO)
piper_left_sub_commander = PiperSubCommander(master_arm=MASTER_ARM_LEFT_INFO)

piper_left_base_commander.sub_commander = piper_left_sub_commander
piper_left_sub_commander.base_commander = piper_left_base_commander
piper_left_base_commander.connect()

# RIGHT ARM SETUP

piper_right_base_commander = PiperBaseCommander(ROBOT_INFO, OPERATOR_INFO)
piper_right_sub_commander = PiperSubCommander(master_arm=MASTER_ARM_RIGHT_INFO)

piper_right_base_commander.sub_commander = piper_right_sub_commander
piper_right_sub_commander.base_commander = piper_right_base_commander
piper_right_base_commander.connect()

from gerri.operator.interface.main_ui import MainUI
from gerri.operator.interface.sample_ui.sample_operator_ui import SampleOperatorUI
from _and_.keti_rtc.operator.webrtc_operator_media_receiver import OperatorMediaReceiever

front_cam_receiver = OperatorMediaReceiever(robot_info=ROBOT_INFO, operator_info=OPERATOR_INFO, channel_info="front_cam")

front_cam_receiver.connect()
receivers = [front_cam_receiver]
window =SampleOperatorUI(1280, 720, receivers)
main_window = MainUI(ui=window, polling_rate_ms=50)
main_window.setWindowTitle("Hello Universe Operator")
main_window.show()

sys.exit(app.exec())


while True:
    time.sleep(1)