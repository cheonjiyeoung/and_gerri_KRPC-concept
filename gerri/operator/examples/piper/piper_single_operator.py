"""
Main entry point for Hello Universe operator UI.
- Connects to the robot using AND
- Sets up command system (GERRI)
- Launches keyboard/mouse input interface
"""

from PySide6.QtWidgets import QApplication

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.keti_rtc.operator.webrtc_operator_command import WebRtcOperatorCommand
from hello_universe_config import ROBOT_INFO, OPERATOR_INFO, VIDEO_INFO
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from PySide6.QtGui import QPalette, QColor

webrtc_daemon = WebRtcOperatorCommand(ROBOT_INFO,OPERATOR_INFO)
webrtc_daemon.connect()

app = QApplication(sys.argv)

from gerri.operator.examples.piper.piper_base_commander import PiperBaseCommander
from gerri.operator.examples.piper.piper_sub_commander import PiperSubCommander

# ARM SETUP

piper_base_commander = PiperBaseCommander(ROBOT_INFO, OPERATOR_INFO)
piper_sub_commander = PiperSubCommander()

# WITH MASTER
# from gerri.operator.examples.piper.piper_single_operator_config import MASTER_ARM_INFO
# piper_sub_commander = PiperSubCommander(master_arm=MASTER_ARM_INFO)

piper_base_commander.sub_commander = piper_sub_commander
piper_sub_commander.base_commander = piper_base_commander
piper_base_commander.connect()

from gerri.operator.interface.main_ui import MainUI
from gerri.operator.interface.sample_ui.sample_operator_ui import SampleOperatorUI
from _and_.keti_rtc.operator.webrtc_operator_media_receiver import OperatorMediaReceiever

front_cam_receiver = OperatorMediaReceiever(robot_info=ROBOT_INFO, operator_info=OPERATOR_INFO, channel_info="front_cam")

front_cam_receiver.connect()
receivers = [front_cam_receiver]
window =SampleOperatorUI(1280, 720, receivers)
main_window = MainUI(ui=window)
main_window.setWindowTitle("Hello Universe Operator")
main_window.show()

sys.exit(app.exec())
