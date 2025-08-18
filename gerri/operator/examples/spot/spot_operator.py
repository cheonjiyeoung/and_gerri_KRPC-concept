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
from gerri.operator.examples.spot.spot_config import ROBOT_INFO, OPERATOR_INFO, VIDEO_INFO, SPOT_INFO, GERRI_ROBOT_INFO
from PySide6.QtWidgets import QApplication
import logging

# 로그 레벨을 ERROR로 설정하여 WARNING 이하의 로그를 표시하지 않음
logging.basicConfig(level=logging.ERROR)
webrtc_daemon = WebRtcOperatorCommand(ROBOT_INFO,OPERATOR_INFO)
webrtc_daemon.connect()

app = QApplication(sys.argv)

from gerri.operator.interface.main_ui import MainUI
from gerri.operator.examples.spot.spot_ui.main_ui import SpotOperationUI
from _and_.keti_rtc.operator.webrtc_operator_media_receiver import OperatorMediaReceiever


# ptz_cam_receiver = OperatorMediaReceiever(robot_info=ROBOT_INFO, operator_info=OPERATOR_INFO,
#                                                  channel_info="front_cam",gerri=False)

ptz_cam_receiver = OperatorMediaReceiever(robot_info=GERRI_ROBOT_INFO, operator_info=OPERATOR_INFO,
                                                 channel_info="ptz_cam")
rest_cam_receiver = OperatorMediaReceiever(robot_info=GERRI_ROBOT_INFO, operator_info=OPERATOR_INFO,
                                                 channel_info="rest_cam")


ptz_cam_receiver.connect()
rest_cam_receiver.connect()
channels = {
    "command": webrtc_daemon,   
    "ptz_cam": ptz_cam_receiver,
    "rest_cam": rest_cam_receiver
}

from gerri.operator.examples.spot.spot_base_commander import SpotBaseCommander
from gerri.operator.examples.spot.spot_sub_commander import SpotSubCommander
# If the model is not predefined in SampleBaseCommander,
# we explicitly assign a sub-function implementation.
spot_base_commander = SpotBaseCommander(ROBOT_INFO, OPERATOR_INFO)
spot_sub_commander = SpotSubCommander(spot_info = SPOT_INFO, channels=channels, robot_info=ROBOT_INFO, operator_info=OPERATOR_INFO)

spot_base_commander.sub_commander = spot_sub_commander
spot_sub_commander.base_commander = spot_base_commander
spot_base_commander.connect()

from gerri.operator.examples.spot.spot_ui.utils.spot_status import SpotStatus
status = SpotStatus(SPOT_INFO)
screen = app.primaryScreen()
size = screen.size()
width = size.width()
height = size.height()
window =SpotOperationUI(w=width,h=height,webrtc_channels=channels,spot_info=SPOT_INFO,status=status)
main_window = MainUI(ui=window)
main_window.setWindowTitle("Hello Universe Operator")
main_window.show()

sys.exit(app.exec())
