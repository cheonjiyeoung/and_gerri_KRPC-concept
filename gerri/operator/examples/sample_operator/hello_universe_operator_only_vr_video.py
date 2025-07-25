"""
Main entry point for Hello Universe operator UI.
- Connects to the robot using AND
- Sets up command system (GERRI)
- Launches keyboard/mouse input interface
"""

from PySide6.QtWidgets import QApplication

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

# from _and_.keti_rtc.webrtc_operator import WebRtcOperator
from _and_.keti_rtc.webrtc_operator_video import WebRtcOperatorVideo
from hello_universe_config import ROBOT_INFO, OPERATOR_INFO


webrtc_daemon = WebRtcOperatorVideo(ROBOT_INFO,OPERATOR_INFO, video_info='front_cam')
webrtc_daemon.connect()

from gerri.operator.examples.sample_operator.sample_base_commander import SampleBaseCommander

# gerri_operator = SampleBaseCommander(ROBOT_INFO)
# gerri_operator.connect()

from gerri.operator.examples.sample_operator.sample_sub_commander import SampleSubCommander
# If the model is not predefined in SampleBaseCommander,
# we explicitly assign a sub-function implementation.
gerri_operator = SampleBaseCommander(ROBOT_INFO, sub_commander=SampleSubCommander())
gerri_operator.connect()

from gerri.operator.interface.vr_video_player import VrVideoPlayer

app = QApplication(sys.argv)
operator_controller = VrVideoPlayer()
operator_controller.show()
sys.exit(app.exec())

