from PySide6.QtWidgets import QApplication

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.keti_rtc.webrtc_operator import WebRtcOperator
from hello_universe_config import ROBOT_INFO, OPERATOR_INFO

webrtc_daemon = WebRtcOperator(ROBOT_INFO,OPERATOR_INFO)
webrtc_daemon.connect()


from gerri.operator.commander.manipulator_commander import ManipulatorCommander

gerri_operator = ManipulatorCommander(ROBOT_INFO)
gerri_operator.connect()

from gerri.operator.interface.keyboard_mouse_controller import KeyboardMouseController

app = QApplication(sys.argv)
operator_controller = KeyboardMouseController(ROBOT_INFO['id'])
operator_controller.show()
sys.exit(app.exec())

