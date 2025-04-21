import sys

from gerri_config import ROBOT_ID, OPERATOR_ID
from webrtc_tools.webrtc_operator_bridge import WebRtcBridgeOperator
from PySide6.QtWidgets import QApplication

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.keyboard_mouse_controller import KeyboardMouseController
from gerri.operator.manipulator_operator import ManipulatorOperator
from gerri.operator.examples.sample_operator import SampleOperator


piper_master = ManipulatorOperator()

robot_group = {
            'manipulator': piper_master,
            # 'pan_tilt': pantilt,
}

# robot_group = {"SAMPLE_OPERATOR": SampleOperator()}



class AvatarOperator:
    def __init__(self, controller, network):
        self.controller = controller
        self.network = network
        self.target = None

    def connect(self):
        if self.controller:
            for controller in self.controller.values():
                print(controller)
                controller.connect()
        else:
            sample_controller = SampleOperator()
            sample_controller.connect()
        self.network.connect()

    def start(self):
        self.connect()
        app = QApplication(sys.argv)
        controller = KeyboardMouseController(robot_camera, 100)
        controller.show()
        sys.exit(app.exec())



if __name__ == '__main__':
    robot_id = ROBOT_ID

    robot_controller_ID = ROBOT_ID+"_command"
    robot_camera = ROBOT_ID+"_video"

    server_host = '175.126.123.199'
    server_port = 8180

    POLLING_RATE_MS = 100

    # manipulator_operator = ManipulatorOperator()
    webrtc_bridge_operator = WebRtcBridgeOperator(robot_controller_ID,OPERATOR_ID)

    avatar_operator = AvatarOperator(robot_group, webrtc_bridge_operator)
    avatar_operator.start()



