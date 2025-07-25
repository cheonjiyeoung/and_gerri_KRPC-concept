from PySide6.QtCore import Qt
import pickle
from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from _and_.keti_rtc.operator.webrtc_operator_media_receiver import OperatorMediaReceiever
from gerri.operator.interface.sample_ui.sample_operator_ui import SampleOperatorUI
from gerri.operator.interface.main_ui import MainUI

class SampleSubCommander:
    def __init__(self, **kwargs):
        self.control_target = 'all'
        self.use_master_arm = False
        self.master_control = False
        self.base_commander = None
        self.app = kwargs.get('app')

        pub.subscribe(self.key_mouse_control, 'key_mouse_control')
        pub.subscribe(self.ui_signal, 'ui_signal')
    """
    Initializes the connection for the sub-function (e.g., hardware setup, activation).
    """
    def connect(self):
        print('Connecting to Gerri...')
    """
    Handles cleanup and shutdown for the sub-function (if applicable).
    """
    def disconnect(self):
        pass
        ### TODO : ADD DISCONNECT FUNCTION

    def get_status(self, value):
        pass

    def ui_signal(self,signal):
        vx=0
        vy=0
        vth=0
        if signal == "clicked_W":
            vx=1
        if signal == "clicked_A":
            vy=-1
        if signal == "clicked_S":
            vx=-1
        if signal == "clicked_D":
            vy=1
        if signal == "clicked_Q":
            vth=-1
        if signal == "clicked_E":
            vth=1
        value = {"vx":vx,"vy":vy,"vth":vth}
        self.base_commander.hello_universe("hello")

    """
    Clamp a given value to a specified min-max range.
    Optionally constrain further with an absolute limit.
    """
    def clamp(self, value, min_value, max_value, absolute_limit=None):
        """
        값을 주어진 범위로 제한 (클램핑).

        :param value: 제한할 값
        :param min_value: 최소값
        :param max_value: 최대값
        :param absolute_limit: 절대 제한 (튜플, optional)
        :return: 제한된 값
        """
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))

    """
    Maps a value from one range into another using linear scaling.
    """
    def map_value(self, value, in_min, in_max, out_min, out_max):
        """
        특정 값을 주어진 범위 내에서 다른 범위로 매핑하는 함수.

        :param value: 매핑할 값
        :param in_min: 매핑 전 최소값
        :param in_max: 매핑 전 최대값
        :param out_min: 매핑 후 최소값
        :param out_max: 매핑 후 최대값
        :return: 매핑된 값
        """
        map_value = self.clamp((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max)

        return map_value

    """
    Handles key and mouse input events and maps them to robot commands.
    """
    def key_mouse_control(self, command):
        # print(command)
        key = command['key_control']
        mouse_d_move = command['mouse_d_move']
        mouse_d_wheel = command['mouse_d_wheel']
        mouse_click = command['mouse_click']

        if self.base_commander:

            if "RETURN" in key:
                self.base_commander.hello_universe(message='Hello Universe!')
