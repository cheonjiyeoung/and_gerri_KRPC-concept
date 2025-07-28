import numpy as np
import time
import math
from gerri.robot.examples.pantilt_2kng.pan_tilt_controller import PanTiltController
from gerri.robot.examples.pantilt_2kng.pan_tilt_status import RobotStatus
import threading

# Piper 인터페이스 초기화 및 연결
FACTOR_DEGREE = 1000 # 1도는 1000펄스임

class PanTiltSubController:
    def __init__(self, port):
        self.robot = PanTiltController(port)

        self.status.joint_state = {
            'name': ['pan', 'tilt'],
            'position': [int(self.map_value(self.robot.last_pan_value, 0, 4096, -180, 180)),
                         int(self.map_value(self.robot.last_tilt_value, 0, 4096, -180, 180))]
        }

    def init_base_controller(self, base_controller):
        self.status = RobotStatus(robot_id=base_controller.robot_info["id"],
                                  model=base_controller.robot_info["model"],
                                  category=base_controller.robot_info["category"],)

        self._status_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._lock = threading.Lock()
        self._status_thread.start()


    def _update_loop(self):
        while True:
            self._lock.acquire()
            ### STATUS UPDATE
            self.status.joint_state = [
                int(self.map_value(self.robot.last_pan_value, 0, 4096, -180, 180)),
                int(self.map_value(self.robot.last_tilt_value, 0, 4096, -180, 180))
            ]
            self._lock.release()
            time.sleep(1)

    def pan_tilt(self, pan_tilt_angle):
        pan_angle = pan_tilt_angle[0]
        tilt_angle = pan_tilt_angle[1]

        pan_value = int(self.map_value(pan_angle, -180, 180, 0, 4096))
        tilt_value = int(self.map_value(tilt_angle, -180, 180, 0, 4096))
        if self.robot.last_pan_value != pan_value or self.robot.last_tilt_value != tilt_value:
            self.robot.pan_tilt_control(pan_value, tilt_value)

    def pan_tilt_step(self, pan_tilt_step_angle, scaler = 100):
        pan_step = pan_tilt_step_angle[0] * scaler
        tilt_step = pan_tilt_step_angle[1] * scaler
        if pan_step != 0 or tilt_step != 0:
            self.robot.pan_tilt_control(self.robot.last_pan_value - pan_step, self.robot.last_tilt_value - tilt_step)

    def home_position(self):
        self.robot.pan_tilt_control(self.robot.default_pan_value, self.robot.default_tilt_angle)

    def connect(self):
        self.robot.connect()

    def disconnect(self):
        self.robot.disconnect()

    @staticmethod
    def clamp(value, min_value, max_value, absolute_limit=None):
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))

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


    def deg2rad(self, degrees):
        """리스트든 단일 값이든 degree(도)를 radian(라디안)으로 변환"""
        if isinstance(degrees, (int, float)):  # 단일 값
            return degrees * (math.pi / 180)
        elif isinstance(degrees, (list, tuple, set)):  # 여러 개의 값 (배열)
            return [deg * (math.pi / 180) for deg in degrees]
        else:
            raise TypeError("Input must be an int, float, list, tuple, or set")


    def rad2deg(self, radians):
        """리스트든 단일 값이든 radian(라디안)을 degree(도)로 변환"""
        if isinstance(radians, (int, float)):  # 단일 값
            return radians * (180 / math.pi)
        elif isinstance(radians, (list, tuple, set)):  # 여러 개의 값 (배열)
            return [rad * (180 / math.pi) for rad in radians]
        else:
            raise TypeError("Input must be an int, float, list, tuple, or set")

    def custom_command(self, command):
        print("CUSTOM COMMAND")
