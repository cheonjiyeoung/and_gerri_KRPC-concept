from robot_status import RobotStatus
from hello_universe_config import ROBOT_INFO
import threading
import random
import time

class SampleSubController:
    def __init__(self):
        robot_id = ROBOT_INFO["id"]
        robot_model= ROBOT_INFO["model"]
        robot_category=ROBOT_INFO["category"]
        self.status = RobotStatus(robot_id=robot_id,
                                  model=robot_model,
                                  category=robot_category)

        self._status_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._status_thread.start()
        self._lock = threading.Lock

    def init_base_controller(self, base_controller):
        self.base_controller = base_controller

    def get_joint_angles(self):
        """
        GET ROBOT JOINT ANGLES
        """

    def _update_loop(self):
        while True:
            self._lock.acquire()
            # For examples #
            self.status.pose["2d"]["x"] = random.randint(0,15)
            self.status.pose["2d"]["y"] = random.randint(0,15)
            self.status.pose["2d"]["th"] = random.randint(0,15)
            self._lock.release()
            time.sleep(0.1)

    def connect(self):
        """
        CONNECT TO ROBOT
        """

    def disconnect(self):
        """
        DISCONNECT FROM ROBOT
        """

    def joint_ctrl(self, joint_angles: list):
        """
        ROBOT JOINT CONTROL BY DEGREE
        """

    def joint_ctrl_step(self, joint_angles: list):
        """
        ROBOT JOINT CONTROL BY DEGREE
        변화량 만큼 이동하는 코드 예 (0,0,0,0,30,0) 5번축만 +30도 회전
        """

    def joint_ctrl_puppet(self, master_joint_angles):
        """
        마스터 컨트롤러가 있을 떄 조인트정보를받고 움직이는 코드

        """

    def gripper_ctrl(self, gripper_angle):
        """
        페러럴 그리퍼 컨트롤
        """

    def update_status(self):
        print('update status')
        return 'hi'

    def hello_universe(self, value):
        print('hello_universe')
        return 'hello_universe_by_robot'

    def custom_command(self, command):
        return command

    def send_message(self, message):
        self.base_controller.send_message(message)

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
