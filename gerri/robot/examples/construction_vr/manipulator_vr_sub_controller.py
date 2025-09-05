import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.construction_vr.robot_status import RobotStatus
from gerri.robot.examples.sample_robot.sample_robot_controller import SampleRobotController
from gerri.robot.examples.construction_vr.doosan_controller import DoosanController
import threading
import random
import time

class DoosanSubController:
    def __init__(self, ip, port):
        self.robot_controller = DoosanController(ip, port)
        self.base_controller = None
        self._lock = threading.Lock()

        self.status = None

    def connect(self):
        self.status = RobotStatus(robot_id=self.base_controller.robot_id,
                                  model=self.base_controller.robot_model,
                                  category=self.base_controller.robot_category)
        threading.Thread(target=self._update_loop,daemon=True).start()

    def _update_loop(self):
        while True:
            self._lock.acquire()
            # For examples #
            self.status.joint_state =self.robot_controller.joint_state
            self.status.pose = self.robot_controller.pose
            ###
            self._lock.release()
            time.sleep(0.1)

    def disconnect(self):
        """
        DISCONNECT FROM ROBOT
        """

    def joint_ctrl(self, value, option):
        """
        ROBOT JOINT CONTROL BY DEGREE
        """
        print("Joint Ctrl in Sub", value, option)

    def joint_ctrl_step(self, value, option):
        """
        ROBOT JOINT CONTROL BY DEGREE
        변화량 만큼 이동하는 코드 예 (0,0,0,0,30,0) 5번축만 +30도 회전
        """
        print("joint_ctrl_step in Sub")

    def joint_ctrl_puppet(self, value, option):
        """
        마스터 컨트롤러가 있을 떄 조인트정보를받고 움직이는 코드

        """
        print("joint_ctrl_puppet in Sub", value, option)


    def gripper_ctrl(self, value, option):
        """
        페러럴 그리퍼 컨트롤
        """
        print("gripper_ctrl in Sub", value, option)

    def gripper_ctrl_puppet(self, value, option):
        """
        페러럴 그리퍼 컨트롤
        """
        print("gripper_ctrl in Sub", value, option)

    def move(self, value, option):
        print("move", value, option)

    def hello_universe(self, value):
        print('hello_universe')
        self.send_message(value)
        return 'hello_universe_by_robot'

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
