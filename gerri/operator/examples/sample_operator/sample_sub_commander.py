import time
from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))


class SampleCommander:
    def __init__(self, base_commander, **kwargs):
        self.base_commander = base_commander
        self.control_target = 'all'
        self.use_master_arm = False
        self.master_control = False

        pub.subscribe(self.key_mouse_control, 'key_mouse_control')

    def connect(self):
        print('Connecting to Gerri...')


    def disconnect(self):
        pass
        ### TODO : ADD DISCONNECT FUNCTION


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

    def key_mouse_control(self, command):
        # print(command)
        key = command['key_control']
        mouse_d_move = command['mouse_d_move']
        mouse_d_wheel = command['mouse_d_wheel']
        mouse_click = command['mouse_click']


        if "RETURN" in key:
            self.base_commander.get_robot_status(target=self.control_target)
            self.base_commander.send_message({'topic':'test', 'value':key, 'target':self.control_target})



    def limit_e_stop(self, joint_value, joint_limit):
        """
        조인트 값이 리미트를 초과하면 긴급 정지.

        :param joint_value: 조인트 값 리스트
        :param joint_limit: 리미트 범위 리스트
        :return: True (긴급 정지 필요), False (정상 범위)
        """
        # for idx, value in enumerate(joint_value):
        #     min_limit, max_limit = joint_limit[idx]
        #     if value < min_limit:
        #         value = min_limit
        #     elif value > max_limit:
        #         value = max_limit
        #         # self.disable_master_arm()
        #         print(
        #             f"JOINT LIMIT EXCEEDED: Joint {idx + 1} | Value: {value} | Limits: [{min_limit}, {max_limit}]")
        #         # return True
        return False
