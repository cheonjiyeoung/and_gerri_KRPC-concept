from pubsub import pub


class RobotBridge:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.robot_info = None

        pub.subscribe(self.handle_bridge_message, "receive_message_bridge")

    def connect(self):
        pass

    def disconnect(self):
        pass

    def send_message(self, message):
        pass

    def handle_bridge_message(self, message):
        if 'robot_info' in message:
            if self.robot_name == message['robot_info']['robot_name']:
                self.robot_info = message['robot_info']


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
