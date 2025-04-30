class SampleSubController:
    def __init__(self):
        """
        ROBOT INIT CODE
        """
        self.robot_state = 'idle'
        self.pose = {
            'position': [0, 0, 0],
            'orientation': [0, 0, 0],
        }
        self.velocity = {
            'linear': [0, 0, 0],
            'angular': [0, 0, 0],
        }
        self.battery_state = {
            'voltage': 0,
            'current': 0,
            'percentage': 0,
            'temperature': 0,
            'capacity': 0,
        }
        self.joint_state = {
            'name': ['left_wheel', 'right_wheel', 'left_arm', 'right_arm'],
            'position': [0, 0, 0, 0],
            'velocity': [0, 0, 0, 0],
            'effort': [0, 0, 0, 0],
        }
        self.sensor = {
            'ultrasonic': [0, 0, 0],
            'laser': [0, 0, 0],
        }
        self.path_plan = {
            'global': [],
            'local': [],
        }
        self.map = {
            'name': 'default',
            'origin': [0, 0],
            'size': [0, 0],
            'scale': 1
        }

    def get_joint_angles(self):
        """
        GET ROBOT JOINT ANGLES
        """

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

    def send_message(self, message):
        pass

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
