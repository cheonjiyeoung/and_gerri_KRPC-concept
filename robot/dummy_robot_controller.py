class DummyController:
    def __init__(self):
        """가짜 컨트롤러 (테스트용)"""
        self.robot_name = "TestBot"
        self.robot_type = {'category': 'mobile',
                           'model': 'gyd'}
        self.robot_state = {'mode': 'auto', }
        self.pose = [0.0, 0.0, 0.0]
        self.battery = 100
        self.robot_shape = "quadruped"
        self.wheel_state = "stopped"
        self.joint_state = [0.0] * 6
        self.velocity = {'2d': [0.0] * 6}


    def connect(self):
        pass

    def disconnect(self):
        pass
