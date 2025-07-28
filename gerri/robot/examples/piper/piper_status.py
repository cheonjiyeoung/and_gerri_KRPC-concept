class RobotStatus:
    def __init__(self,robot_id=None,model=None,category=None):
        self.robot_id = robot_id
        self.robot_type = {
            "category": category,
            "model": model
        }

        self.joint_state = {
            "name": None,
            "position": None,
            "velocity": None,
            "effort": None
        }