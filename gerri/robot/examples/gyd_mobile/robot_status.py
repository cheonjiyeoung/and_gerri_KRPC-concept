class RobotStatus:
    def __init__(self, robot_id=None, model=None, category=None):
        self.robot_id = robot_id
        self.robot_type = {
            "category": category,
            "model": model,
            "footprint": None,
            "size": {
                "width": 360,
                "length": 540,
                "height": 840
            }
        }
        self.robot_state = {
            "mode":"AUTO",
            "operating_state":["IDLE"]
        }

        # ZYX
        self.pose = {
            "position": {
                "x": None,
                "y": None,
                "z": None
            },
            "orientation": {
                "yaw": None,
                "pitch": None,
                "roll": None,
            },
            "2d": {
                "x": None,
                "y": None,
                "th": None
            }
        }
        self.velocity = {
            "linear": {
                "x": None,
                "y": None,
                "z": None
            },
            "angular": {
                "x": None,
                "y": None,
                "z": None
            },
            "2d": {
                "vx": None,
                "vy": None,
                "vth": None,
            }
        }
        self.battery_state = {
            "voltage": None,
            "current": None,
            "capacity": None,
            "percentage": None,
            "temperature": None,
            "now_charging": None
        }
        self.joint_state = {
            "name": None,
            "position": None,
            "velocity": None,
            "effort": None
        }
        self.path_plan = {
            "global": None,
            "local": None,
        }
        self.sensor = {
            "ultrasonic": None,
            "laser": None
        }
        self.map = {
            "name": "KETI-SUSEO2F-3F",
            "origin": (36,377),
            "size": (1477,654),
            "resolution": 0.05000000074505806
        }

        self.nav_state = {"goal":None,"result":None,"info":None}

        # self.current_floor