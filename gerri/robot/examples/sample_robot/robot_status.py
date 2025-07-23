class RobotStatus:
    def __init__(self,robot_id=None,model=None,category=None):
        self.robot_id = robot_id
        self.robot_type = {
            "category": category,
            "model": model,
            "footprint": None,
            "size": {
                "width": None,
                "length": None,
                "height": None
            }
        }

        self.pose = {
            "position": {
                "x": None,
                "y": None,
                "z": None
            },
            "orientation": {
                "x":None,
                "y":None,
                "z":None,
                "w":None
            },
            "2d":{"x":None,
                  "y":None,
                  "th":None}
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
        }
        self.battery_state = {
            "voltage": None, # ok
            "current": None,
            "capacity": None,
            "percentage": None, # ok
            "temperature": None, 
            "power_supply_status": None,
            "now_charging": None # ok
        }
        self.joint_state = {
            "name": None,
            "position": None,
            "velocity": None,
            "effort": None
        }
        self.sensor = {
            "ultrasonic": None,
            "laser": None
        }
        self.map = {
            "name": None,
            "origin": None,
            "size": None,
            "scale": None
        }
