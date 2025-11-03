class RobotStatus:
    """
    Basic robot status data format.

    This class provides a convenient default structure for representing
    robot status information such as pose, velocity, battery state, sensors,
    and map data. You can use this format as-is or extend it by adding
    custom attributes according to your robot's requirements.
    """
    
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
            "voltage": None,
            "current": None,
            "capacity": None,
            "percentage": None,
            "temperature": None, 
            "power_supply_status": None,
            "now_charging": None
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