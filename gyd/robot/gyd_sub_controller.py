"""
RobotSubController is wraping your robot control code for follow and-gerri rules
TODO:
    1. You must import your own robot control code. (SDK or Written by robot manual.)
    2. Just coding on owr pre-defined methods (Not required, but recommended.)
    3. coding another methods to control your robot by KRPC Via and-gerri
"""

import threading
import time
from pubsub import pub
import json

# You must import your robot controll code in here
# from test.robot.test_robot_controller import TestRobotController

class DummyController:
    def __getattr__(self, name):
        raise NotImplementedError(
            f"Robot controller is not set. "
            f"Replace DummyController with your real robot SDK controller."
        )
    
class RobotStatus:
    """
    Note:
    RobotStatus is common status following ISO-21413(future)
    This is not strictly required, but if you want to fully utilize our solution,
    we recommend updating your robot status in this standard format.
    You may freely add additional status fields if your robot provides more data.
    """
    def __init__(self, robot_id=None, model=None, category=None):
        self.robot_id = robot_id
        self.robot_type = {
            "category": category,
            "model": model,
            "footprint": None,
            "size": {"width": None,"length": None,"height": None}
        }
        self.pose = {
            "position": {"x": None,"y": None,"z": None},
            "orientation": {"yaw": None,"pitch": None,"roll": None},
            "2d": {"x": None,"y": None,"th": None}
        }
        self.velocity = {
            "linear": {"x": None,"y": None,"z": None},
            "angular": {"x": None,"y": None,"z": None},
            "2d": {"vx": None,"vy": None,"vth": None,}
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
            "name": [],
            "position": [],
            "velocity": [],
            "effort": []
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
            "name": None,
            "origin": None,
            "size": None,
            "resolution": None
        }

MAX_LINEAR_VELOCITY = 1.0
MIN_LINEAR_VELOCITY = 0.1
MAX_ANGULAR_VELOCITY = 1.0
MIN_ANGULAR_VELOCITY = 0.1

class GydSubController:
    def __init__(self,robot_id,robot_model,robot_category):
        self.robot_id = robot_id
        self.robot_model = robot_model
        self.robot_category = robot_category
        self.robot_controller = DummyController() # MUST REPLACE THIS
        self.status = None
        self._lock = threading.Lock()

        self.current_linear_velocity = round(MAX_LINEAR_VELOCITY / 2, 2)
        self.current_angular_velocity = round(MAX_ANGULAR_VELOCITY / 2, 2)

        self.operation_mode = "AUTO"


    def _update_loop(self):
        while True:
            with self._lock:
                # if you want use all features we provide, please implement the logic to update your robot’s status information in our standard format.
                time.sleep(0.1)

    def _connect(self):
        self.status = RobotStatus(robot_id=self.robot_id,
                                  model=self.robot_model,
                                  category=self.robot_category)
        threading.Thread(target=self._update_loop,daemon=True).start()
        """
        TODO: code initialize function(release the brake, power on motor, get lease ... ) here.
        """

    def _disconnect(self):
        """
        DISCONNECT FROM ROBOT
        """

    def _send_message(self, message):
        # message will send by pypubsub to operator
        if type(message) == dict:
            message = json.dumps(message)
        pub.sendMessage("send_message",message=message)


# You can fold here :)
# region PreDefinedRobotControl
# <editor-fold desc="Robot Control Methods">
    def move_forward(self):
        """
        TODO: code "forward movement" function here.
        ex) self.robot.move(vx=self.current_linear_velocity, vy=0.0, vth=0.0)
        """
        pass

    def move_backward(self):
        """
        TODO: code "backward movement" function here.
        ex) self.robot.move(vx=self.current_linear_velocity * -1, vy=0.0, vth=0.0)
        """
        pass

    def shift_left(self):
        """
        TODO: code "left movement" function here.
        ex) self.robot.move(vx=0.0, vy=self.current_linear_velocity, vth=0.0)
        """
        pass

    def shift_right(self):
        """
        TODO: code "right movement" function here.
        ex) self.robot.move(vx=0.0, vy=self.current_linear_velocity * -1, vth=0.0)
        """
        pass

    def turn_left(self):
        """
        TODO: code "turn left movement" function here.
        ex) self.robot.move(vx=0.0, vy=0.0, vth=self.current_angular_velocity)
        """
        pass

    def turn_right(self):
        """
        TODO: code "turn right movement" function here.
        ex) self.robot.move(vx=0.0, vy=0.0, vth=self.current_angular_velocity * -1)
        """
        pass

    def stop(self):
        """
        TODO: code "stop movement" function here.
        """
        pass

    def estop(self):
        """
        TODO: code "estop" function here.
        """
        pass

    def dock(self):
        """
        TODO: code "dock to station" function here.
        """
        pass

    def undock(self):
        """
        TODO: code "undock station" function here.
        """
        pass

    def set_auto_mode(self):
        """
        >>> Optional.
        TODO: code another logic(resume task, send message to operator...) for here.
        """
        self.operation_mode = "AUTO"

    def set_teleop_mode(self):
        """
        >>> Optional.
        TODO: code another logic(pause task, send message to operator...) for here.
        """
        self.operation_mode = "TELEOP"

    def move_to_waypoint(self, name):
        """
        >>> Optional.
        TODO: If your robot supports moving to a waypoint, implement the waypoint movement function here.
        ex) self.robot.move_to_waypoint(name)
        """
        pass

    def move_to_goal(self, x, y, theta):
        """
        >>> Optional.
        TODO: If your robot supports moving to coordinate, implement the coordinate movement function here.
        ex) self.robot.move_to_goal(x, y, theta)
        """
        pass

    def increase_linear_speed(self):
        # increase current linear velocity when teleoperation control
        if self.current_linear_velocity < MAX_LINEAR_VELOCITY:
            step = round(MAX_LINEAR_VELOCITY / 10, 2)
            self.current_linear_velocity += step

    def decrease_linear_speed(self):
        # decrease current linear velocity when teleoperation control
        if self.current_linear_velocity > MIN_LINEAR_VELOCITY:
            step = round(MAX_LINEAR_VELOCITY / 10, 2)
            self.current_linear_velocity -= step

    def increase_angular_speed(self):
        # increase current angular velocity when teleoperation control
        if self.current_angular_velocity < MAX_ANGULAR_VELOCITY:
            step = round(MAX_ANGULAR_VELOCITY / 10, 2)
            self.current_angular_velocity += step

    def decrease_angular_speed(self):
        # decrease current angular velocity when teleoperation control
        if self.current_angular_velocity > MIN_ANGULAR_VELOCITY:
            step = round(MAX_ANGULAR_VELOCITY / 10, 2)
            self.current_angular_velocity -= step

    def reset_speed_to_default(self):
        # reset current velocity to default velocity
        self.current_linear_velocity = round(MAX_LINEAR_VELOCITY / 2, 2)
        self.current_angular_velocity = round(MAX_ANGULAR_VELOCITY / 2, 2)
# </editor-fold>
#endregion
    # ===============================================================
    # Additional methods
    # You can define any methods and excute by KRPC Via and-gerri
    # =============================================================== 