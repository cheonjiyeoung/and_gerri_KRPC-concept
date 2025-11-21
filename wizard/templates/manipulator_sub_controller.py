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

# Default mapping: joint index → joint index
# If your robot uses joint names, replace the values with your robot's joint names.
JOINT_NAME_MAP = '__JOINT_NAME_MAP__'

class __CLASS_NAME__:
    def __init__(self,robot_id,robot_model,robot_category):
        self.robot_id = robot_id
        self.robot_model = robot_model
        self.robot_category = robot_category
        self.robot_controller = DummyController() # MUST REPLACE THIS
        self.status = None
        self._lock = threading.Lock()

        self.operation_mode = "AUTO"


    def _update_loop(self):
        while True:
            with self._lock:
                # if you want use all features we provide, please implement the logic to update your robot’s status information in our standard format.
                time.sleep(0.1)

    def disconnect(self):
        """
        DISCONNECT FROM ROBOT
        """

    def send_message(self, message):
        # message will send by pypubsub to operator
        if type(message) == dict:
            message = json.dumps(message)
        pub.sendMessage("send_message",message=message)

    def connect(self):
        self.status = RobotStatus(robot_id=self.robot_id,
                                  model=self.robot_model,
                                  category=self.robot_category)
        threading.Thread(target=self._update_loop,daemon=True).start()
        """
        TODO: code initialize function(release the brake, power on motor, get lease ... ) here.
        """
# You can fold here :)
# region PreDefinedRobotControl
# <editor-fold desc="Robot Control Methods">
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

    def joint_ctrl(self, joint_array:list):
        """
        TODO: code "joint angle control" function here.
        ex) self.robot.joint_control(joint_array)
        """
        pass

    def single_joint_ctrl(self, idx, angle):
        """
        TODO: code "single joint angle control" function here.
        ex) self.robot.set_position(JOINT_NAME_MAP[idx],angle)
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
# </editor-fold>
#endregion
    # ===============================================================
    # Additional methods
    # You can define any methods and excute by KRPC Via and-gerri
    # =============================================================== 
    def hello_universe(self):
        message = {"topic" : "hello", "value" : "universe"}
        self.send_message(message)
