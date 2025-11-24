import asyncio

REASON_MAP = {
    0: "nomal",
    1: "docking...",
    2: "EMS",
    3: "charging",
    4: "no route",
    5: "docking failed",
    6: "abnomal pose",
    7: "too far",
    8: "no fixed route",
    9: "faild to read point"
}

NAV_MODE_MAP = {
    1:"MAPPING",
    2:"NAV"
}

class RobotStatus:
    def __init__(self, robot_id=None, model=None, category=None):
        self.robot_id = robot_id
        self.robot_type = {
            "category": category,
            "model": model,
            "footprint": None,
            "size": {"width": 360,"length": 540,"height": 840}
        }
        self.robot_state = {"mode":"AUTO","operating_state":["IDLE"]}
        # ZYX
        self.pose = {
            "position": {"x": None,"y": None,"z": None},
            "orientation": {"yaw": None,"pitch": None,"roll": None,},
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
            "name": None,
            "position": None,
            "velocity": None,
            "effort": None
        }
        self.sensor = {
            "ultrasonic": None,
            "laser": None,
            "imu":{"a":None,"g":None}
        }
        self.path_plan = {
            "global" : None,
            "local" : None
        }
        self.map = {
            "name": "KETI-SUSEO2F-3F",
            "origin": (36,377),
            "size": (1477,654),
            "resolution": 0.05000000074505806
        }

        self.nav_state = {"goal":None,"result":None,"info":None, "dist":None, "mileage":None}

        self.current_floor = None
        self.nav_mode = None
        self.nav_start_event = asyncio.Event()
        self.nav_end_event = asyncio.Event()

    def parse_current_path_plan(self,status):
        self.path_plan['global'] = status

    def parse_current_pose(self,status):
        self.pose['2d']['x'] = status.get("x",0)
        self.pose['2d']['y'] = status.get("y",0)
        self.pose['2d']['th'] = status.get("th",0)

        if self.pose['2d']['x'] > 36:
            self.current_floor = 3
        if self.pose['2d']['x'] < 36:
            self.current_floor = 2

    def parse_current_mode(self,status):
        mode = status.get("mode",0)
        self.nav_mode = NAV_MODE_MAP.get(mode,"ERROR")

    def parse_current_IMU(self,status):
        self.sensor['imu']['a'] = status.get('a')
        self.sensor['imu']['g'] = status.get('g')

    def parse_current_laser(self,status):
        self.sensor['laser'] = status.get("coordinates")

    def parse_power_management(self,status):
        charge_flag = status.get("chargeFlag")
        now_charging = True if charge_flag == 2 else False
        self.battery_state["percentage"] = status.get("battery")
        self.battery_state["now_charging"] = now_charging
        if now_charging:
            self.robot_state.append("DOCKING")

        if self.battery_state["percentage"] < 10:
            self.robot_state.append("LOW_BATTERY")
    #     ems = result.get("emergencyButton",None)
    #     emergency_stop = True if ems == 0 else False

    def parse_current_velocity(self,status):
        self.velocity["2d"]["vx"] = status.get("vx",0)
        self.velocity["2d"]["vth"] = status.get("vth",0)

        if self.velocity["2d"]["vx"] < 0 and self.velocity["2d"]["vth"] < 0:
            self.robot_state.append("STOPPED")
            self.robot_state.append("IDLE")

        if self.velocity["2d"]["vth"] > 0.05:
            self.robot_state.append("LEFT_TRUN")

        if self.velocity["2d"]["vth"] < -0.05:
            self.robot_state.append("RIGHT_TRUN")

        if not "TURN" in self.robot_state and self.velocity["2d"]["vx"] > 0.05:
            self.robot_state.append("FORWORD")
        if not "TURN" in self.robot_state and self.velocity["2d"]["vx"] < -0.05:
            self.robot_state.append("REVERSE")
    
    def parse_current_nav_state(self,status):
        result = status.get("res")
        reason_code = status.get("reason")
        reason = REASON_MAP.get(reason_code)
        goal = status.get("goal",0)   
        dist = status.get("dist",0)   
        mileage = status.get("mileage",0)  

        if result == 1:
            flag = "START"
            self.init_nav_state()
            self.nav_start_event.set()
            self.status.path_plan["global"] = None
        elif result == 2:
            flag = "PAUSED"
        elif result == 3:
            flag = "SUCCEED" if reason == 0 else "FALIED"
            self.nav_end_event.set()
            self.status.path_plan["global"] = None
        elif result == 4:
            flag = "CANCELED"
            self.nav_end_event.set()
        else:
            flag = "ERROR"
        self.nav_state = {"goal":goal,"result":flag,"info":reason, "dist":dist, "mileage":mileage}
