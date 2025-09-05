import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from robot_status import RobotStatus
from gerri.robot.examples.gyd_mobile.gyd_mobile_controller import GydMobileController
from gerri.robot.examples.gyd_mobile.gyd_base_controller import GydBaseController
import threading
import time
import pprint

class GydSubController:
    def __init__(self):
        self.base_controller:GydBaseController = None
        self._lock = threading.Lock()

    def connect(self):
        self.status = RobotStatus(robot_id=self.base_controller.robot_id,
                                  model=self.base_controller.robot_model,
                                  category=self.base_controller.robot_category)
        self.robot_controller = GydMobileController(robot_status=self.status)
        threading.Thread(target=self._update_loop,daemon=True).start()

    def _update_loop(self):
        while True:
            self._lock.acquire()
            turn_left = False
            trun_right = False
            robot_state = []

            pose_x, pose_y, pose_th = self.robot_controller._get_pose()
            self.status.pose["2d"]["x"] = pose_x
            self.status.pose["2d"]["y"] = pose_y
            self.status.pose["2d"]["th"] = pose_th

            vx, vth = self.robot_controller._get_speed()
            self.status.velocity["2d"]["vx"] = vx
            self.status.velocity["2d"]["vth"] = vth
            if vx < 0 and vth < 0:
                robot_state.append("STOPPED")
            if vth > 0.05:
                turn_left = True
                robot_state.append("LEFT_TRUN")
            if vth < -0.05:
                turn_right = True
                robot_state.append("RIGHT_TRUN")
            if not (trun_right and turn_left) and vx > 0.05:
                robot_state.append("FORWORD")
            if not (trun_right and turn_left) and vx < -0.05:
                robot_state.append("REVERSE")

            laser = self.robot_controller._get_laser()
            self.status.sensor["laser"] = laser

            battery_percentage, now_charging, emergency_stop = self.robot_controller._get_power_management()
            self.status.battery_state["percentage"] = battery_percentage
            self.status.battery_state["now_charging"] = now_charging
            if now_charging:
                robot_state.append("DOCKING")

            if battery_percentage < 10:
                robot_state.append("LOW_BATTERY")

            nav_state, dist, mileage = self.robot_controller._get_nav_status()
            self.status.nav_state = nav_state
            nav_result = nav_state.get("result")

            if nav_result == "SUCCEED":
                robot_state.append("IDLE")
                self.status.path_plan["global"] = None
            if nav_result == "PAUSED":
                robot_state.append("PAUSED")
            if nav_result == "START":
                path_plan = self.robot_controller._get_path_plan()
                if path_plan is not None:
                    self.status.path_plan["global"] = path_plan
            if nav_result == "FAILED" or nav_result == "CANCELED":
                self.status.path_plan["global"] = None

            self.status.robot_state["operating_state"] = robot_state

            self._lock.release()
            # test = vars(self.status)
            # pprint.pprint(self.status.path_plan)
            time.sleep(0.1)

    def disconnect(self):
        """
        DISCONNECT FROM ROBOT
        """

    def mode_auto(self):
        self.status.robot_state["mode"] = "AUTO"
        if self.status.nav_state["result"] == "PAUSED":
            self.robot_controller.nav_resume()

    def mode_teleop(self):
        self.status.robot_state["mode"] = "TELEOP"
        if self.status.nav_state["result"] == "START":
            self.robot_controller.nav_pause()

    def move(self,value=None,option=None):
        if self.status.robot_state["mode"] == "TELEOP":
            vx = value.get("vx",0)
            vth = value.get("vth",0)
            self.robot_controller.move(vx=vx,vth=vth)

    def move_waypoint(self,value=None,option=None):
        if self.status.robot_state["mode"] == "TELEOP":
            self.status.robot_state["mode"] = "AUTO"
        poi = value.get("poi")
        if poi is not None:
            self.robot_controller.move_waypoint(poi=poi)

    def move_coord(self,value=None,option=None):
        if self.status.robot_state["mode"] == "TELEOP":
            self.status.robot_state["mode"] = "AUTO"
        x = value.get("x",0)
        y = value.get("y",0)
        th = value.get("th",0)
        self.robot_controller.move_coord(x,y,th)

    def stop(self,value=None,option=None):
        self.robot_controller.move(0,0)

    def nav_pause(self,value=None,option=None):
        self.robot_controller.nav_pause()

    def nav_cancel(self,value=None,option=None):
        self.robot_controller.nav_cancel()

    def nav_resume(self,value=None,option=None):
        self.robot_controller.nav_resume()

    def set_poi(self,value=None,option=None):
        x = value.get("x")
        y = value.get("y")
        th = value.get("th")
        type = value.get("type")
        name = value.get("name")
        
        if not None in [x, y, th, name, type]:
            self.robot_controller.set_poi(x,y,th,type,name)
    
    def reloc(self,value=None,option=None):
        absolute = option.get("absolute",False)
        name = value.get("name")
        if name is not None:
            if not absolute:
                self.robot_controller.reloc(name)
            else:
                self.robot_controller.reloc_absolute(name)
    ### 수정 시작
    def turn_on_power(self, value = None, option = None):
        self.robot_controller.turn_on_power()
    def turn_off_power(self, value = None, option = None):
        self.robot_controller.turn_off_power()
    ### 수정 완료
    def send_message(self, message):
        self.base_controller.send_message(message)

    def handle_joy_command(self,value=None,option=None):
        print("\n /joy input")
        axis = value.get('axes')
        buttons = value.get('buttons')
        print(f"axis = {axis} / buttons = {buttons}")
        vx = 0
        vth = 0

        LT = True if (axis[4] or buttons[6]) else False
        RT = True if (axis[5] or buttons[7]) else False

        A = True if buttons[0] else False
        B = True if buttons[1] else False
        X = True if buttons[2] else False
        Y = True if buttons[3] else False
        LB = True if buttons[4] else False
        RB = True if buttons[5] else False
        BACK = True if buttons[8] else False
        START = True if buttons[9] else False
        C1 = True if buttons[10] else False
        C2 = True if buttons[11] else False
        FOWARD = True if buttons[12] else False
        BACKWARD = True if buttons[13] else False
        LEFT = True if buttons[14] else False
        RIGHT = True if buttons[15] else False

        AXIS_1_LEFT = True if axis[0] == -1 else False
        AXIS_1_RIGHT = True if axis[0] == 1 else False
        AXIS_1_UP = True if axis[1] == -1 else False
        AXIS_1_DOWN = True if axis[1] == 1 else False
        AXIS_2_LEFT = True if axis[2] == -1 else False
        AXIS_2_RIGHT = True if axis[2] == 1 else False
        AXIS_2_UP = True if axis[3] == -1 else False
        AXIS_2_DOWN = True if axis[3] == 1 else False

        actions = [
            ("LT", LT),
            ("RT", RT),
            ("A", A),
            ("B", B),
            ("X", X),
            ("Y", Y),
            ("LB", LB),
            ("RB", RB),
            ("BACK", BACK),
            ("START", START),
            ("C1", C1),
            ("C2", C2),
            ("FOWARD", FOWARD),
            ("BACKWARD", BACKWARD),
            ("LEFT", LEFT),
            ("RIGHT", RIGHT),
            ("AXIS_1_LEFT", AXIS_1_LEFT),
            ("AXIS_1_RIGHT", AXIS_1_RIGHT),
            ("AXIS_1_UP", AXIS_1_UP),
            ("AXIS_1_DOWN", AXIS_1_DOWN),
            ("AXIS_2_LEFT", AXIS_2_LEFT),
            ("AXIS_2_RIGHT", AXIS_2_RIGHT),
            ("AXIS_2_UP", AXIS_2_UP),
            ("AXIS_2_DOWN", AXIS_2_DOWN)
        ]

        # 각 버튼이나 축에 대해, True인 항목을 찾아 출력
        for action, is_pressed in actions:
            if is_pressed:
                print(f"{action} is pressed!")

        if START:
            self.robot_controller.dock_activate()

        if A:
            self.robot_controller.patroll_async(["2F_COFFEEPICK","2F_CHARGINGPILE"])

        if LB:
            self.mode_teleop()
        if RB:
            self.mode_auto()

        if self.status.robot_state["mode"] == "TELEOP":
            if AXIS_1_LEFT:
                vth = 0.3
            if AXIS_1_RIGHT:
                vth = -0.3
            if AXIS_1_UP:
                vx = 0.5
            if AXIS_1_DOWN:
                vx = -0.5
            self.robot_controller.move(vx,vth)
            




