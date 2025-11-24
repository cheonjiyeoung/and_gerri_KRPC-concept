"""
RobotSubController is wraping your robot control code for follow and-gerri rules
TODO:
    1. You must import your own robot control code. (SDK or Written by robot manual.)
    2. Just coding on owr pre-defined methods (Not required, but recommended.)
    3. coding another methods to control your robot by KRPC Via and-gerri
"""

import threading, asyncio
import time
from pubsub import pub
import json

# You must import your robot controll code in here
from gyd_mobile.robot.gyd_mobile_controller import GydMobileController
from gyd_mobile.robot.robot_status import RobotStatus

MAX_LINEAR_VELOCITY = 1.0
MIN_LINEAR_VELOCITY = 0.1
MAX_ANGULAR_VELOCITY = 1.0
MIN_ANGULAR_VELOCITY = 0.1

class GydMobileSubController:
    def __init__(self,robot_id,robot_model,robot_category):
        self.robot_id = robot_id
        self.robot_model = robot_model
        self.robot_category = robot_category
        self.robot_controller = GydMobileController()

        self.status = RobotStatus(robot_id=robot_id,
                                  model=robot_model,
                                  category=robot_category)
        self._lock = threading.Lock()

        self.current_linear_velocity = round(MAX_LINEAR_VELOCITY / 2, 2)
        self.current_angular_velocity = round(MAX_ANGULAR_VELOCITY / 2, 2)

        self.operation_mode = "AUTO"

        # Timer-based update system
        self.update_interval = 0.1      # 주기
        self.call_timeout = 1.0         # _get_status() 타임아웃
        self._update_running = False
        self._update_timer = None

    # --------------------------------------------------------------------
    # 상태 정보 수집
    # --------------------------------------------------------------------
    def _get_status(self):
        with self._lock:
            current_pose = self.robot_controller.get_pose()
            self.status.parse_current_pose(current_pose)

            current_nav_mode = self.robot_controller.get_mode()
            self.status.parse_current_mode(current_nav_mode)

            current_imu = self.robot_controller.get_IMU()
            self.status.parse_current_IMU(current_imu)

            current_laser = self.robot_controller.get_laser()
            self.status.parse_current_laser(current_laser)

            current_power_status = self.robot_controller.get_power_management()
            self.status.parse_power_management(current_power_status)

            current_velocity = self.robot_controller.get_speed()
            self.status.parse_current_velocity(current_velocity)

            current_nav_status = self.robot_controller.get_nav_status()
            self.status.parse_current_nav_state(current_nav_status)

            if self.status.nav_state["result"] in ["START", "PAUSE"]:
                current_path_plan = self.robot_controller.get_path_plan()
                self.status.parse_current_path_plan(current_path_plan)

    # --------------------------------------------------------------------
    # worker thread for timeout control
    # --------------------------------------------------------------------
    def _get_status_worker(self):
        try:
            self._get_status()
        except Exception as e:
            print(f"_get_status() error: {e}")

    # --------------------------------------------------------------------
    # Timer 기반 주기 실행
    # --------------------------------------------------------------------
    def _run_status_update(self):
        if not self._update_running:
            return

        # worker thread 생성
        worker = threading.Thread(target=self._get_status_worker)
        worker.daemon = True
        worker.start()

        # 타임아웃 wait
        worker.join(timeout=self.call_timeout)

        # 타임아웃 발생 시 worker는 종료 못하지만 결과 무시
        if worker.is_alive():
            print("WARNING: _get_status() timeout. Skip this cycle.")

        # 다음 주기 예약
        self._update_timer = threading.Timer(
            self.update_interval,
            self._run_status_update
        )
        self._update_timer.daemon = True
        self._update_timer.start()

    def start_status_update(self, interval=0.1, call_timeout=1.0):
        self.update_interval = interval
        self.call_timeout = call_timeout
        self._update_running = True

        self._update_timer = threading.Timer(
            self.update_interval,
            self._run_status_update
        )
        self._update_timer.daemon = True
        self._update_timer.start()

    def stop_status_update(self):
        self._update_running = False
        if self._update_timer:
            self._update_timer.cancel()
            self._update_timer = None

    # --------------------------------------------------------------------
    # Robot connection
    # --------------------------------------------------------------------
    def _connect(self):
        self.status = RobotStatus(robot_id=self.robot_id,
                                  model=self.robot_model,
                                  category=self.robot_category)

        # 주기 0.1초, 각 호출 타임아웃 1초
        self.start_status_update(interval=0.1, call_timeout=1.0)

        """
        TODO: robot initialize procedure (release brake, motor on, lease, etc.)
        """

    def _disconnect(self):
        self.stop_status_update()
        """
        TODO: robot disconnect code
        """

    def _send_message(self, message):
        if type(message) == dict:
            message = json.dumps(message)
        pub.sendMessage("send_message",message=message)

    # region PreDefinedRobotControl
    # ---------------------------------------------------------
    # Robot control commands
    # ---------------------------------------------------------
    def move_forward(self):
        print("move_forward")
        self.move(vx=self.current_linear_velocity)

    def move_backward(self):
        self.move(vx=self.current_linear_velocity * -1)

    def turn_left(self):
        self.move(vth=self.current_angular_velocity)

    def turn_right(self):
        self.move(vth=self.current_angular_velocity * -1)

    def stop(self):
        self.move()
        self.robot_controller.nav_cancel()

    def estop(self):
        """
        TODO: code "estop" function here.
        """
        pass

    def dock(self):
        self.robot_controller.move_waypoint("2F_CHARGINGPILE")

    def undock(self):
        """
        TODO: code "undock station" function here.
        """
        pass

    def task_pause(self):
        self.robot_controller.nav_pause()

    def task_resume(self):
        self.robot_controller.nav_resume()

    def set_auto_mode(self):
        print("running set_auto_mode")
        self.status.robot_state["mode"] = "AUTO"
        if self.status.nav_state["result"] == "PAUSED":
            self.robot_controller.nav_resume()

    def set_teleop_mode(self):
        self.status.robot_state["mode"] = "TELEOP"
        if self.status.nav_state["result"] == "START":
            self.robot_controller.nav_pause()

    def move_to_waypoint(self, name):
        self.robot_controller.move_waypoint(name)

    def move_to_goal(self, x, y, theta):
        self.robot_controller.move_coord(x,y,theta)

    def increase_linear_speed(self):
        if self.current_linear_velocity < MAX_LINEAR_VELOCITY:
            step = round(MAX_LINEAR_VELOCITY / 10, 2)
            self.current_linear_velocity += step

    def decrease_linear_speed(self):
        if self.current_linear_velocity > MIN_LINEAR_VELOCITY:
            step = round(MAX_LINEAR_VELOCITY / 10, 2)
            self.current_linear_velocity -= step

    def increase_angular_speed(self):
        if self.current_angular_velocity < MAX_ANGULAR_VELOCITY:
            step = round(MAX_ANGULAR_VELOCITY / 10, 2)
            self.current_angular_velocity += step

    def decrease_angular_speed(self):
        if self.current_angular_velocity > MIN_ANGULAR_VELOCITY:
            step = round(MAX_ANGULAR_VELOCITY / 10, 2)
            self.current_angular_velocity -= step

    def reset_speed_to_default(self):
        self.current_linear_velocity = round(MAX_LINEAR_VELOCITY / 2, 2)
        self.current_angular_velocity = round(MAX_ANGULAR_VELOCITY / 2, 2)
    # endregion

    # ===============================================================
    # Additional methods
    # ===============================================================

    def move(self,vx=0,vth=0):
        if self.status.robot_state["mode"] == "TELEOP":
            self.robot_controller.move(vx,vth)

    def nav_cancel(self):
        self.robot_controller.nav_cancel()

    def set_poi(self,x,y,th,type,name):
        if not None in [x, y, th, name, type]:
            self.robot_controller.set_poi(x,y,th,type,name)

    def reloc(self,name,absolute=False):
        if not absolute:
            self.robot_controller.reloc(name)
        else:
            self.robot_controller.reloc_absolute(name)

    def chargingpile_activate(self):
        self.robot_controller.turn_on_power()

    def chargingpile_inactivate(self):
        self.robot_controller.turn_off_power()

    def move_waypoint_async(self,poi,solar_callback=False,uuid=None):
        loop = asyncio.get_event_loop()
        loop.create_task(self._move_waypoint_async(poi,solar_callback,uuid))

    async def _move_waypoint_async(self,poi,solar_callback=False,uuid=None): 
        self.robot_controller.move_waypoint(poi)
        self.status.nav_start_event.clear()
        await self.nav_start_event.wait()
        print(f"move to {poi} start !")
        self.status.nav_end_event.clear()
        await self.status.nav_end_event.wait()
        print(f"move to {poi} done !")
        if solar_callback:
            message = {"topic":"solar_callback","uuid":uuid}
            print(f"solar_send : {message}")
            pub.sendMessage(topicName="solar_send",message=message)
        return True
