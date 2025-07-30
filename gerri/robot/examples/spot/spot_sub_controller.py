import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from robot_status import RobotStatus
import threading
import random
import time

class SpotSubController:
    def __init__(self,spot_info):
        self.spot_info = spot_info
        if spot_info["using_ptz"]:   
            from gerri.robot.examples.spot.spot_ptz import SpotPTZ
            self.robot_controller = SpotPTZ(spot_info)
        elif spot_info["using_arm"]:
            from gerri.robot.examples.spot.spot_arm import SpotArm
            self.robot_controller = SpotArm(spot_info)
        else:
            from gerri.robot.examples.spot.spot_controller import SpotController
            self.robot_controller = SpotController(spot_info)

        self.base_controller = None
        self.status = None
        self._lock = threading.Lock()

    def connect(self):
        self.status = RobotStatus(robot_id=self.base_controller.robot_id,
                                  model=self.base_controller.robot_model,
                                  category=self.base_controller.robot_category)
        self.th_update_status = threading.Thread(target=self._update_loop,daemon=True)
        self.th_update_status.start()
        self.robot_controller.connect()
        time.sleep(1)
        print("[DEBUG] update loop thread alive?", self.th_update_status.is_alive())
        
        

    def disconnect(self):
        self.th_update_status.join()
        self.robot_controller.lease_keepalive.shutdown()
        

    def _update_loop(self):
        while True:
            try:
                print("run???????????????")
                self._lock.acquire()
                self.robot_controller._robot_state_task.update()
                
                fiducial_objects = self.robot_controller.spot_camera.get_fiducial_objects()
                mission_state = self.robot_controller.spot_autowalk.get_mission_state()

                self.status.common_status_parser(self.robot_controller.robot_state)
                self.status.fiducial_info_parser(fiducial_objects)
                self.status.mission_status_parser(mission_state)

                if self.spot_info["using_ptz"]:
                    self.get_ptz_status()
                self._lock.release()
                time.sleep(0.2)
            except Exception as e:
                pass

    def get_ptz_status(self):
        ptz_focus = self.robot_controller.spot_ptz_get_focus()
        ptz_pose = self.robot_controller.spot_ptz_get_position()
        self.status.ptz_pose_parser(ptz_pose)
        self.status.ptz_focus_parser(ptz_focus)


    def walk(self,value=None,option=None):
        if self.status.mode == 'Stand':
            self.robot_controller.spot_act.walk()
            self.status.mode = 'Walk'

    def stand(self,value=None,option=None):
        self.robot_controller.spot_act.stand()
        self.status.mode = 'Stand'

    def sit(self,value=None,option=None):
        self.robot_controller.spot_act.sit()
        self.status.mode = 'Sit'

    def action_stop(self,value=None,option=None):
        self.robot_controller.spot_act.action_stop()

    def move(self,value=None,option=None):
        print(1)
        if self.status.mode=='Walk':
            print(2)
            vx = value.get('vx',0)
            vy = value.get('vy',0)
            vth = value.get('vth',0)
            print(3)
            self.robot_controller.spot_act.move(vx=vx,vy=vy,vth=vth)
            print(4)

    def run_mission(self,value=None,option=None):
        if self.status.mode =='Walk':
            mission_name = value
            self.robot_controller.spot_autowalk.handle_mission(mission_name)

    def stop_mission(self,value=None,option=None):
        self.robot_controller.spot_autowalk.stop_mission()

    def pause_mission(self,value=None,option=None):
        self.robot_controller.spot_autowalk.pause_mission()

    def resume_mission(self,value=None,option=None):
        self.robot_controller.spot_autowalk.restart_mission()

    def dock(self,value=None,option=None):
        undock = value
        if undock:
            self.status.mode = "Stand"
        self.robot_controller.spot_act.dock(undock)

    # SPOT PTZ
    def spot_ptz_absolute(self,value=None,option=None):
        pan = value.get("pan", 0)
        tilt = value.get("tilt", 0)
        zoom = value.get("zoom", 1) 
        ptz_position = self.robot_controller.spot_ptz_absolute(
                                                 pan,
                                                 tilt,
                                                 zoom)
        return ptz_position
    
    # def spot_ptz_absolute(self,value=None,option=None):
    #     pan = value.get("pan", 0)
    #     tilt = value.get("tilt", 0)
    #     zoom = value.get("zoom", 1) 
    #     ptz_position = self.robot_controller.spot_ptz_absolute(
    #                                              pan,
    #                                              tilt,
    #                                              zoom)
    #     return ptz_position

    def spot_ptz_initialize(self,value=None,option=None):
        self.robot_controller.spot_ptz_initialize()

    def spot_ptz_set_focus(self,value=None,option=None):
        mode = value.get('mode')
        if mode == 'manual':
            dist = value.get('dist')
        else:
            dist = None
        self.robot_controller.spot_ptz_set_focus(mode,dist)

    def spot_ptz_set_focus_relative(self,value=None,option=None):
        step = value
        current_dist = self.status.ptz_focus.get('dist')
        if current_dist is not None:
            target_dist = current_dist+step
            self.robot_controller.spot_ptz_set_focus_relative(target_dist)

    def set_compositor(self,value=None,option=None):
        name = value
        self.robot_controller.spot_set_compositor(name)

    # SPOT_ARM



