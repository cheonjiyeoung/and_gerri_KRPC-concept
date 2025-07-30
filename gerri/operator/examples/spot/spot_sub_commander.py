import time
from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.examples.spot.spot_base_commander import SpotBaseCommander

import threading

class SpotSubCommander:
    def __init__(self, spot_info, channels, **kwargs):
        self.control_target = 'all'
        self.use_master_arm = False
        self.master_control = False
        self.base_commander:SpotBaseCommander = None
        self.ijkl_mode = "STANCE"

        self.linear_speed = spot_info["default_linear_speed"]
        self.linear_low_speed = spot_info["default_linear_low_speed"]

        self.angular_speed = spot_info["default_angular_speed"]
        self.angular_low_speed = spot_info["default_angular_low_speed"] 

        self.pt_relative_step = spot_info["default_pt_relative_step"]
        self.zoom_relative_step = spot_info["default_zoom_relative_step"]

        self.current_compositor = None
        self.absolute_pan_pose = None
        self.absolute_tilt_pose = None
        self.absolute_zoom_pose = None

        self.current_mission = None

        self.channels = channels
        self._lock = threading.Lock()
        pub.subscribe(self.key_mouse_control, 'key_mouse_control')
        pub.subscribe(self.ui_signal,"ui_signal")

    """
    Initializes the connection for the sub-commander (e.g., hardware setup, activation).
    """
    def connect(self):
        print('Connecting to Gerri...')


    """
    Handles cleanup and shutdown for the sub-commander (if applicable).
    """
    def disconnect(self):
        pass
        ### TODO : ADD DISCONNECT FUNCTION

    """
    Sets the reference to the base commander instance.
    This allows the sub commander to call base-level methods.
    """
    def init_base_commander(self, base_commander):
        self.base_commander = base_commander


    """
    Clamp a given value to a specified min-max range.
    Optionally constrain further with an absolute limit.
    """
    def clamp(self, value, min_value, max_value, absolute_limit=None):
        """
        값을 주어진 범위로 제한 (클램핑).

        :param value: 제한할 값
        :param min_value: 최소값
        :param max_value: 최대값
        :param absolute_limit: 절대 제한 (튜플, optional)
        :return: 제한된 값
        """
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))

    """
    Maps a value from one range into another using linear scaling.
    """
    def map_value(self, value, in_min, in_max, out_min, out_max):
        """
        특정 값을 주어진 범위 내에서 다른 범위로 매핑하는 함수.

        :param value: 매핑할 값
        :param in_min: 매핑 전 최소값
        :param in_max: 매핑 전 최대값
        :param out_min: 매핑 후 최소값
        :param out_max: 매핑 후 최대값
        :return: 매핑된 값
        """
        map_value = self.clamp((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max)

        return map_value

    def ui_signal(self, signal, value=None):
        if signal == "cb_mission_list":
            self.current_mission = value
        if signal == "Mission Start":
            self.base_commander.run_mission(value = self.current_mission)
        if signal == "Mission Pause":
            self.base_commander.pause_mission()
        if signal == "Mission Stop":
            self.base_commander.stop_mission()
        if signal == "Mission Restart":
            self.base_commander.restart_mission()

        if signal == "Stand":
            self.base_commander.stand()
        if signal == "Sit":
            self.base_commander.sit()
        if signal == "Walk":
            self.base_commander.walk()
        if signal == "Stop":
            self.base_commander.action_stop()
        if signal == "Docking":
            self.base_commander.dock()
        if signal == "UnDocking":
            self.base_commander.undock()
        if signal == "Stance":
            self.ijkl_mode = "STANCE"
        if signal == "PTZ":
            self.ijkl_mode = "PTZ"
        if signal == "spin_default_linear_speed":
            self._lock.acquire()
            self.linear_speed = float(value)
            self._lock.release()
        if signal == "spin_low_linear_speed":
            self._lock.acquire()
            self.linear_low_speed = float(value)
            self._lock.release()
        if signal == "spin_default_angular_speed":
            self._lock.acquire()
            self.angular_speed = float(value)
            self._lock.release()
        if signal == "spin_low_angular_speed":
            self._lock.acquire()
            self.angular_low_speed = float(value)
            self._lock.release()

        if signal == "cb_compositor":
            self.current_compositor = value
        if signal == "set_compositor":
            self.base_commander.set_main_cam(self.current_compositor)
        if signal == "Tilt UP":
            ptz_step = {"pan":0,"tilt":self.pt_relative_step,"zoom":0}
            self.base_commander.ptz_relative(ptz_step)
        if signal == "Tilt UP":
            ptz_step = {"pan":0,"tilt":-1*self.pt_relative_step,"zoom":0}
            self.base_commander.ptz_relative(ptz_step)
        if signal == "Pan LEFT":
            ptz_step = {"pan":-1*self.pt_relative_step,"tilt":0,"zoom":0}
            self.base_commander.ptz_relative(ptz_step)
        if signal == "Pan RIGHT":
            ptz_step = {"pan":self.pt_relative_step,"tilt":0,"zoom":0}
            self.base_commander.ptz_relative(ptz_step)
        if signal == "Zoom":
            ptz_step = {"pan":0,"tilt":0,"zoom":self.zoom_relative_step}
            self.base_commander.ptz_relative(ptz_step)
        if signal == "ptz_absolute":
            ptz = {
                "pan": self.absolute_pan_pose,
                "tilt": self.absolute_tilt_pose,
                "zoom": self.absolute_zoom_pose
            }
            self.base_commander.ptz_absolute(ptz)
        if signal == "edit_pan_pose":
            self.absolute_pan_pose = value
        if signal == "edit_tilt_pose":
            self.absolute_tilt_pose = value
        if signal == "edit_zoom_pose":
            self.absolute_zoom_pose = value
        if signal == "spin_pt_step":
            self.pt_relative_step = value
        if signal == "spin_zoom_step":
            self.zoom_relative_step = value
        if "_connect" in signal:
            channel_name = signal.removesuffix("_connect")
            print(f"channel_name:{channel_name}")
            self.channels[channel_name].connect()
        if "_disconnect" in signal:
            channel_name = signal.removesuffix("_disconnect")
            print(f"channel_name:{channel_name}")
            self.channels[channel_name].disconnect()

 
        

        


            
    """
    Handles key and mouse input events and maps them to robot commands.
    """
    def key_mouse_control(self, command):
        key = command['key_control']
        mouse_d_move = command['mouse_d_move']
        mouse_d_wheel = command['mouse_d_wheel']
        mouse_click = command['mouse_click']
        vx = 0
        vy = 0
        vth = 0

        pitch = 0
        yaw = 0
        roll = 0

        pan = 0
        tilt = 0
        zoom = 0
        reset = False
        if self.base_commander:

            if "W" in key:
                if "SHIFT" in key:
                    vx = self.linear_low_speed
                else:
                    vx = self.linear_speed
            elif "S" in key:
                if "SHIFT" in key:
                    vx = -1 * self.linear_low_speed
                else:
                    vx = -1 * self.linear_speed
            if "A" in key:
                if "SHIFT" in key:
                    vy = self.linear_low_speed
                else:
                    vy = self.linear_speed
            elif "D" in key:
                if "SHIFT" in key:
                    vy = -1 * self.linear_low_speed
                else:
                    vy = -1 * self.linear_speed
            if "Q" in key:
                if "SHIFT" in key:
                    vth = self.angular_low_speed
                else:
                    vth = self.angular_speed
            elif "E" in key:
                if "SHIFT" in key:
                    vth = -1 * self.angular_low_speed
                else:
                    vth = -1 * self.angular_speed
            if vx != 0 or vy != 0 or vth != 0:
                move_params = {"vx":vx,"vy":vy,"vth":vth}
                self.base_commander.move(move_params)

            if self.ijkl_mode == "STANCE":
                if "I" in key:
                    pitch = 0.1
                if "K" in key:
                    pitch = -0.1
                if "J" in key:
                    roll = 0.1
                if "L" in key:
                    roll = -0.1
                if "U" in key:
                    yaw = 0.1
                if "O" in key:
                    yaw = -0.1
                if "P" in key:
                    reset = True

                if pitch != 0 or roll != 0 or yaw != 0:
                    stance_params = {"pitch":pitch,"roll":roll,"yaw":yaw,"reset":reset}
                    self.base_commander.set_stance(stance_params)

            if self.ijkl_mode == "PTZ":
                if "I" in key:
                    tilt = self.pt_relative_step
                if "K" in key:
                    tilt = -1 * self.pt_relative_step
                if "J" in key:
                    pan = -1 * self.pt_relative_step
                if "L" in key:
                    pan = self.pt_relative_step
                if "U" in key:
                    zoom = self.zoom_relative_step
                if "O" in key:
                    zoom = -1 * self.zoom_relative_step

                if pan != 0 or tilt != 0 or zoom != 0:
                    ptz_step = {"pan":pan,"tilt":tilt,"zoom":zoom}
                    self.base_commander.ptz_relative(ptz_step)

            if "F1" in key:
                self.base_commander.stand()
            if "F2" in key:
                self.base_commander.sit()
            if "F3" in key:
                self.base_commander.walk()
            if "F4" in key:
                self.base_commander.stop()                
            if "F5" in key and self.spot_info["using_ptz"]:
                self.ijkl_mode = "STANCE"
                print(f"\n\n{self.ijkl_mode}\n\n")
            if "F6" in key and self.spot_info["using_ptz"]:
                self.ijkl_mode = "PTZ"
                print(f"\n\n{self.ijkl_mode}\n\n")
        else:
            print("basecontroller none")
            