from pubsub import pub

class SpotStatus:
    def __init__(self,spot_info):
        # pose 그대로 유지
        self.pose = {
            'position': {'x': None, 'y': None, 'z': None},
            'orientation': {'x': None, 'y': None, 'z': None, 'w': None}
        }

        # battery_state의 개별 속성
        self.voltage = None
        self.current = None
        self.capacity = None
        self.percentage = None                      # ok
        self.temperature = None
        self.power_supply_status = None
        self.now_charging = None                    # ok

        # joint_state 분리
        self.joint_names = None
        self.joint_positions = None
        self.joint_velocities = None
        self.joint_efforts = None

        # sensor
        self.ultrasonic = None
        self.laser = None

        # map
        self.map_name = None
        self.map_origin = None
        self.map_size = None
        self.map_scale = None

        # additional_info → 최상위 속성으로 분리
        self.locomotion_estimated_runtime = None
        self.motor_power_state = None               # ok
        self.sw_estop_state = None          # ok
        self.hw_estop_state = None          # ok
        self.payload_estop_state = None     # ok
        self.fiducial_info = None

        self.spot_mission_status = None     # ok
        self.spot_mission_tick = None       # ok

        self.ptz_pan = None                 # ok
        self.ptz_tilt = None                # ok 
        self.ptz_zoom = None                # ok

        self.ptz_focus_mode = None          # ok
        self.ptz_focus_position = None      # ok
        self.ptz_focus_distance = None      # ok

        self.spot_mode = None

        self.spot_info = spot_info

        pub.subscribe(self.update, "status_update")


    def update(self, data):
        try:
            data = data["robot_info"]
            # pose
            self.pose['position'] = data.get('pose', {}).get('position', self.pose['position'])
            self.pose['orientation'] = data.get('pose', {}).get('orientation', self.pose['orientation'])

            # battery
            battery = data.get('battery_state', {})
            self.voltage = battery.get('voltage')
            self.current = battery.get('current')
            self.capacity = battery.get('capacity')
            self.percentage = battery.get('percentage')
            self.temperature = battery.get('temperature')
            self.power_supply_status = battery.get('power_supply_status')
            self.now_charging = self.charging_state_mapper(battery.get('now_charging'))

            # joint_state
            joint = data.get('joint_state', {})
            self.joint_names = joint.get('name')
            self.joint_positions = joint.get('position')
            self.joint_velocities = joint.get('velocity')
            self.joint_efforts = joint.get('effort')

            # sensors
            sensors = data.get('sensor', {})
            self.ultrasonic = sensors.get('ultrasonic')
            self.laser = sensors.get('laser')

            # map
            map_data = data.get('map', {})
            self.map_name = map_data.get('name')
            self.map_origin = map_data.get('origin')
            self.map_size = map_data.get('size')
            self.map_scale = map_data.get('scale')

            # additional_info (flattened)
            self.spot_mode = data.get('mode')
            self.locomotion_estimated_runtime = data.get('locomotion_estimated_runtime')
            self.motor_power_state = self.motor_power_state_mapper(data.get('motor_power_state'))
            self.sw_estop_state = self.estop_state_mapper(data.get('sw_estop_state'))
            self.hw_estop_state = self.estop_state_mapper(data.get('hw_estop_state'))
            self.payload_estop_state = self.estop_state_mapper(data.get('payload_estop_state'))
            self.fiducial_info = data.get('fiducial_info')

            mission = data.get('spot_mission_state', {})
            self.spot_mission_status = self.mission_status_mapper(mission.get('status'))
            self.spot_mission_tick = mission.get('tick')

            if self.spot_info["using_ptz"]:
                ptz_pose = data.get('ptz_pose', {})
                self.ptz_pan = round(ptz_pose.get('pan'),2)
                self.ptz_tilt = round(ptz_pose.get('tilt'),2)
                self.ptz_zoom = round(ptz_pose.get('zoom'),2)

                ptz_focus = data.get('ptz_focus', {})
                self.ptz_focus_mode = ptz_focus.get('mode')
                self.ptz_focus_position = ptz_focus.get('position')
                self.ptz_focus_distance = ptz_focus.get('distance')
        except Exception as e:
            pass

        
    def safe_enum_mapper(self, idx, states):
        return states[idx] if 0 <= idx < len(states) else "INVALID"

    def motor_power_state_mapper(self,idx):
        lst_state = ["UNKNOWN","OFF","ON","ING_ON","ING_OFF","ERROR"]
        return self.safe_enum_mapper(idx, lst_state)
    
    def estop_state_mapper(self,idx):
        lst_state = ["UNKNOWN","TRUE","FALSE"]
        return self.safe_enum_mapper(idx, lst_state)
    
    def charging_state_mapper(self,idx):
        lst_state = ["UNKNOWN","MISSING","CHARGING","DISCHARGING","BOOTING"]
        return self.safe_enum_mapper(idx, lst_state)

    def behaver_state_mapper(self,idx):
        lst_state = ["UNKNOWN","NOT_READY","TRANSITION","STANDING","STEPPING"]
        return self.safe_enum_mapper(idx, lst_state)

    def behaverfault_cause_state_mapper(self,idx):
        lst_state = ["UNKNOWN","FALL","HARDWARE","LEASE_TIMEOUT"]
        return self.safe_enum_mapper(idx, lst_state)
    
    def behaverfault_status_state_mapper(self,idx):
        lst_state = ["UNKNOWN","CLEARABLE","UNCLEARABLE"]
        return self.safe_enum_mapper(idx, lst_state)
    
    def manipulator_stow_status_mapper(self,idx):
        lst_state = ["UNKNOWN","STOWED","DEPLOYED"]
        return self.safe_enum_mapper(idx, lst_state)       
    
    def manipulator_carry_status_mapper(self,idx):
        lst_state = ["UNKNOWN","NOT_CARRIABLE","CARRIABLE","CARRIABLE_AND_STOWABLE"]
        return self.safe_enum_mapper(idx, lst_state)
    
    def mission_status_mapper(self,idx):
        lst_state = ["UNKNOWN","FAILURE","RUNNING","SUCCESS",
                     "PAUSED","ERROR","NONE","STOPPED"]
        return self.safe_enum_mapper(idx, lst_state)
    
    def answerquestion_state_mapper(self,idx):
        lst_state = ["UNKNOWN","OK","INVALID_QUESTION_ID","INVALID_CODE",
                     "ALREADY_ANSWERED","CUSTOM_PARAMS_ERROR","INCOMPATIBLE_ANSWER"]
        return self.safe_enum_mapper(idx, lst_state)
    

    
