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

        # spot
        self.locomotion_estimated_runtime = None
        self.motor_power_state = None

        self.sw_estop_state = None
        self.hw_estop_state = None
        self.payload_estop_state = None

        self.fiducial_info = None

        self.spot_mission_state = {"status":None,
                                   "tick":None}
        
        self.ptz_pose = {"pan":None,
                         "tilt":None,
                         "zoom":None}
        
        self.ptz_focus = {"mode":None,
                          "position":None,
                          "distance":None}
        
        self.mode = "SIT"


    def common_status_parser(self,status):
        try:
            battery = status.battery_states[0]  # Protobuf 객체
            self.battery_state["percentage"] = battery.charge_percentage.value
            self.battery_state["now_charging"] = battery.status
            self.battery_state["voltage"] = battery.voltage.value
        except Exception as e:
            print(f"err in common_status_parser / battery : {e}")

        try:
            power = status.power_state
            self.locomotion_estimated_runtime = power.locomotion_estimated_runtime.seconds
            self.motor_power_state = power.motor_power_state
        except Exception as e:
            print(f"err in common_status_parser / power : {e}")
            try:
                print(f"power_state = {status.power_state}")
            except:
                pass

        try:
            estop_states = status.estop_states
            for estop_state in estop_states:
                if estop_state.name == "hardware_estop":
                    self.hw_estop_state = estop_state.state
                if estop_state.name == "software_estop":
                    self.sw_estop_state = estop_state.state
                if estop_state.name == "payload_estop":
                    self.payload_estop_state = estop_state.state
        except Exception as e:
            print(f"err in common_status_parser / estop_states : {e}")

        try:
            joint_states = status.kinematic_state.joint_states
            lst_name = []
            lst_position = []
            lst_velocity = []
            lst_effort = []

            for joint_state in joint_states:
                lst_name.append(joint_state.name)
                lst_position.append(joint_state.position.value)
                lst_velocity.append(joint_state.velocity.value)
                lst_effort.append(joint_state.load.value)
            
            self.joint_state['name'] = lst_name
            self.joint_state['position'] = lst_position
            self.joint_state['velocity'] = lst_velocity
            self.joint_state['effort'] = lst_effort
        except Exception as e:
            print(f"err in common_status_parser / joint_state : {e}")

        try:
            velocity = status.kinematic_state.velocity_of_body_in_odom
            self.velocity['linear']['x'] = velocity.linear.x
            self.velocity['linear']['y'] = velocity.linear.y
            self.velocity['linear']['z'] = velocity.linear.z

            self.velocity['angular']['x'] = velocity.angular.x
            self.velocity['angular']['y'] = velocity.angular.y
            self.velocity['angular']['z'] = velocity.angular.z
        except Exception as e:
            print(f"err in common_status_parser / velocity : {e}")

        try:
            transforms_snapshot = status.kinematic_state.transforms_snapshot
            odom_to_body = transforms_snapshot.child_to_parent_edge_map["odom"] 
            pose_tform = odom_to_body.parent_tform_child

            self.pose["position"]["x"] = pose_tform.position.x
            self.pose["position"]["y"] = pose_tform.position.y
            self.pose["position"]["z"] = pose_tform.position.z

            self.pose["orientation"]["x"] = pose_tform.rotation.x
            self.pose["orientation"]["y"] = pose_tform.rotation.y
            self.pose["orientation"]["z"] = pose_tform.rotation.z
            self.pose["orientation"]["w"] = pose_tform.rotation.w
        except Exception as e:
            print(f"err in common_status_parser / pose : {e}")
        
    def arm_status_parser(self,status):
        pass

    def ptz_pose_parser(self,status):
        try:
            pan = status.pan.value
            tilt = status.tilt.value
            zoom = status.zoom.value
            self.ptz_pose["pan"] = pan
            self.ptz_pose["tilt"] = tilt
            self.ptz_pose["zoom"] = zoom
        except Exception as e:
            print(f"err in ptz_pose_parser: {e}")
            try:
                print(f"ptz_pose = {status}")
            except:
                pass

    def ptz_focus_parser(self,status):
        try:
            mode = status.mode
            position = status.focus_position.value
            distance = status.approx_distance.value
            self.ptz_focus["mode"] = mode
            self.ptz_focus["position"] = position
            self.ptz_focus["distance"] = distance
        except Exception as e:
            print(f"err in ptz_focus_parser: {e}")
            try:
                print(f"focus = {status}")
            except:
                pass

    def fiducial_info_parser(self,objects):
        try:
            for obj in objects:
                camera = obj.apriltag_properties.frame_name_camera
                fiducial_idx = int(obj.apriltag_properties.tag_id) # upper 499 = docking station
                if camera == "frontleft_fisheye" or camera == "frontright_fisheye" and fiducial_idx < 500:
                    self.fiducial_info = fiducial_idx
                    break
                else:
                    self.fiducial_info = None
                    continue
        except Exception as e:
            # print(f"err in fiducial_info_parser: {e}")
            pass

    def mission_status_parser(self,status):
        try:
            mission_status = status.status
            tick_counter = status.tick_counter
            self.spot_mission_state["status"] = mission_status
            self.spot_mission_state["tick"] = tick_counter
        except Exception as e:
            print(f"err in mission_status_parser: {e}")
            try:
                print(f"mission_status: {status.status}")
            except:
                pass