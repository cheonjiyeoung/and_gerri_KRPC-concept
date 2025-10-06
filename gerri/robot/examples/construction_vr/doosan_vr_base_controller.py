import asyncio
from pubsub import pub
import os, sys
import copy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.status_manager import StatusManager
from gerri.robot.function.manipulator_function import ManipulatorFunction
from gerri.robot.function.pan_tilt_zoom_function import PTZFunction

from gerri.robot.function.tf_helper import *
from gerri.robot.function.ik_solver import IKSolver


class DoosanVRBaseController:
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.sub_controller = None
        self.start_pose = None
        self.target_pose = None


        self.zoom_level = 1.0

        if 'interface' in params:
            self.interface = params['interface']
            self.last_interface_value = copy.deepcopy(self.interface)
        else:
            self.interface = None
            self.last_interface_value = None

        pub.subscribe(self.receive_message,"receive_message")
        pub.subscribe(self.set_zoom_level, "zoom_level")

    def connect(self):
        self.sub_controller.connect()
        self.status_manager = StatusManager(self.robot_info, self.sub_controller)

    def disconnect(self):
        self.sub_controller.disconnect()

    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def set_zoom_level(self, level):
        self.zoom_level = level
        print("zoom_level:", self.zoom_level)


    def receive_message(self, message):
        ManipulatorFunction.receive_message(self, message=message)
        PTZFunction.receive_message(self, message)
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == self.interface.name:
                            self.interface.update(value)
                            # --- 팔 제어 로직 (오른쪽 그립) ---
                            if self.interface.button_left_thumbstick:
                                pub.sendMessage('zoom_step_control', step=self.interface.left_axis_Y/10)

                            if self.interface.button_A:
                                self.sub_controller.joint_ctrl(self.sub_controller.joint_preset['home'])

                            if self.interface.button_right_grip != self.last_interface_value.button_right_grip:
                                self.interface.reset_initial_pose('right')
                                if self.interface.button_right_grip:
                                    self.start_pose = self.sub_controller.get_current_SE3_pose()

                                else:
                                    self.start_pose = None
                                    self.sub_controller.joint_ctrl_vel_stop()

                            if self.interface.button_right_grip and self.start_pose:
                                # 속도 제어
                                delta_vr = vr_ee_converter(self.interface.right_delta_pose)

                                delta_position = np.round(delta_vr.translation, 2)

                                delta_vr_rotation = pin.rpy.matrixToRpy(delta_vr.rotation)
                                delta_vr_deg = np.rad2deg(delta_vr_rotation)
                                rpy_rounded = np.round(delta_vr_deg, 1)

                                print(delta_position, rpy_rounded)

                                target_pose_base = self.start_pose * delta_vr

                                # 6. 최종 변환된 목표 자세를 CLIK 속도 제어 함수로 전달합니다.
                                self.sub_controller.joint_ctrl_vel(target_pose_base)


                            self.last_interface_value = copy.deepcopy(self.interface)

                        elif topic == 'connect_robot':
                            self.connect()
                        elif topic == 'get_robot_status':
                            pub.sendMessage('send_message', message=self.sub_controller.update_status())
                        elif topic == 'custom_command':
                            self.sub_controller.custom_command(value)

                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                        pass
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")
                        pass