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
                                # 1. VR 컨트롤러의 움직임을 월드 좌표계 기준의 '움직임 변화량'으로 변환합니다.
                                delta_ee_world = vr_ee_converter(self.interface.right_delta_pose)

                                delta_ee_cali = self.sub_controller.T_correction * delta_ee_world


                                # 1. 시작점 -> 월드 변환
                                start_pose_world = self.sub_controller.T_world_base * self.start_pose
                                # 2. 월드에서 목표점 계산
                                target_pose_world = start_pose_world * delta_ee_cali
                                # 3. 최종 목표점 -> 베이스 변환
                                target_pose_base = self.sub_controller.T_world_base.inverse() * target_pose_world

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