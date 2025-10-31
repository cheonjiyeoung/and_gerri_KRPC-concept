import asyncio
from pubsub import pub
import os, sys
import copy

from gerri.robot.examples.construction_vr.doosan_vr_sub_controller import RobotMode
from gerri.robot.interface.vr_controller import VRController

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.status_manager import StatusManager
from gerri.robot.function.manipulator_function import ManipulatorFunction

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
        self.move_scale = 1
        self.force_scale = 100

        self.action_mode = 'MASTER_CONTROL'


        if 'interface' in params:
            self.interface:VRController = params['interface']
            self.last_interface_value = copy.deepcopy(self.interface)
        else:
            self.interface = None
            self.last_interface_value = None

        pub.subscribe(self.receive_message,"receive_message")
        pub.subscribe(self.set_zoom_level, "zoom_level")

    def connect(self):
        if self.sub_controller:
            self.sub_controller.connect()
        else:
            from gerri.robot.examples.construction_vr.construction_vr_config import ROBOT_INFO
            from gerri.robot.examples.construction_vr.doosan_vr_sub_controller import DoosanVRSubController, RobotMode
            sub_controller = DoosanVRSubController(ROBOT_INFO['ip'], ROBOT_INFO['port'], debug=True)
            self.sub_controller = sub_controller
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
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == self.interface.name:
                            self.interface.update(value)


                            # ##############################################################################
                            # #                          KINETIC ZOOM SECTION                              #
                            # ##############################################################################
                            if self.interface.button_left_thumbstick:
                                pub.sendMessage('zoom_step_control', step=self.interface.left_axis_Y / 15)

                            if self.interface.button_Y:
                                if self.sub_controller.control_mode == RobotMode.FORCE:
                                    self.sub_controller.change_mode(24)
                                else:
                                    self.sub_controller.change_mode(0)
                                self.sub_controller.control_mode = RobotMode.IDLE


                            # ##############################################################################
                            # #                          IDLE_CONTROL SECTION                              #
                            # ##############################################################################
                            if self.sub_controller.control_mode == RobotMode.IDLE:
                                if self.interface.button_left_grip and self.interface.right_grip and self.interface.button_A and self.interface.button_B:
                                    self.sub_controller.control_mode = RobotMode.MASTER
                                elif self.interface.button_A:
                                    self.sub_controller.joint_ctrl(self.sub_controller.joint_preset['home'])
                                elif self.interface.button_B:
                                    self.sub_controller.control_mode = RobotMode.FORCE
                                    self.sub_controller.change_mode(21)
                                elif self.interface.button_X:
                                    self.sub_controller.control_mode = RobotMode.MOVE
                                    self.sub_controller.change_mode(7)


                            # ##############################################################################
                            # #                          MASTER_CONTROL SECTION                            #
                            # ##############################################################################

                            if self.sub_controller.control_mode == RobotMode.MASTER:
                                if self.interface.button_A:
                                    self.sub_controller.joint_ctrl(self.sub_controller.joint_preset['home'])

                                if 'left' in self.robot_id:
                                    if self.interface.button_left_grip != self.last_interface_value.button_left_grip:
                                        self.interface.reset_initial_pose('left')
                                        if self.interface.button_left_grip:
                                            self.start_pose = self.sub_controller.get_current_SE3_pose()

                                        else:
                                            self.start_pose = None
                                            self.sub_controller.joint_ctrl_vel_stop()

                                    if self.interface.button_left_grip and self.start_pose:
                                        # 1. VR 컨트롤러의 움직임을 월드 좌표계 기준의 '움직임 변화량'으로 변환합니다.
                                        delta_ee_world = vr_ee_converter(self.interface.left_delta_pose, scale_factor=1/self.zoom_level)
                                        print(delta_ee_world)

                                        delta_ee_cali = self.sub_controller.T_correction * delta_ee_world
                                        # 1. 시작점 -> 월드 변환
                                        start_pose_world = self.sub_controller.T_world_base * self.start_pose
                                        # 2. 월드에서 목표점 계산
                                        target_pose_world = start_pose_world * delta_ee_cali
                                        # 3. 최종 목표점 -> 베이스 변환
                                        target_pose_base = self.sub_controller.T_world_base.inverse() * target_pose_world

                                        self.sub_controller.joint_ctrl_clik(target_pose_base, tolerance = 10)

                                if 'right' in self.robot_id:

                                    if self.interface.button_right_grip != self.last_interface_value.button_right_grip:
                                        self.interface.reset_initial_pose('right')
                                        if self.interface.button_right_grip:
                                            self.start_pose = self.sub_controller.get_current_SE3_pose()

                                        else:
                                            self.start_pose = None
                                            self.sub_controller.joint_ctrl_vel_stop()

                                    if self.interface.button_right_grip and self.start_pose:
                                        # 1. VR 컨트롤러의 움직임을 월드 좌표계 기준의 '움직임 변화량'으로 변환합니다.
                                        delta_ee_world = vr_ee_converter(self.interface.right_delta_pose, scale_factor=1/self.zoom_level)
                                        print(delta_ee_world)

                                        delta_ee_cali = self.sub_controller.T_correction * delta_ee_world
                                        # 1. 시작점 -> 월드 변환
                                        start_pose_world = self.sub_controller.T_world_base * self.start_pose
                                        # 2. 월드에서 목표점 계산
                                        target_pose_world = start_pose_world * delta_ee_cali
                                        # 3. 최종 목표점 -> 베이스 변환
                                        target_pose_base = self.sub_controller.T_world_base.inverse() * target_pose_world

                                        self.sub_controller.joint_ctrl_clik(target_pose_base, tolerance = 10)

                            # ##############################################################################
                            # #                       MOVE AND FORCE_CONTROL SECTION                       #
                            # ##############################################################################

                            if self.sub_controller.control_mode == RobotMode.MOVE or self.sub_controller.control_mode == RobotMode.FORCE:
                                if self.sub_controller.control_mode == RobotMode.MOVE:
                                    scaler = self.move_scale
                                elif self.sub_controller.control_mode == RobotMode.FORCE:
                                    scaler = self.force_scale
                                else:
                                    scaler = 0

                                if 'left' in self.robot_id:
                                    x = self.interface.left_axis_X * scaler
                                    y = self.interface.left_axis_Y * scaler
                                    if self.interface.button_left_grip:
                                        z = self.interface.left_trigger * scaler
                                    else:
                                        z = -self.interface.left_trigger * scaler
                                    self.sub_controller.xyz_move([x, y, z])

                                if 'right' in self.robot_id:
                                    x = self.interface.right_axis_X * scaler
                                    y = self.interface.right_axis_Y * scaler
                                    if self.interface.button_right_grip:
                                        z = self.interface.right_trigger * scaler
                                    else:
                                        z = -self.interface.right_trigger * scaler
                                    self.sub_controller.xyz_move([x, y, z])

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
                        if topic == 'pan_tilt':
                            return
                        print(f"❌ Error processing topic '{topic}': {e}")
                        pass