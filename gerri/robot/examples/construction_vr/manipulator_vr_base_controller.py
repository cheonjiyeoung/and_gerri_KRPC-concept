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


class ManipulatorVRBaseController:
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.sub_controller = None
        self.start_pose = None
        self.target_pose = None

        # --- 사용자 정의 변수 (T_도착 <- 출발) ---
        T_base_world = tf_from_rpy_deg([0, 45, -90])
        T_ee_world = tf_from_rpy_deg([0, 0, 180])
        T_vr_world = tf_from_axis_map(['-y', '-x', 'z'])
        T_ctrl_world = tf_from_rpy_deg([0, 0, -90])


        # self.T_ros_rh = tf_from_axis_map(['x', '-y', 'z'])
        # self.T_ros_rh = tf_from_rpy_deg([180, 0, 90])

        # --- Pinocchio 표준 변수로 변환 (T_도착 <- 출발) ---
        # 실제 계산에는 Pinocchio 표준에 맞는 변수명을 사용합니다.
        # T_world_base는 base에서 world로 가는 변환을 의미합니다.
        self.T_world_base = T_base_world.inverse()
        self.T_world_ee = T_ee_world.inverse()
        self.T_world_vr = T_vr_world.inverse()
        self.T_world_ctrl = T_ctrl_world.inverse()
        self.T_base_vr = self.T_world_base.inverse() * self.T_world_vr
        # --- 마스터 VR 변환 행렬 ---
        self.T_world_vrctrl = self.T_world_ctrl * self.T_world_vr



        if 'interface' in params:
            self.interface = params['interface']
            self.last_interface_value = copy.deepcopy(self.interface)
        else:
            self.interface = None
            self.last_interface_value = None

        pub.subscribe(self.receive_message,"receive_message")

    def connect(self):
        self.sub_controller.connect()
        self.status_manager = StatusManager(self.robot_info, self.sub_controller)

    def disconnect(self):
        self.sub_controller.disconnect()

    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

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
                                delta_ros = self.interface.right_delta_pose

                                # 2. 컨트롤 좌표계 기준으로 ROS 변화량을 해석하여 월드 공간의 변화량으로 만듭니다.
                                delta_world = self.T_world_ctrl * delta_ros

                                # 3. 월드 기준 시작 자세를 계산합니다.
                                start_pose_world = self.T_world_base * self.start_pose

                                print(delta_ros)

                                # 4. 월드 기준 시작 자세에 월드 기준 변화량을 적용하여 목표 자세를 계산합니다.
                                target_pose_world = start_pose_world * delta_world


                                # 5. 월드 기준 목표 자세를 로봇 베이스 기준으로 변환합니다.
                                target_pose_base = self.T_world_base.inverse() * target_pose_world

                                # 6. 최종 변환된 목표 자세를 CLIK 속도 제어 함수로 전달합니다.
                                self.sub_controller.joint_ctrl_vel(target_pose_base)

                                # # 1. VR 컨트롤러의 전체 6축(위치+자세) 변화량(delta)을 가져옵니다.
                                # delta_vr = self.interface.right_delta_pose
                                #
                                # # 2. VR 컨트롤러의 변화량을 월드 좌표계 기준으로 변환합니다.
                                # delta_world = self.T_vrctrl_ros * delta_vr
                                #
                                # # 3. 로봇의 시작 자세(월드 기준)에 월드 기준 변화량을 적용하여
                                # #    최종 목표 자세(월드 기준)를 계산합니다.
                                # start_pose_world = self.T_world_base * self.start_pose
                                # target_pose_world = start_pose_world * delta_world
                                #
                                #
                                # # 4. 최종 목표 자세(월드 기준)를 다시 로봇 베이스 좌표계 기준으로 변환합니다.
                                # target_pose_base = self.T_world_base.inverse() * target_pose_world
                                #
                                #
                                # # 5. 계산된 최종 목표 자세를 새로운 속도 제어 함수로 직접 전달합니다.
                                # self.sub_controller.joint_ctrl_vel(target_pose_base)

                                # 위치 제어
                                # delta_vr = self.interface.right_delta_pose
                                # delta_world = self.T_vrctrl_ros * delta_vr
                                #
                                # # 3. 로봇의 '시작 자세'를 월드 기준으로 변환
                                # start_pose_world = self.T_world_base * self.start_pose
                                #
                                # # 4. 월드 기준 '목표 자세' 계산 = 월드 기준 '시작 자세' * 월드 기준 '변화량'
                                # target_pose_world = start_pose_world * delta_world
                                #
                                # # 5. 월드 기준 '목표 자세'를 다시 로봇 베이스 기준으로 변환
                                # target_pose_base = self.T_world_base.inverse() * target_pose_world
                                #
                                # # 6. 최종 '베이스 기준 목표 자세'를 로봇에게 전송
                                # self.sub_controller.end_pose_ctrl(target_pose_base)

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