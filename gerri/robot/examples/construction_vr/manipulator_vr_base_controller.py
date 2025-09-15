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

        # --- 좌표계 변환 행렬 정의 ---
        # 1. 월드 좌표계에서 본 로봇 베이스의 자세
        self.T_world_base = tf_from_rpy_deg([0, 45, -90])

        # 2. Quest 원본 -> ROS 표준 좌표계 변환 (tf_helper.py의 tf_preset 사용)
        self.T_ros_quest = tf_from_axis_map(['x', '-y', 'z'])

        # 3. ROS 표준 -> 사용자 컨트롤 좌표계 변환 ('z'가 앞, 'x'가 위, 'y'가 오른쪽)
        self.T_hctrl_ros = tf_from_axis_map(['z', '-y', 'x'])

        # 4. Quest -> 사용자 컨트롤(Human Control) 좌표계로 한 번에 변환하는 "마스터 변환 행렬"
        # 제어 루프에서 계산을 단순화하기 위해 미리 곱해둡니다.
        self.T_hctrl_quest = self.T_hctrl_ros * self.T_ros_quest

        np.printoptions(suppress=True)


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
                                # 1. VR 인터페이스에서 '초기 자세 대비 변화량'을 가져옴 (Quest 좌표계 기준)
                                delta_quest = self.interface.right_delta_pose
                                # delta_ctrl_ros = self.T_hctrl_ros * delta_quest * self.T_hctrl_ros.inverse()

                                # # 2. 'Quest 기준 변화량'을 '사용자 컨트롤(월드 기준) 변화량'으로 변환
                                # # T * delta * T_inv 공식 사용
                                delta_world = self.T_hctrl_quest * delta_quest * self.T_hctrl_quest.inverse()
                                #
                                # # 3. 로봇의 '시작 자세'를 월드 기준으로 변환
                                # start_pose_world = self.T_world_base * self.start_pose
                                #
                                # # 4. 월드 기준 '목표 자세' 계산 = 월드 기준 '시작 자세' * 월드 기준 '변화량'
                                # target_pose_world = start_pose_world * delta_world
                                #
                                #
                                # # 5. 월드 기준 '목표 자세'를 다시 로봇 베이스 기준으로 변환
                                # target_pose_base = self.T_world_base.inverse() * target_pose_world

                                target_pose = self.start_pose * delta_quest
                                print(np.round(se3_to_pose(delta_quest),2))
                                print(np.round(se3_to_pose(target_pose),2))
                                # target_pose = self.start_pose * delta_world
                                # target_pose = self.start_pose * delta_ctrl_ros

                                # 6. 최종 '베이스 기준 목표 자세'를 로봇에게 전송
                                self.sub_controller.end_pose_ctrl(target_pose)

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
