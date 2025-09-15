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

        # 최종 마스터 변환 행렬: T_base_vr (베이스에서 본 VR컨트롤러의 자세)
        # VR -> World -> Base 순서로 변환을 합성
        self.T_quest_to_ros = tf_preset['quest_to_ros']

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
                                # print(self.interface.right_current_pose, self.interface.right_delta_pose)
                                # 1. VR의 원본 delta (VR 좌표계 기준)
                                raw_delta_pose_vr = self.interface.right_delta_pose
                                # print(raw_delta_pose_vr)
                                print('r : ',np.round(se3_to_pose(raw_delta_pose_vr, 'mm'), 2))

                                # 2. 마스터 변환 행렬을 이용해 'VR 기준 delta'를 '로봇 베이스 기준 delta'로 변환
                                delta_pose_base = self.T_quest_to_ros * raw_delta_pose_vr * self.T_quest_to_ros.inverse()
                                print('d : ',np.round(se3_to_pose(delta_pose_base, 'mm'),2))

                                # 3. SubController에는 '베이스 기준'의 start_pose와 '베이스 기준'의 delta_pose를 전달
                                self.sub_controller.end_pose_ctrl_delta(self.start_pose, delta_pose_base)

                                # 그리퍼 제어 (오른쪽 트리거)
                                # gripper_value = round(1 - self.interface.right_trigger, 1)
                                # self.sub_controller.gripper_ctrl(gripper_value)

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
