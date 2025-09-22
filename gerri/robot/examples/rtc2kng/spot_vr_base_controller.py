import asyncio
from pubsub import pub
import os, sys
import math

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.status_manager import StatusManager
from gerri.robot.examples.spot_arm.spot_arm_sub_controller import SpotArmSubController, RobotMode


class SpotArmBaseController:
    def __init__(self, robot_info, **params):
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        self.last_vr_position = None
        self.last_vr_rotation = None

        # --- 제어 로직을 위한 변수 추가 ---
        self.initial_vr_pose = None
        self.initial_robot_pose = None
        self.enable_arm = False

        # --- 좌표계 변환을 위한 쿼터니언 ---
        self.q_vr_to_hand_coord = [0.5, -0.5, -0.5, 0.5]
        self.q_hand_to_vr_coord = self.quaternion_inverse(self.q_vr_to_hand_coord)


        if 'use_bridge' in params and params['use_bridge']:
            self.bridge = True
        else:
            self.bridge = False

        if 'sub_controller' in params:
            self.sub_controller = params['sub_controller']
        else:
            self.sub_controller = self.init_sub_controller(**params)

        if hasattr(self.sub_controller, 'init_base_controller'):
            self.sub_controller.init_base_controller(base_controller=self)

        self.status_manager = StatusManager(robot_info, self.sub_controller)
        pub.subscribe(self.receive_message,"receive_message")

    def init_sub_controller(self, **params):
        """
        로봇 모델에 따라 적절한 컨트롤러를 초기화.

        :param robot_model: 로봇 모델
        :param kwargs: 추가적인 파라미터
        :return: 특정 모델 컨트롤러 인스턴스
        """
        if self.robot_model == 'spot_arm':
            return SpotArmSubController()
        else:
            raise ValueError(f"Unsupported robot model: {self.robot_model}")

    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == 'vr_controller':
                            # Left controller
                            left = value.get('left', {})
                            left_axes = left.get('axes', [0, 0, 0, 0])
                            left_buttons = left.get('buttons', [False, False, False, False, False, False])

                            left_axis_X = left_axes[0]
                            left_axis_Y = left_axes[1]
                            left_trigger = left_axes[2]
                            left_grip = left_axes[3]

                            button_X = left_buttons[0]
                            button_Y = left_buttons[1]
                            button_left_thumbstick = left_buttons[2]
                            button_left_trigger = left_buttons[3]
                            button_left_grip = left_buttons[4]
                            button_left_menu = left_buttons[5]

                            # Right controller
                            right = value.get('right', {})
                            right_axes = right.get('axes', [0, 0, 0, 0])
                            right_buttons = right.get('buttons', [False, False, False, False, False, False])

                            right_axis_X = right_axes[0]
                            right_axis_Y = right_axes[1]
                            right_trigger = right_axes[2]
                            right_grip = right_axes[3]

                            button_A = right_buttons[0]
                            button_B = right_buttons[1]
                            button_right_thumbstick = right_buttons[2]
                            button_right_trigger = right_buttons[3]
                            button_right_grip = right_buttons[4]
                            button_right_menu = right_buttons[5]

                            if button_A:
                                self.sub_controller._walk()
                            if button_B:
                                self.sub_controller._stand()
                            if button_X:
                                self.sub_controller._sit()
                            if button_Y:
                                if button_left_grip:
                                    self.sub_controller._unstow()
                                    self.enable_arm = True
                                else:
                                    self.sub_controller._stow()
                                    self.enable_arm = False

                            if self.sub_controller.mode == RobotMode.Stand:
                                if button_right_grip:
                                    self.sub_controller._orientation_cmd_helper(yaw = -left_axis_X, roll = - right_axis_X * 0.8, pitch = -left_axis_Y * 0.6)


                            if self.sub_controller.mode == RobotMode.Walk:
                                self.sub_controller._move(left_x=left_axis_X, left_y=left_axis_Y, right_x=right_axis_X)

                            if (gripper_effort := self.sub_controller.joint_state['effort'][18]) > 0.5:
                                self.send_message({"topic": "haptic_feedback","target": "left","value": left_trigger,"option": {"duration": 0.2}})
                                map_effort = self.map_value(gripper_effort, 0.5, 5.5, 0, 1)
                                print(gripper_effort, map_effort)
                                self.send_message({"topic": "haptic_feedback","target": "right","value": map_effort,"option": {"duration": 0.2}})
                                # print(topic, left_trigger, right_trigger)

                            # --- 팔 제어 로직 (위치/회전 모두 절대 좌표 방식) ---
                            if self.enable_arm:
                                if button_right_grip:
                                    # 1. 현재 VR 컨트롤러의 위치와 회전 값 가져오기
                                    current_vr_pos = right.get('position')
                                    current_vr_rot = right.get('rotation')

                                    if current_vr_pos and current_vr_rot:
                                        # 2. 클러치(그립)를 처음 눌렀을 때, 초기 상태 저장
                                        if self.initial_robot_pose is None:
                                            print("VR-Arm 제어 연결(Link) 시작...")
                                            # 로봇 팔의 현재 절대 Pose 가져오기
                                            self.initial_robot_pose = self.sub_controller.get_current_arm_pose_in_body_frame()

                                            if self.initial_robot_pose:
                                                # VR 컨트롤러의 초기 상태 저장
                                                self.initial_vr_pose = {
                                                    'pos': current_vr_pos,
                                                    'rot': [current_vr_rot[3], current_vr_rot[0], current_vr_rot[1],
                                                            current_vr_rot[2]]  # w,x,y,z
                                                }
                                                print("초기 상태 설정 완료. 이제부터 움직임을 따라합니다.")
                                            else:
                                                print("오류: 로봇 팔의 현재 상태를 가져올 수 없습니다.")
                                            return  # 첫 프레임은 상태 저장만 하고 종료

                                        # 3. 연결(Link)이 수립된 후, 매 프레임 목표 Pose 계산
                                        # 3-1. VR 좌표계에서의 변화량(Delta) 계산
                                        delta_pos_vr = [c - i for c, i in
                                                        zip(current_vr_pos, self.initial_vr_pose['pos'])]

                                        q_vr_current = [current_vr_rot[3], current_vr_rot[0], current_vr_rot[1],
                                                        current_vr_rot[2]]
                                        q_vr_initial_inv = self.quaternion_inverse(self.initial_vr_pose['rot'])
                                        delta_rot_vr = self.quaternion_multiply(q_vr_current, q_vr_initial_inv)

                                        # 3-2. VR 변화량을 로봇(Hand) 좌표계의 변화량으로 변환
                                        dx, dy, dz = self.convert_vr_to_hand_frame(delta_pos_vr)
                                        delta_rot_hand = self.transform_quaternion_coord(delta_rot_vr)

                                        # 3-3. 로봇의 초기 Pose에 변환된 변화량을 적용하여 최종 목표 Pose 계산
                                        target_pos_x = self.initial_robot_pose.position.x + dx
                                        target_pos_y = self.initial_robot_pose.position.y + dy
                                        target_pos_z = self.initial_robot_pose.position.z + dz

                                        q_robot_initial = [self.initial_robot_pose.rotation.w,
                                                           self.initial_robot_pose.rotation.x,
                                                           self.initial_robot_pose.rotation.y,
                                                           self.initial_robot_pose.rotation.z]
                                        target_rot_q = self.quaternion_multiply(delta_rot_hand, q_robot_initial)
                                        target_rot_q = self.quaternion_normalize(target_rot_q)

                                        # 4. 계산된 절대 목표 Pose로 이동 명령 전송
                                        self.sub_controller.move_arm_to_absolute_position_in_body_frame(
                                            x=target_pos_x, y=target_pos_y, z=target_pos_z,
                                            qw=target_rot_q[0], qx=target_rot_q[1], qy=target_rot_q[2],
                                            qz=target_rot_q[3],
                                            seconds=0.1
                                        )

                                        # 5. 그리퍼 제어
                                        self.sub_controller.gripper_percent(round(1 - right_trigger, 1))

                                else:  # 그립 버튼을 놓았을 때
                                    if self.initial_robot_pose is not None:
                                        print("VR-Arm 제어 연결(Link) 해제.")
                                        self.sub_controller.stop_arm()
                                    # 다음 제어를 위해 상태 리셋
                                    self.initial_vr_pose = None
                                    self.initial_robot_pose = None

                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                        pass
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")
                        pass


    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def connect(self):
        self.sub_controller.connect()

    def disconnect(self):
        self.sub_controller.disconnect()

    def convert_vr_to_hand_frame(self, delta):
        dx, dy, dz = delta
        return dz, -dx, dy # hand_dx, hand_dy, hand_dz

    def quaternion_inverse(self, q):
        w, x, y, z = q
        return [w, -x, -y, -z]

    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return [w, x, y, z]

    def quaternion_normalize(self, q):
        w, x, y, z = q
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm == 0: return [1, 0, 0, 0]
        return [c/norm for c in q]

    def transform_quaternion_coord(self, q_vr):
        temp_q = self.quaternion_multiply(q_vr, self.q_hand_to_vr_coord)
        return self.quaternion_multiply(self.q_vr_to_hand_coord, temp_q)


    @staticmethod
    def clamp(value, min_value, max_value, absolute_limit=None):
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """
        특정 값을 주어진 범위 내에서 다른 범위로 매핑하는 함수.

        :param value: 매핑할 값
        :param in_min: 매핑 전 최소값
        :param in_max: 매핑 전 최대값        :param out_min: 매핑 후 최소값
        :param out_max: 매핑 후 최대값
        :return: 매핑된 값
        """
        map_value = self.clamp((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max)

        return map_value
