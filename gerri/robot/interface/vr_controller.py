import pinocchio as pin
import numpy as np

class VRController:
    """VR 컨트롤러의 현재 입력을 파싱하고 상태를 저장하는 데이터 클래스."""

    def __init__(self, axis_map=None, rotation_offset_q=None, position_offset=None, rpy=None):
        """
        :param axis_map: ['x','y','z'] 로봇의 (X,Y,Z)가 VR의 어느 축에 해당하는지. 예: ['z', '-x', 'y']
        :param rotation_offset_q: [w,x,y,z] 회전을 쿼터니언으로 직접 지정
        :param position_offset: [x,y,z] 위치 오프셋 지정
        """
        self.name = 'vr_controller'

        # --- 초기 좌표 변환 설정 ---
        self.set_transform(axis_map, rotation_offset_q, position_offset, rpy)

        # --- Left Controller ---
        self.left_axis_X = 0.0
        self.left_axis_Y = 0.0
        self.left_trigger = 0.0
        self.left_grip = 0.0
        self.button_X = False
        self.button_Y = False
        self.button_left_thumbstick = False
        self.button_left_trigger = False
        self.button_left_grip = False
        self.button_left_menu = False
        self.left_position = [0, 0, 0]
        self.left_rotation = [0, 0, 0, 1]  # x,y,z,w

        # --- Right Controller ---
        self.right_axis_X = 0.0
        self.right_axis_Y = 0.0
        self.right_trigger = 0.0
        self.right_grip = 0.0
        self.button_A = False
        self.button_B = False
        self.button_right_thumbstick = False
        self.button_right_trigger = False
        self.button_right_grip = False
        self.button_right_menu = False
        self.right_position = [0, 0, 0]
        self.right_rotation = [0, 0, 0, 1]  # x,y,z,w

        # 현재(Current) Pose
        self.left_current_pose = pin.SE3.Identity()
        self.right_current_pose = pin.SE3.Identity()

        # 초기(Initial) Pose - 클러치 기준점
        self.left_initial_pose = pin.SE3.Identity()
        self.right_initial_pose = pin.SE3.Identity()

        # 델타(Delta) Pose - 초기 자세 대비 변화량
        self.left_delta_pose = pin.SE3.Identity()
        self.right_delta_pose = pin.SE3.Identity()


    def update(self, vr_data):
        """최신 VR 데이터로 모든 상태 변수를 업데이트합니다."""

        # Left controller
        if vr_data['left']:
            left = vr_data.get('left', {})
            left_axes = left.get('axes', [0, 0, 0, 0])
            left_buttons = left.get('buttons', [False] * 6)

            self.left_axis_X = left_axes[0]
            self.left_axis_Y = left_axes[1]
            self.left_trigger = left_axes[2]
            self.left_grip = left_axes[3]
            self.button_X = left_buttons[0]
            self.button_Y = left_buttons[1]
            self.button_left_thumbstick = left_buttons[2]
            self.button_left_trigger = left_buttons[3]
            self.button_left_grip = left_buttons[4]
            self.button_left_menu = left_buttons[5]
            self.left_position = left.get('position', self.left_position)
            self.left_rotation = left.get('rotation', self.left_rotation)

            # 왼쪽 손 현재 Pose 업데이트 및 델타 계산
            left_pos = left.get('position', [0,0,0])
            left_rot_xyzw = left.get('rotation', [0,0,0,1])
            self.left_current_pose = self.tf_vr_to_robot(pin.SE3(pin.Quaternion(np.array(left_rot_xyzw)), np.array(left_pos)))
            self.left_delta_pose = self.left_initial_pose.inverse() * self.left_current_pose

        else:
            self.left_axis_X = 0.0
            self.left_axis_Y = 0.0
            self.left_trigger = 0.0
            self.left_grip = 0.0
            self.button_X = False
            self.button_Y = False
            self.button_left_thumbstick = False
            self.button_left_trigger = False
            self.button_left_grip = False
            self.button_left_menu = False
            self.left_position = [0, 0, 0]
            self.left_rotation = [0, 0, 0, 1]  # x,y,z,w

            # 현재(Current) Pose
            self.left_current_pose = pin.SE3.Identity()

            # 초기(Initial) Pose - 클러치 기준점
            self.left_initial_pose = pin.SE3.Identity()

            # 델타(Delta) Pose - 초기 자세 대비 변화량
            self.left_delta_pose = pin.SE3.Identity()



        if vr_data['right']:
        # Right controller
            right = vr_data.get('right', {})
            right_axes = right.get('axes', [0, 0, 0, 0])
            right_buttons = right.get('buttons', [False] * 6)

            self.right_axis_X = right_axes[0]
            self.right_axis_Y = right_axes[1]
            self.right_trigger = right_axes[2]
            self.right_grip = right_axes[3]
            self.button_A = right_buttons[0]
            self.button_B = right_buttons[1]
            self.button_right_thumbstick = right_buttons[2]
            self.button_right_trigger = right_buttons[3]
            self.button_right_grip = right_buttons[4]
            self.button_right_menu = right_buttons[5]
            self.right_position = right.get('position', self.right_position)
            self.right_rotation = right.get('rotation', self.right_rotation)

            # 오른쪽 손 현재 Pose 업데이트 및 델타 계산
            right_pos = right.get('position', [0,0,0])
            right_rot_xyzw = right.get('rotation', [0,0,0,1])
            self.right_current_pose = self.tf_vr_to_robot(pin.SE3(pin.Quaternion(np.array(right_rot_xyzw)), np.array(right_pos)))
            self.right_delta_pose = self.right_initial_pose.inverse() * self.right_current_pose

        else:

            self.right_axis_X = 0.0
            self.right_axis_Y = 0.0
            self.right_trigger = 0.0
            self.right_grip = 0.0
            self.button_A = False
            self.button_B = False
            self.button_right_thumbstick = False
            self.button_right_trigger = False
            self.button_right_grip = False
            self.button_right_menu = False
            self.right_position = [0, 0, 0]
            self.right_rotation = [0, 0, 0, 1]  # x,y,z,w

            # 현재(Current) Pose
            self.right_current_pose = pin.SE3.Identity()

            # 초기(Initial) Pose - 클러치 기준점
            self.right_initial_pose = pin.SE3.Identity()

            # 델타(Delta) Pose - 초기 자세 대비 변화량
            self.right_delta_pose = pin.SE3.Identity()

    def reset_initial_pose(self, hand: str):
        """로봇 컨트롤러의 판단에 따라 호출되는 함수. 현재 자세를 새로운 기준점으로 설정."""
        if hand == 'left':
            self.left_initial_pose = self.left_current_pose.copy()
        elif hand == 'right':
            self.right_initial_pose = self.right_current_pose.copy()


    def tf_vr_to_robot(self, delta_pose_vr: pin.SE3) -> pin.SE3:
        """인스턴스에 저장된 변환 행렬을 사용해 델타 Pose를 변환합니다."""
        return self.T_vr_robot * delta_pose_vr * self.T_vr_robot.inverse()


    def set_transform(self, axis_map=None, rotation_offset_q=None, position_offset=None, rpy=None):
        """
        좌표 변환 설정을 동적으로 업데이트합니다.
        """
        print("🔄 좌표 변환 설정을 업데이트합니다.")
        self.T_vr_robot = self._build_transform(axis_map, rotation_offset_q, position_offset, rpy)


    def _build_transform(self, axis_map, rotation_offset_q, position_offset, rpy) -> pin.SE3:
        """직관적인 파라미터로부터 SE3 변환 행렬을 생성합니다."""
        R = np.identity(3)
        t = np.array(position_offset if position_offset is not None else [0,0,0], dtype=float)

        if rotation_offset_q is not None:
            q = pin.Quaternion(rotation_offset_q[1], rotation_offset_q[2], rotation_offset_q[3], rotation_offset_q[0]).normalize()
            R = q.toRotationMatrix()
        elif axis_map is not None:
            axis_map_dict = {'x': [1,0,0], 'y': [0,1,0], 'z': [0,0,1],
                             '-x': [-1,0,0], '-y': [0,-1,0], '-z': [0,0,-1]}
            new_x_axis = np.array(axis_map_dict[axis_map[0]])
            new_y_axis = np.array(axis_map_dict[axis_map[1]])
            new_z_axis = np.array(axis_map_dict[axis_map[2]])
            R = np.array([new_x_axis, new_y_axis, new_z_axis]).T

        elif rpy is not None:
            # 💡 사용자가 입력한 '도(degree)'를 '라디안(radian)'으로 변환합니다.
            rpy_rad = np.deg2rad(rpy)

            # 라디안 값으로 회전 행렬을 생성
            R = pin.rpy.rpyToMatrix(rpy_rad[0], rpy_rad[1], rpy_rad[2])


        return pin.SE3(R, t)
