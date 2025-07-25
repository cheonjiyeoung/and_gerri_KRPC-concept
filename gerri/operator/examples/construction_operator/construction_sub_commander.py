import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.interface.construction_ui.main_ui import ConstructionOperatorUI
from gerri.operator.examples.construction_operator import doosanM_right_commander
from gerri.operator.examples.construction_operator import doosanM_left_commander
from gerri.operator.examples.construction_operator.construction_media_receiver import OperatorMediaReceiever
from gerri.operator.examples.construction_operator.construction_config import ROBOT_INFO, OPERATOR_INFO, VIDEO_INFO, AUDIO_INFO
from PySide6.QtGui import QPixmap, QPainter, QPen, QImage
from PySide6.QtWidgets import QApplication, QMainWindow, QGraphicsView, QGraphicsScene, QGraphicsProxyWidget
from PySide6.QtCore import Qt
import pickle
from pubsub import pub


import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import cv2


class ConstructionSubCommander(Node, ConstructionOperatorUI):
    def __init__(self, **kwargs):
        Node.__init__(self, 'construction_joint_publisher')  # ROS 2 노드 초기화

        app = kwargs.get('app')
        if app is None:
            raise ValueError("app 파라미터가 필요합니다.")

        screen = app.primaryScreen()
        full_size = screen.geometry()
        print(f"전체 화면 크기: {full_size.width()} x {full_size.height()}")

        # 사용 가능한 영역 (작업 표시줄 등 제외)
        available_size = screen.availableGeometry()
        print(f"사용 가능한 크기: {available_size.width()} x {available_size.height()}")
        widget_w = available_size.width() - 70
        widget_h = available_size.height() - 60
        ConstructionOperatorUI.__init__(self, 1920-70, 1080-60)
        # self.window = SpotOperatorUI(widget_w, widget_h)
        self.setWindowTitle("Construction Robot Controller") 
        self.show()

        self.rgb_receiver = OperatorMediaReceiever(ROBOT_INFO, OPERATOR_INFO, 'realsense_D435if')
        self.depth_receiver = OperatorMediaReceiever(ROBOT_INFO, OPERATOR_INFO, 'realsense_D435if_depth')

        self.doosanM_left = doosanM_left_commander.DoosanMLeftCommander()
        self.doosanM_right = doosanM_right_commander.DoosanMRightCommander()

        self.pixmap_rgb = None

        self.webrtc_connection.Connect_Command.clicked.connect(self.clicked_connect_command)
        self.webrtc_connection.Disconnect_Command.clicked.connect(self.clicked_disconnect_command)
        self.webrtc_connection.Connect_Camera.clicked.connect(self.clicked_connect_camera)
        self.webrtc_connection.Disconnect_Camera.clicked.connect(self.clicked_disconnect_camera)

        #### LEFT ARM
        self.robot_controller_left.MoveJ.clicked.connect(self.clicked_moveJ_left)
        self.robot_controller_left.MoveL.clicked.connect(self.clicked_moveL_left)
        self.robot_controller_left.getCurrentJoint.clicked.connect(self.clicked_getJointCurrent_left)
        self.robot_controller_left.getCurrentPose.clicked.connect(self.clicked_getCurrentPose_left)
        self.robot_controller_left.Stop.clicked.connect(self.clicked_left_stop)
        self.robot_controller_left.master_start.clicked.connect(self.clicked_master_start_left)
        self.robot_controller_left.master_stop.clicked.connect(self.clicked_master_stop_left)
        self.robot_controller_left.master_go_back.clicked.connect(self.doosanM_left.master_go_back)
        self.robot_controller_left.Reload_1.clicked.connect(self.doosanM_left.reload_1_btn)
        self.robot_controller_left.Reload_2.clicked.connect(self.doosanM_left.reload_2_btn)
        self.robot_controller_left.Reload_3.clicked.connect(self.doosanM_left.reload_3_btn)

        #### RIGHT ARM
        self.robot_controller_right.MoveJ.clicked.connect(self.clicked_moveJ_right)
        self.robot_controller_right.MoveL.clicked.connect(self.clicked_moveL_right)
        self.robot_controller_right.getCurrentJoint.clicked.connect(self.clicked_getJointCurrent_right)
        self.robot_controller_right.getCurrentPose.clicked.connect(self.clicked_getCurrentPose_right)
        self.robot_controller_right.Stop.clicked.connect(self.clicked_right_stop)
        self.robot_controller_right.master_start.clicked.connect(self.clicked_master_start_right)
        self.robot_controller_right.master_stop.clicked.connect(self.clicked_master_stop_right)
        self.robot_controller_right.master_go_back.clicked.connect(self.doosanM_right.master_go_back)

        self.connected = False
        self.connected_camera = False
        self.left_robot_state = "UNKNWON"
        self.right_robot_state = "UNKNOWN"

        self.enable_keyboard = False
        self._lock = threading.Lock()

        pub.subscribe(self.get_connection_state, "connection_state")
        pub.subscribe(self.get_connection_state_camera, "connection_state_camera")

        self.left_publisher = self.create_publisher(JointState, '/left/joint_states', 1)
        self.right_publisher = self.create_publisher(JointState, '/right/joint_states', 1)

        self.control_target = 'all'
        self.use_master_arm = False
        self.master_control = False
        self.base_commander = None

        # pub.subscribe(self.key_mouse_control, 'key_mouse_control')

    """
    Initializes the connection for the sub-function (e.g., hardware setup, activation).
    """
    def connect(self):
        print('######################### [Connecting to Gerri...] ####################################')
        left_joint_thread = threading.Thread(target=self.publish_left_joint_message)
        left_joint_thread.daemon = True
        left_joint_thread.start()
        right_joint_thread = threading.Thread(target=self.publish_right_joint_message)
        right_joint_thread.daemon = True
        right_joint_thread.start()
        self.rgb_receiver.connect()
        self.depth_receiver.connect()
        camera_view_th = threading.Thread(target=self.camera_view_thread)
        camera_view_th.daemon = True
        camera_view_th.start()
        depth_camera_view_th = threading.Thread(target=self.camera_view_depth_thread)
        depth_camera_view_th.daemon = True
        depth_camera_view_th.start()


    """
    Handles cleanup and shutdown for the sub-function (if applicable).
    """
    def disconnect(self):
        pass
        ### TODO : ADD DISCONNECT FUNCTION

    """
    Sets the reference to the base function instance.
    This allows the sub function to call base-level methods.
    """
    def init_base_commander(self, base_commander):
        self.base_commander = base_commander

    def publish_left_joint_message(self):
        while True:            
            msg = JointState()
            msg.name = self.doosanM_left.joint_names
            msg.header.stamp = self.get_clock().now().to_msg()

            ## For TEST
            # position_left = [90., 0., -90., 0., 45.0, 0.0]

            msg.position = [angle * math.pi / 180 for angle in self.doosanM_left.current_joint_position]

            ## For Test
            # msg.position = [angle * math.pi / 180 for angle in position_left]
            self._lock.acquire()
            self.left_publisher.publish(msg)
            self._lock.release()
            time.sleep(0.01)

    def publish_right_joint_message(self):
        while True:            
            msg = JointState()
            msg.name = self.doosanM_right.joint_names
            msg.header.stamp = self.get_clock().now().to_msg()

            ## For TEST
            # position_right = [-90., 0., 90., 0., -45.0, 0.0]

            msg.position = [angle * math.pi / 180 for angle in self.doosanM_right.current_joint_position]

            ## For Test
            # msg.position = [angle * math.pi / 180 for angle in position_right]
            self._lock.acquire()
            self.right_publisher.publish(msg)
            self._lock.release()
            time.sleep(0.01)

    def get_connection_state(self, connection_state):
        if connection_state:
            self.connected = True
            self.webrtc_connection.webrtc_connection_command_edit.setText("CONNECT")
        else:
            self.connected = False
            self.webrtc_connection.webrtc_connection_command_edit.setText("DISCONNECT")

    def get_connection_state_camera(self, connection_state):
        if connection_state:
            self.connected_camera = True
            self.webrtc_connection.webrtc_connection_camera_edit.setText("CONNECT")
        else:
            self.connected_camera = False
            self.webrtc_connection.webrtc_connection_camera_edit.setText("DISCONNECT")

    ### LEFT ARM
    def get_robot_state_left(self, current_state, current_joint, current_pose, current_orientation):
        # print(f"current_joint: {current_joint}")
        self.doosanM_left.current_joint_position = current_joint
        self.doosanM_left.current_point = current_pose
        self.doosanM_left.current_angle = current_orientation
        self.left_robot_state = current_state
        
        self.robot_controller_left.robot_state_edit.setText(self.left_robot_state)
        # print(self.current_joint_position)
        self.robot_controller_left.j1_edit.setText(str(round(current_joint[0] * 100) / 100))
        self.robot_controller_left.j2_edit.setText(str(round(current_joint[1] * 100) / 100))
        self.robot_controller_left.j3_edit.setText(str(round(current_joint[2] * 100) / 100))
        self.robot_controller_left.j4_edit.setText(str(round(current_joint[3] * 100) / 100))
        self.robot_controller_left.j5_edit.setText(str(round(current_joint[4] * 100) / 100))
        self.robot_controller_left.j6_edit.setText(str(round(current_joint[5] * 100) / 100))

        self.robot_controller_left.x_edit.setText(str(round(current_pose[0] * 1000) / 1000))
        self.robot_controller_left.y_edit.setText(str(round(current_pose[1] * 1000) / 1000))
        self.robot_controller_left.z_edit.setText(str(round(current_pose[2] * 1000) / 1000))

        self.robot_controller_left.rx_edit.setText(str(round(current_orientation[0] * 100) / 100))
        self.robot_controller_left.ry_edit.setText(str(round(current_orientation[1] * 100) / 100))
        self.robot_controller_left.rz_edit.setText(str(round(current_orientation[2] * 100) / 100))
    
    ## RIGHT ARM
    def get_robot_state_right(self, current_state, current_joint, current_pose, current_orientation):
        # print(f"current_joint: {current_joint}")
        self.doosanM_right.current_joint_position = current_joint
        self.doosanM_right.current_point = current_pose
        self.doosanM_right.current_angle = current_orientation
        self.right_robot_state = current_state
        
        self.robot_controller_right.robot_state_edit.setText(self.right_robot_state)
        # print(self.current_joint_position)
        self.robot_controller_right.j1_edit.setText(str(round(current_joint[0] * 100) / 100))
        self.robot_controller_right.j2_edit.setText(str(round(current_joint[1] * 100) / 100))
        self.robot_controller_right.j3_edit.setText(str(round(current_joint[2] * 100) / 100))
        self.robot_controller_right.j4_edit.setText(str(round(current_joint[3] * 100) / 100))
        self.robot_controller_right.j5_edit.setText(str(round(current_joint[4] * 100) / 100))
        self.robot_controller_right.j6_edit.setText(str(round(current_joint[5] * 100) / 100))

        self.robot_controller_right.x_edit.setText(str(round(current_pose[0] * 1000) / 1000))
        self.robot_controller_right.y_edit.setText(str(round(current_pose[1] * 1000) / 1000))
        self.robot_controller_right.z_edit.setText(str(round(current_pose[2] * 1000) / 1000))

        self.robot_controller_right.rx_edit.setText(str(round(current_orientation[0] * 100) / 100))
        self.robot_controller_right.ry_edit.setText(str(round(current_orientation[1] * 100) / 100))
        self.robot_controller_right.rz_edit.setText(str(round(current_orientation[2] * 100) / 100))

    def camera_view_thread(self):
        while True:
            if self.rgb_receiver.last_frame is not None:
                self._lock.acquire()
                rgb_frame = self.convert_frame_to_pixmap(self.rgb_receiver.last_frame)
                self.camera_rgb_realsense_view.cam_view.setPixmap(rgb_frame)
                # self.camera_rgb_realsense_view.cam_view.setScaledContents(True)
                self._lock.release()
            time.sleep(0.05)

    def camera_view_depth_thread(self):
        while True:
            if self.depth_receiver.last_frame is not None:
                self._lock.acquire()
                depth_frame = self.convert_frame_to_pixmap(self.depth_receiver.last_frame)
                self.camera_depth_realsense_view.cam_view.setPixmap(depth_frame)
                # self.camera_rgb_realsense_view.cam_view.setScaledContents(True)
                self._lock.release()
            time.sleep(0.05)

    def convert_frame_to_pixmap(self, frame):
        # print(frame)
        fram_array = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = fram_array.shape
        bytes_per_line = ch * w
        return QPixmap.fromImage(QImage(fram_array.data, w, h, bytes_per_line, QImage.Format_RGB888))    

    def clicked_connect_command(self):
        self.log_message.log_msg_input(f"Connect to WebRTC")
        if not self.connected:
            pub.sendMessage('connect_webrtc', connect_webrtc=True)
        else:
            print("Already connected to Gerri")

    def clicked_disconnect_command(self):
        self.log_message.log_msg_input(f"Disconnect to WebRTC")
        if self.connected:
            pub.sendMessage('connect_webrtc', connect_webrtc=False)
        else:
            print("Already disconnected to Gerri")

    def clicked_connect_camera(self):
        self.log_message.log_msg_input(f"Connect to Camera")
        if not self.connected_camera:
            pub.sendMessage('connect_webrtc_camera', connect_webrtc_camera=True)
        else:
            print("Already connected to Camera")

    def clicked_disconnect_camera(self):
        self.log_message.log_msg_input(f"Disconnect to Camera")
        if self.connected_camera:
            pub.sendMessage('connect_webrtc_camera', connect_webrtc_camera=False)
        else:
            print("Already disconnected to Camera")


    ####################    
    # LEFT ARM
    #####################
    def clicked_left_stop(self):
        self.log_message.log_msg_input(f"Stop Moving")
        self.doosanM_left.stop()

    def clicked_moveJ_left(self):
        if self.doosanM_left.master_start == False:
            self.log_message.log_msg_input(f"Left Arm MoveJ, {self.robot_controller_left.target_j1.value()}, {self.robot_controller_left.target_j2.value()}, {self.robot_controller_left.target_j3.value()}, {self.robot_controller_left.target_j4.value()}, {self.robot_controller_left.target_j5.value()}, {self.robot_controller_left.target_j6.value()}")
            self.doosanM_left.moveJ(self.robot_controller_left.target_j1.value(), self.robot_controller_left.target_j2.value(), self.robot_controller_left.target_j3.value(),
                                    self.robot_controller_left.target_j4.value(), self.robot_controller_left.target_j5.value(), self.robot_controller_left.target_j6.value(),
                                    self.robot_controller_left.movejVel.value(), self.robot_controller_left.movejAcc.value())
        else:
            self.log_message.log_msg_input(f"Turn Off Left Master")

    def clicked_moveL_left(self):
        if self.doosanM_left.master_start == False:
            self.log_message.log_msg_input(f"Left Arm MoveL, {self.robot_controller_left.target_x.value()}, {self.robot_controller_left.target_y.value()}, {self.robot_controller_left.target_z.value()}, {self.robot_controller_left.target_rx.value()}, {self.robot_controller_left.target_ry.value()}, {self.robot_controller_left.target_rz.value()}")
            self.doosanM_left.moveL(self.robot_controller_left.target_x.value(), self.robot_controller_left.target_y.value(), self.robot_controller_left.target_z.value(),
                                    self.robot_controller_left.target_rx.value(), self.robot_controller_left.target_rx.value(), self.robot_controller_left.target_rz.value(),
                                    self.robot_controller_left.movelVel.value(), self.robot_controller_left.movelAcc.value())
        else:
            self.log_message.log_msg_input(f"Turn Off Left Master")

    def clicked_getJointCurrent_left(self):
        self.log_message.log_msg_input(f"Get Current Left Joint, {self.doosanM_left.current_joint_position[0]}, {self.doosanM_left.current_joint_position[1]}, {self.doosanM_left.current_joint_position[2]}, {self.doosanM_left.current_joint_position[3]}, {self.doosanM_left.current_joint_position[4]}, {self.doosanM_left.current_joint_position[5]}")
        self.robot_controller_left.target_j1.setValue(round(self.doosanM_left.current_joint_position[0] * 100) / 100)
        self.robot_controller_left.target_j2.setValue(round(self.doosanM_left.current_joint_position[1] * 100) / 100)
        self.robot_controller_left.target_j3.setValue(round(self.doosanM_left.current_joint_position[2] * 100) / 100)
        self.robot_controller_left.target_j4.setValue(round(self.doosanM_left.current_joint_position[3] * 100) / 100)
        self.robot_controller_left.target_j5.setValue(round(self.doosanM_left.current_joint_position[4] * 100) / 100)
        self.robot_controller_left.target_j6.setValue(round(self.doosanM_left.current_joint_position[5] * 100) / 100)

    def clicked_getCurrentPose_left(self):
        self.log_message.log_msg_input(f"Get Current Left Pose, {self.doosanM_left.current_point[0]}, {self.doosanM_left.current_point[1]}, {self.doosanM_left.current_point[2]}, {self.doosanM_left.current_angle[0]}, {self.doosanM_left.current_angle[1]}, {self.doosanM_left.current_angle[2]}")
        self.robot_controller_left.target_x.setValue(round(self.doosanM_left.current_point[0] * 1000) / 1000)
        self.robot_controller_left.target_y.setValue(round(self.doosanM_left.current_point[1] * 1000) / 1000)
        self.robot_controller_left.target_z.setValue(round(self.doosanM_left.current_point[2] * 1000) / 1000)

        self.robot_controller_left.target_rx.setValue(self.doosanM_left.current_angle[0])
        self.robot_controller_left.target_ry.setValue(self.doosanM_left.current_angle[1])
        self.robot_controller_left.target_rz.setValue(self.doosanM_left.current_angle[2])

    def clicked_master_start_left(self):
        self.log_message.log_msg_input(f"Left Master Start")
        self.doosanM_left.master_read_start()
        self.get_master_onoff_left()

    def clicked_master_stop_left(self):
        self.log_message.log_msg_input(f"Left Master Stop")
        self.doosanM_left.master_stop()
        self.get_master_onoff_left()

    def get_master_onoff_left(self):
        if self.doosanM_left.master_start:
            self.robot_controller_left.master_onoff_edit.setText(str("On"))
        elif not self.doosanM_left.master_start:
            self.robot_controller_left.master_onoff_edit.setText(str("Off"))


    ####################    
    # RIGHT ARM
    #####################
    def clicked_right_stop(self):
        self.log_message.log_msg_input(f"Stop Moving Right Arm")
        self.doosanM_right.stop()

    def clicked_moveJ_right(self):
        if self.doosanM_right.master_start == False:
            self.log_message.log_msg_input(f"Right Arm MoveJ, {self.robot_controller_right.target_j1.value()}, {self.robot_controller_right.target_j2.value()}, {self.robot_controller_right.target_j3.value()}, {self.robot_controller_right.target_j4.value()}, {self.robot_controller_right.target_j5.value()}, {self.robot_controller_right.target_j6.value()}")
            self.doosanM_right.moveJ(self.robot_controller_right.target_j1.value(), self.robot_controller_right.target_j2.value(), self.robot_controller_right.target_j3.value(),
                                     self.robot_controller_right.target_j4.value(), self.robot_controller_right.target_j5.value(), self.robot_controller_right.target_j6.value(),
                                     self.robot_controller_right.movejVel.value(),  self.robot_controller_right.movejAcc.value())
        else:
            self.log_message.log_msg_input(f"Turn Off Right Master")

    def clicked_moveL_right(self):
        if self.doosanM_right.master_start == False:
            self.log_message.log_msg_input(f"Right Arm MoveL, {self.robot_controller_right.target_x.value()}, {self.robot_controller_right.target_y.value()}, {self.robot_controller_right.target_z.value()}, {self.robot_controller_right.target_rx.value()}, {self.robot_controller_right.target_ry.value()}, {self.robot_controller_right.target_rz.value()}")
            self.doosanM_right.moveL(self.robot_controller_right.target_x.value(), self.robot_controller_right.target_y.value(), self.robot_controller_right.target_z.value(),
                                    self.robot_controller_right.target_rx.value(), self.robot_controller_right.target_rx.value(), self.robot_controller_right.target_rz.value(),
                                    self.robot_controller_right.movelVel.value(), self.robot_controller_right.movelAcc.value())
        else:
            self.log_message.log_msg_input(f"Turn Off Right Master")

    def clicked_getJointCurrent_right(self):
        self.log_message.log_msg_input(f"Get Current Right Arm Joint, {self.doosanM_right.current_joint_position[0]}, {self.doosanM_right.current_joint_position[1]}, {self.doosanM_right.current_joint_position[2]}, {self.doosanM_right.current_joint_position[3]}, {self.doosanM_right.current_joint_position[4]}, {self.doosanM_right.current_joint_position[5]}")
        self.robot_controller_right.target_j1.setValue(round(self.doosanM_right.current_joint_position[0] * 100) / 100)
        self.robot_controller_right.target_j2.setValue(round(self.doosanM_right.current_joint_position[1] * 100) / 100)
        self.robot_controller_right.target_j3.setValue(round(self.doosanM_right.current_joint_position[2] * 100) / 100)
        self.robot_controller_right.target_j4.setValue(round(self.doosanM_right.current_joint_position[3] * 100) / 100)
        self.robot_controller_right.target_j5.setValue(round(self.doosanM_right.current_joint_position[4] * 100) / 100)
        self.robot_controller_right.target_j6.setValue(round(self.doosanM_right.current_joint_position[5] * 100) / 100)

    def clicked_getCurrentPose_right(self):
        self.log_message.log_msg_input(f"Get Current Right Arm Pose, {self.doosanM_right.current_point[0]}, {self.doosanM_right.current_point[1]}, {self.doosanM_right.current_point[2]}, {self.doosanM_right.current_angle[0]}, {self.doosanM_right.current_angle[1]}, {self.doosanM_right.current_angle[2]}")
        self.robot_controller_right.target_x.setValue(round(self.doosanM_right.current_point[0] * 1000) / 1000)
        self.robot_controller_right.target_y.setValue(round(self.doosanM_right.current_point[1] * 1000) / 1000)
        self.robot_controller_right.target_z.setValue(round(self.doosanM_right.current_point[2] * 1000) / 1000)

        self.robot_controller_right.target_rx.setValue(self.doosanM_right.current_angle[0])
        self.robot_controller_right.target_ry.setValue(self.doosanM_right.current_angle[1])
        self.robot_controller_right.target_rz.setValue(self.doosanM_right.current_angle[2])

    def clicked_master_start_right(self):
        self.log_message.log_msg_input(f"Right Master Start")
        # self.doosanM_right.master_read_start()
        self.get_master_onoff_right()

    def clicked_master_stop_right(self):
        self.log_message.log_msg_input(f"Right Master Stop")
        self.doosanM_right.master_stop()
        self.get_master_onoff_right()

    def get_master_onoff_right(self):
        if self.doosanM_right.master_start:
            self.robot_controller_right.master_onoff_edit.setText(str("On"))
        elif not self.doosanM_right.master_start:
            self.robot_controller_right.master_onoff_edit.setText(str("Off"))
