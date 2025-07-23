from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QLineEdit, QGroupBox, QPushButton
)
from PySide6.QtGui import QPalette, QColor
from PySide6.QtCore import QTimer, Slot

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time


class MobileLiftUi(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, 'construction_mobile_lift')
        QMainWindow.__init__(self)

        self.joint_labels = [
            'x_slide_joint', 'y_slide_joint', 'yaw_joint',  # 항상 0.0 유지
            'lift_base_joint',
            'lift_first_joint_1',
            'lift_first_joint_2',
            'lift_second_joint',
            'torso_joint'
        ]

        self.fixed_zero_joints = {'x_slide_joint', 'y_slide_joint', 'yaw_joint'}  # 항상 0 유지
        self.joint_display_edits = {}
        self.joint_input_edits = {}
        self.joint_threads = {}
        self.joint_publish_flags = {name: False for name in self.joint_labels}
        self.joint_degrees = {name: 0.0 for name in self.joint_labels}
        self.joint_target_degrees = {name: 0.0 for name in self.joint_labels}

        self.setFixedSize(800, 400)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # === Current State Display ===
        state_layout = QGridLayout()
        for i, name in enumerate(self.joint_labels):
            label = QLabel(name)
            edit = QLineEdit("", readOnly=True)
            self.joint_display_edits[name] = edit
            state_layout.addWidget(label, i, 0)
            state_layout.addWidget(edit, i, 1)
        current_state_box = QGroupBox("Current Joint States (deg)")
        current_state_box.setLayout(state_layout)

        # === Control Panel ===
        control_layout = QGridLayout()
        for i, name in enumerate(self.joint_labels):
            input_edit = QLineEdit()
            input_edit.setPlaceholderText("deg")

            if name in self.fixed_zero_joints:
                input_edit.setReadOnly(True)
                input_edit.setPlaceholderText("None")
            if name == 'lift_first_joint_2':
                input_edit.setReadOnly(True)
                input_edit.setPlaceholderText("Auto Work")
            if name == 'torso_joint':
                input_edit.setReadOnly(True)
                input_edit.setPlaceholderText("Auto Work")

            button = QPushButton("Start Publishing")
            button.clicked.connect(lambda _, n=name: self.send_joint_state(n))

            self.joint_input_edits[name] = input_edit
            control_layout.addWidget(QLabel(name), i, 0)
            control_layout.addWidget(input_edit, i, 1)
            control_layout.addWidget(button, i, 2)

        control_box = QGroupBox("Control Panel")
        control_box.setLayout(control_layout)

        main_layout = QHBoxLayout()
        main_layout.addWidget(current_state_box)
        main_layout.addWidget(control_box)
        central_widget.setLayout(main_layout)

        # === ROS Communication ===
        self.latest_joint_state = None
        self.subscription = self.create_subscription(
            JointState,
            '/lift_base/joint_states',
            self.joint_state_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            JointState,
            '/lift_base/joint_states',
            10
        )

        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.update_gui_from_joint_state)
        self.gui_timer.start(100)

        self.ros_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
        self.ros_thread.start()

        QTimer.singleShot(500, self.publish_initial_zero)

    def _ros_spin_loop(self):
        rclpy.spin(self)

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    @Slot()
    def update_gui_from_joint_state(self):
        if not self.latest_joint_state:
            return
        joint_map = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        for name in self.joint_labels:
            if name in joint_map:
                deg = round(joint_map[name] * 180.0 / 3.14159, 2)
                self.joint_display_edits[name].setText(str(deg))

    def publish_initial_zero(self):
        for name in self.joint_labels:
            self.joint_degrees[name] = 0.0
        self.publish_full_joint_state()

    def publish_full_joint_state(self):
        # 항상 x, y, yaw는 0으로 고정
        for fixed in self.fixed_zero_joints:
            self.joint_degrees[fixed] = 0.0

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_labels
        msg.position = [
            self.joint_degrees[name] * 3.14159 / 180.0 for name in self.joint_labels
        ]
        self.publisher_.publish(msg)

    def publisher_thread(self, joint_name):
        while True:
            target = self.joint_target_degrees[joint_name]
            current = self.joint_degrees[joint_name]
            step = 0.1 if target > current else -0.1
            if abs(target - current) < 0.5:
                step = 0
            next_value = current + step if step != 0 else current

            self.joint_degrees[joint_name] = next_value
            if joint_name == 'lift_first_joint_1':
                self.joint_degrees['lift_first_joint_2'] = -next_value

            if joint_name == 'lift_second_joint':
                self.joint_degrees['torso_joint'] = -next_value

            self.publish_full_joint_state()
            time.sleep(0.01)

    @Slot()
    def send_joint_state(self, joint_name):
        if joint_name in self.fixed_zero_joints or joint_name == 'lift_first_joint_2':
            print(f"[Error] {joint_name}은 직접 제어할 수 없습니다.")
            return
        try:
            deg_str = self.joint_input_edits[joint_name].text()
            deg = float(deg_str)

            if joint_name == 'lift_first_joint_1' and deg > 0:
                print("[Error] lift_first_joint_1은 음수만 입력해야 합니다.")
                return

            self.joint_target_degrees[joint_name] = deg

            if not self.joint_publish_flags[joint_name]:
                self.joint_publish_flags[joint_name] = True
                thread = threading.Thread(
                    target=self.publisher_thread, args=(joint_name,), daemon=True)
                self.joint_threads[joint_name] = thread
                thread.start()

        except ValueError:
            print(f"[Error] {joint_name}: 유효한 숫자를 입력하세요")


if __name__ == "__main__":
    app = QApplication(sys.argv)

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(50, 50, 50))
    palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
    palette.setColor(QPalette.Button, QColor(70, 70, 70))
    palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
    palette.setColor(QPalette.Base, QColor(40, 40, 40))
    palette.setColor(QPalette.Text, QColor(230, 230, 230))
    app.setPalette(palette)

    rclpy.init()
    window = MobileLiftUi()
    window.setWindowTitle("Construction Mobile Lift Control Panel")
    window.show()
    app.exec()
    rclpy.shutdown()
