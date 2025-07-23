from PySide6.QtWidgets import (
    QApplication, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout, QWidget, QLabel, QDoubleSpinBox, QLineEdit, QGroupBox
)
from PySide6.QtCore import Qt
import sys

class RobotControllerRight(QWidget):
    def __init__(self, widget_w, widget_h):
        super().__init__()
        self.setFixedSize(widget_w, widget_h)
        print(f"widget_w: {widget_w}, widget_h: {widget_h}")
        buttons_height = int(widget_h / 22)

        ##### Status #####

        self.robot_state_edit = QLineEdit("UNKNON", readOnly=True)
        self.j1_edit = QLineEdit("",readOnly=True)
        self.j2_edit = QLineEdit("", readOnly=True)
        self.j3_edit = QLineEdit("", readOnly=True)
        self.j4_edit = QLineEdit("", readOnly=True)
        self.j5_edit = QLineEdit("", readOnly=True)
        self.j6_edit = QLineEdit("", readOnly=True)

        self.x_edit = QLineEdit("", readOnly=True)
        self.y_edit = QLineEdit("", readOnly=True)
        self.z_edit = QLineEdit("", readOnly=True)
        self.rx_edit = QLineEdit("", readOnly=True)
        self.ry_edit = QLineEdit("", readOnly=True)
        self.rz_edit = QLineEdit("", readOnly=True)

        self.master_onoff_edit = QLineEdit("Off", readOnly=True)
        # 버튼 선언
        self.Connect = QPushButton("Connect")
        self.Disconnect = QPushButton("Disconnect")

        ############################ Stop ###########################

        self.Stop = QPushButton("Stop")
        self.Stop.setFixedWidth(250)
        
        ############################ Joint Control ############################

        self.getCurrentJoint = QPushButton("Current Joint")
        self.target_j1 = QDoubleSpinBox()
        self.target_j1.setFixedHeight(buttons_height)
        self.target_j1.setRange(-180.0, 180.0)
        self.target_j1.setValue(-90.0)
        self.target_j1.setSingleStep(0.1)
        self.target_j1.setDecimals(1)

        self.target_j2 = QDoubleSpinBox()
        self.target_j2.setFixedHeight(buttons_height)
        self.target_j2.setRange(-180.0, 180.0)
        self.target_j2.setValue(0.0)
        self.target_j2.setSingleStep(0.1)
        self.target_j2.setDecimals(1)
        
        self.target_j3 = QDoubleSpinBox()
        self.target_j3.setFixedHeight(buttons_height)
        self.target_j3.setRange(-180.0, 180.0)
        self.target_j3.setValue(90.0)
        self.target_j3.setSingleStep(0.1)
        self.target_j3.setDecimals(1)
        
        self.target_j4 = QDoubleSpinBox()
        self.target_j4.setFixedHeight(buttons_height)
        self.target_j4.setRange(-180.0, 180.0)
        self.target_j4.setValue(0.0)
        self.target_j4.setSingleStep(0.1)
        self.target_j4.setDecimals(1)

        self.target_j5 = QDoubleSpinBox()
        self.target_j5.setFixedHeight(buttons_height)
        self.target_j5.setRange(-180.0, 180.0)
        self.target_j5.setValue(-45.0)
        self.target_j5.setSingleStep(0.1)
        self.target_j5.setDecimals(1)

        self.target_j6 = QDoubleSpinBox()
        self.target_j6.setFixedHeight(buttons_height)
        self.target_j6.setRange(-180.0, 180.0)
        self.target_j6.setValue(0.0)
        self.target_j6.setSingleStep(0.1)
        self.target_j6.setDecimals(1)

        self.movejVel = QDoubleSpinBox()
        self.movejVel.setFixedHeight(buttons_height)
        self.movejVel.setRange(0.0, 200.0)
        self.movejVel.setValue(30.0)
        self.movejVel.setSingleStep(1)
        self.movejVel.setDecimals(1)

        self.movejAcc = QDoubleSpinBox()
        self.movejAcc.setFixedHeight(buttons_height)
        self.movejAcc.setRange(0.0, 200.0)
        self.movejAcc.setValue(30.0)
        self.movejAcc.setSingleStep(1)
        self.movejAcc.setDecimals(1)

        self.MoveJ = QPushButton("MoveJ")

        ############################ Pose Control ############################

        self.getCurrentPose = QPushButton("Current Pose")
        self.target_x = QDoubleSpinBox()
        self.target_x.setFixedHeight(buttons_height)
        self.target_x.setValue(0.0)
        self.target_x.setRange(-1509.0, 1509.0)
        self.target_x.setSingleStep(1)
        self.target_x.setDecimals(1)

        self.target_y = QDoubleSpinBox()
        self.target_y.setFixedHeight(buttons_height)
        self.target_y.setValue(0.0)
        self.target_y.setRange(-1509.0, 1509.0)
        self.target_y.setSingleStep(1)
        self.target_y.setDecimals(1)
        
        self.target_z = QDoubleSpinBox()
        self.target_z.setFixedHeight(buttons_height)
        self.target_z.setValue(0.0)
        self.target_z.setRange(-1509.0, 1509.0)
        self.target_z.setSingleStep(1)
        self.target_z.setDecimals(1)
        
        self.target_rx = QDoubleSpinBox()
        self.target_rx.setFixedHeight(buttons_height)
        self.target_rx.setValue(0.0)
        self.target_rx.setRange(-180.0, 180.0)
        self.target_rx.setSingleStep(0.1)
        self.target_rx.setDecimals(1)

        self.target_ry = QDoubleSpinBox()
        self.target_ry.setFixedHeight(buttons_height)
        self.target_ry.setValue(0.0)
        self.target_ry.setRange(-180.0, 180.0)
        self.target_ry.setSingleStep(0.1)
        self.target_ry.setDecimals(1)

        self.target_rz = QDoubleSpinBox()
        self.target_rz.setFixedHeight(buttons_height)
        self.target_rz.setValue(0.0)
        self.target_rz.setRange(-180.0, 180.0)
        self.target_rz.setSingleStep(0.1)
        self.target_rz.setDecimals(1)

        self.movelVel = QDoubleSpinBox()
        self.movelVel.setFixedHeight(buttons_height)
        self.movelVel.setValue(0.0)
        self.movelVel.setRange(0.0, 200.0)
        self.movelVel.setSingleStep(1)
        self.movelVel.setDecimals(1)

        self.movelAcc = QDoubleSpinBox()
        self.movelAcc.setFixedHeight(buttons_height)
        self.movelAcc.setValue(0.0)
        self.movelAcc.setRange(0.0, 200.0)
        self.movelAcc.setSingleStep(1)
        self.movelAcc.setDecimals(1)

        self.MoveL = QPushButton("MoveL")

        ############################ Master Control ############################

        self.master_start = QPushButton("Start")
        self.master_stop = QPushButton("Stop")
        self.master_go_back = QPushButton("Back")

        ##### right WebRTC State Box#####
        state_label = QLabel("State")
        state_label.adjustSize()
        self.robot_state_edit.setFixedWidth(90)

        self.j1_edit.setFixedWidth(60)
        self.j2_edit.setFixedWidth(60)
        self.j3_edit.setFixedWidth(60)
        self.j4_edit.setFixedWidth(60)
        self.j5_edit.setFixedWidth(60)
        self.j6_edit.setFixedWidth(60)
        joint1_label = QLabel("j1 (deg)")
        joint2_label = QLabel("j2 (deg)")
        joint3_label = QLabel("j3 (deg)")
        joint4_label = QLabel("j4 (deg)")
        joint5_label = QLabel("j5 (deg)")
        joint6_label = QLabel("j6 (deg)")
        joint1_label.setFixedWidth(50)
        joint2_label.setFixedWidth(50)
        joint3_label.setFixedWidth(50)
        joint4_label.setFixedWidth(50)
        joint5_label.setFixedWidth(50)
        joint6_label.setFixedWidth(50)

        self.x_edit.setFixedWidth(60)  
        self.y_edit.setFixedWidth(60)
        self.z_edit.setFixedWidth(60)
        self.rx_edit.setFixedWidth(60)
        self.ry_edit.setFixedWidth(60)
        self.rz_edit.setFixedWidth(60)

        x_label = QLabel("x (mm)")
        y_label = QLabel("y (mm)")
        z_label = QLabel("z (mm)")
        rx_label = QLabel("rx (deg)")
        ry_label = QLabel("ry (deg)")
        rz_label = QLabel("rz (deg)")
        x_label.setFixedWidth(50)
        y_label.setFixedWidth(50)
        z_label.setFixedWidth(50)
        rx_label.setFixedWidth(50)
        ry_label.setFixedWidth(50)
        rz_label.setFixedWidth(50)
        
        state_layout = QGridLayout(self)
        state_layout.addWidget(state_label, 0, 0, 1, 2, alignment=Qt.AlignCenter)
        state_layout.addWidget(self.robot_state_edit, 0, 2, 1, 2)
        state_layout.addWidget(joint1_label, 1, 0)
        state_layout.addWidget(self.j1_edit, 1, 1)
        state_layout.addWidget(x_label, 1, 2)
        state_layout.addWidget(self.x_edit, 1, 3)
        state_layout.addWidget(joint2_label, 2, 0)
        state_layout.addWidget(self.j2_edit, 2, 1)
        state_layout.addWidget(y_label, 2, 2)
        state_layout.addWidget(self.y_edit, 2, 3)
        state_layout.addWidget(joint3_label, 3, 0)
        state_layout.addWidget(self.j3_edit, 3, 1)
        state_layout.addWidget(z_label, 3, 2)
        state_layout.addWidget(self.z_edit, 3, 3)
        state_layout.addWidget(joint4_label, 4, 0)
        state_layout.addWidget(self.j4_edit, 4, 1)
        state_layout.addWidget(rx_label, 4, 2)
        state_layout.addWidget(self.rx_edit, 4, 3)
        state_layout.addWidget(joint5_label, 5, 0)
        state_layout.addWidget(self.j5_edit, 5, 1)
        state_layout.addWidget(ry_label, 5, 2)
        state_layout.addWidget(self.ry_edit, 5, 3)
        state_layout.addWidget(joint6_label, 6, 0)
        state_layout.addWidget(self.j6_edit, 6, 1)
        state_layout.addWidget(rz_label, 6, 2)
        state_layout.addWidget(self.rz_edit, 6, 3)
        state_layout.setContentsMargins(0, 0, 0, 0)
        state_layout.setSpacing(10)

        current_state_box = QGroupBox("Current State")
        current_state_box.setLayout(state_layout)
        current_state_box.setStyleSheet("""
                                QGroupBox {
                                    margin-left: 0px;
                                    margin-right: 0px;
                                    padding-top: 0px;
                                    padding-bottom: 0px;
                                    padding-left: 10px;
                                    padding-right: 10px;
                                }
                            """)
        ##### Master Control Box#####
        master_onoff_label = QLabel("On/Off")
        master_onoff_label.setFixedWidth(80)
        self.master_onoff_edit.setFixedWidth(30)
        self.master_start.setFixedWidth(master_onoff_label.width() + self.master_onoff_edit.width() + 10)
        self.master_stop.setFixedWidth(master_onoff_label.width() + self.master_onoff_edit.width() + 10)
        self.master_go_back.setFixedWidth(master_onoff_label.width() + self.master_onoff_edit.width() + 10)
        master_layout = QGridLayout(self)
        master_layout.addWidget(master_onoff_label, 0, 0)
        master_layout.addWidget(self.master_onoff_edit, 0, 1)
        master_layout.addWidget(self.master_start, 1, 0 , 1, 2)
        master_layout.addWidget(self.master_stop, 2, 0 , 1, 2)
        master_layout.addWidget(self.master_go_back, 3, 0 , 1, 2)
        master_layout.setContentsMargins(0, 0, 0, 0)
        master_layout.setSpacing(10)


        master_box = QGroupBox("Master Control")
        master_box.setLayout(master_layout)
        master_box.setStyleSheet("""
                        QGroupBox {
                            margin-left: 0px;
                            margin-right: 0px;
                            padding-top: 10px;
                            padding-bottom: 10px;
                            padding-left: 5px;
                            padding-right: 5px;
                        }
                    """)


        ##### right MoveJ Target Box#####    
        target_joint_1 = QLabel("j1")
        target_joint_2 = QLabel("j2")
        target_joint_3 = QLabel("j3")
        target_joint_4 = QLabel("j4")
        target_joint_5 = QLabel("j5")
        target_joint_6 = QLabel("j6")
        target_movej_vel = QLabel("vel")
        target_movej_acc = QLabel("acc")

        target_joint_1.setFixedWidth(10)
        target_joint_2.setFixedWidth(10)
        target_joint_3.setFixedWidth(10)
        target_joint_4.setFixedWidth(10)
        target_joint_5.setFixedWidth(10)
        target_joint_6.setFixedWidth(10)
        target_movej_vel.setFixedWidth(20)
        target_movej_acc.setFixedWidth(20)

        self.target_j1.setFixedWidth(60)
        self.target_j2.setFixedWidth(60)
        self.target_j3.setFixedWidth(60)
        self.target_j4.setFixedWidth(60)
        self.target_j5.setFixedWidth(60)
        self.target_j6.setFixedWidth(60)
        self.movejVel.setFixedWidth(60)
        self.movejAcc.setFixedWidth(60)

        self.MoveJ.setFixedWidth(target_joint_1.width() + self.target_j1.width() + 20)
        self.getCurrentJoint.setFixedWidth(target_joint_1.width() + self.target_j1.width() + 20)
        
        joint_target_layout = QGridLayout(self)
        joint_target_layout.addWidget(self.getCurrentJoint, 0, 0, 1, 2)
        joint_target_layout.addWidget(target_joint_1, 1, 0)
        joint_target_layout.addWidget(self.target_j1, 1, 1)
        joint_target_layout.addWidget(target_joint_2, 2, 0)
        joint_target_layout.addWidget(self.target_j2, 2, 1)
        joint_target_layout.addWidget(target_joint_3, 3, 0)
        joint_target_layout.addWidget(self.target_j3, 3, 1)
        joint_target_layout.addWidget(target_joint_4, 4, 0)
        joint_target_layout.addWidget(self.target_j4, 4, 1)
        joint_target_layout.addWidget(target_joint_5, 5, 0)
        joint_target_layout.addWidget(self.target_j5, 5, 1)
        joint_target_layout.addWidget(target_joint_6, 6, 0)
        joint_target_layout.addWidget(self.target_j6, 6, 1)
        joint_target_layout.addWidget(target_movej_vel, 7, 0)
        joint_target_layout.addWidget(self.movejVel, 7, 1)
        joint_target_layout.addWidget(target_movej_acc, 8, 0)
        joint_target_layout.addWidget(self.movejAcc, 8, 1)
        joint_target_layout.addWidget(self.MoveJ, 9, 0, 1, 2)
        joint_target_layout.setContentsMargins(0, 0, 0, 0)
        joint_target_layout.setSpacing(10)

        joint_target_box = QGroupBox("Right MoveJ")
        joint_target_box.setLayout(joint_target_layout)
        joint_target_box.setStyleSheet("""
                        QGroupBox {
                            margin-left: 0px;
                            margin-right: 0px;
                            padding-top: 10px;
                            padding-bottom: 0px;
                            padding-left: 10px;
                            padding-right: 10px;
                        }
                    """)
        ##### right Pose Target Box#####
        target_x_label = QLabel("x")
        target_y_label = QLabel("y")
        target_z_label = QLabel("z")
        target_rx_label = QLabel("rx")
        target_ry_label = QLabel("ry")
        target_rz_label = QLabel("rz")
        target_moveVel_label = QLabel("vel")
        target_moveAcc_label = QLabel("acc")

        target_x_label.setFixedWidth(10)
        target_y_label.setFixedWidth(10)
        target_z_label.setFixedWidth(10)
        target_rx_label.setFixedWidth(20)
        target_ry_label.setFixedWidth(20)
        target_rz_label.setFixedWidth(20)
        target_moveVel_label.setFixedWidth(20)
        target_moveAcc_label.setFixedWidth(20)

        self.target_x.setFixedWidth(60)
        self.target_y.setFixedWidth(60)
        self.target_z.setFixedWidth(60)
        self.target_rx.setFixedWidth(60)
        self.target_ry.setFixedWidth(60)
        self.target_rz.setFixedWidth(60)
        self.movelVel.setFixedWidth(60)
        self.movelAcc.setFixedWidth(60)

        self.MoveL.setFixedWidth(target_x_label.width() + self.target_x.width() + 20)
        self.getCurrentPose.setFixedWidth(target_x_label.width() + self.target_x.width() + 20)
  
        pose_target_layout = QGridLayout(self)
        pose_target_layout.addWidget(self.getCurrentPose, 0, 0, 1, 2)
        pose_target_layout.addWidget(target_x_label, 1, 0)
        pose_target_layout.addWidget(self.target_x, 1, 1)
        pose_target_layout.addWidget(target_y_label, 2, 0)
        pose_target_layout.addWidget(self.target_y, 2, 1)
        pose_target_layout.addWidget(target_z_label, 3, 0)
        pose_target_layout.addWidget(self.target_z, 3, 1)
        pose_target_layout.addWidget(target_rx_label, 4, 0)
        pose_target_layout.addWidget(self.target_rx, 4, 1)
        pose_target_layout.addWidget(target_ry_label, 5, 0)
        pose_target_layout.addWidget(self.target_ry, 5, 1)
        pose_target_layout.addWidget(target_rz_label, 6, 0)
        pose_target_layout.addWidget(self.target_rz, 6, 1)
        pose_target_layout.addWidget(target_moveVel_label, 7, 0)
        pose_target_layout.addWidget(self.movelVel, 7, 1)
        pose_target_layout.addWidget(target_moveAcc_label, 8, 0)
        pose_target_layout.addWidget(self.movelAcc, 8, 1)
        pose_target_layout.addWidget(self.MoveL, 9, 0, 1, 2)
        pose_target_layout.setContentsMargins(0, 0, 0, 0)
        pose_target_layout.setSpacing(10)

        pose_target_box = QGroupBox("Right MoveL")
        pose_target_box.setLayout(pose_target_layout)
        pose_target_box.setStyleSheet("""
                        QGroupBox {
                            margin-left: 0px;
                            margin-right: 0px;
                            padding-top: 10px;
                            padding-bottom: 0px;
                            padding-left: 10px;
                            padding-right: 10px;
                        }
                    """)


        other_command_box = QGroupBox("Other Command")
        # other_command_box.setLayout(other_command_layout)
        other_command_box.setStyleSheet("""
                        QGroupBox {
                            margin-left: 0px;
                            margin-right: 0px;
                            padding-top: 0px;
                            padding-bottom: 0px;
                            padding-left: 00px;
                            padding-right: 0px;
                        }
                    """)

        ###  Making Widgets ###

        status_full = QWidget()
        layout_status_full = QVBoxLayout(status_full)
        layout_status_full.addWidget(current_state_box)
        layout_status_full.setContentsMargins(0, 0, 0, 0)
        layout_status_full.setSpacing(0)

        basic_command_widget = QWidget()
        layout_basic_command_widget = QHBoxLayout(basic_command_widget)
        layout_basic_command_widget.addWidget(joint_target_box)
        layout_basic_command_widget.addWidget(pose_target_box)

        command_total_widget = QWidget()
        layout_command_total = QVBoxLayout(command_total_widget)
        layout_command_total.addWidget(basic_command_widget)
        layout_command_total.setContentsMargins(0, 0, 0, 0)
        layout_command_total.setSpacing(0)
        layout_command_total.addWidget(self.Stop)

        other_command_widget = QWidget()
        layout_other_command = QVBoxLayout(other_command_widget)
        layout_other_command.addWidget(master_box)
        layout_other_command.addWidget(other_command_box)

        main_layout = QHBoxLayout(self)
        main_layout.addWidget(status_full)
        main_layout.addWidget(command_total_widget)
        main_layout.addWidget(other_command_widget)
        main_layout.addStretch(0)

# 테스트용 UI 실행
if __name__ == "__main__":
    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    full_size = screen.geometry()
    print(f"전체 화면 크기: {full_size.width()} x {full_size.height()}")

    # 사용 가능한 영역 (작업 표시줄 등 제외)
    available_size = screen.availableGeometry()
    print(f"사용 가능한 크기: {available_size.width()} x {available_size.height()}")
    widget_w = available_size.width() - 70
    widget_h = available_size.height() - 60
    window = RobotControllerRight(925, 408)
    window.setWindowTitle("Construction Robot Controller")
    window.show()
    sys.exit(app.exec())
