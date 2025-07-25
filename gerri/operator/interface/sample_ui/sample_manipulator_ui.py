from PySide6.QtWidgets import (
    QApplication, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout, QWidget, QLabel, QDoubleSpinBox, QLineEdit, QGroupBox
)
from PySide6.QtCore import Qt
import sys

class sample_manipulator_ui(QWidget):
    def __init__(self, widget_w, widget_h):
        super().__init__()
        self.setFixedSize(widget_w, widget_h)
        print(f"widget_w: {widget_w}, widget_h: {widget_h}")
        buttons_height = int(widget_h / 22)

        self.target_j1 = QDoubleSpinBox()
        self.target_j1.setFixedHeight(buttons_height)
        self.target_j1.setRange(-360.0, 360.0)
        self.target_j1.setValue(90.0)
        self.target_j1.setSingleStep(0.1)
        self.target_j1.setDecimals(1)

        self.target_j2 = QDoubleSpinBox()
        self.target_j2.setFixedHeight(buttons_height)
        self.target_j2.setRange(-360.0, 360.0)
        self.target_j2.setValue(0.0)
        self.target_j2.setSingleStep(0.1)
        self.target_j2.setDecimals(1)

        self.target_j3 = QDoubleSpinBox()
        self.target_j3.setFixedHeight(buttons_height)
        self.target_j3.setRange(-360.0, 360.0)
        self.target_j3.setValue(-90.0)
        self.target_j3.setSingleStep(0.1)
        self.target_j3.setDecimals(1)

        self.target_j4 = QDoubleSpinBox()
        self.target_j4.setFixedHeight(buttons_height)
        self.target_j4.setRange(-360.0, 360.0)
        self.target_j4.setValue(0.0)
        self.target_j4.setSingleStep(0.1)
        self.target_j4.setDecimals(1)

        self.target_j5 = QDoubleSpinBox()
        self.target_j5.setFixedHeight(buttons_height)
        self.target_j5.setRange(-360.0, 360.0)
        self.target_j5.setValue(45.0)
        self.target_j5.setSingleStep(0.1)
        self.target_j5.setDecimals(1)

        self.target_j6 = QDoubleSpinBox()
        self.target_j6.setFixedHeight(buttons_height)
        self.target_j6.setRange(-360.0, 360.0)
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

        ##### Left MoveJ Target Box#####    
        target_joint_1 = QLabel("j1")
        target_joint_2 = QLabel("j2")
        target_joint_3 = QLabel("j3")
        target_joint_4 = QLabel("j4")
        target_joint_5 = QLabel("j5")
        target_joint_6 = QLabel("j6")
        target_movej_vel = QLabel("vel")
        target_movej_acc = QLabel("acc")

        self.MoveJ.setFixedWidth(target_joint_1.width() + self.target_j1.width() + 20)

        joint_target_layout = QGridLayout(self)
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
        joint_target_layout.setSpacing(0)

        joint_target_box = QGroupBox("Left MoveJ")
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
        basic_command_widget = QWidget()
        layout_basic_command_widget = QHBoxLayout(basic_command_widget)
        layout_basic_command_widget.addStretch(0)
        layout_basic_command_widget.addWidget(joint_target_box)
        layout_basic_command_widget.addStretch(0)

        main_layout = QHBoxLayout(self)
        main_layout.addWidget(basic_command_widget)
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
    window = sample_manipulator_ui(925, 408)
    window.setWindowTitle("Construction Robot Controller")
    window.show()
    sys.exit(app.exec())
