from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QApplication, QLabel, QGroupBox, QTabWidget
from PySide6.QtWidgets import QFrame
from PySide6.QtGui import QPalette, QColor
import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
# print(os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.interface.construction_ui.connection_ui import ConnectionUi
from gerri.operator.interface.construction_ui.controller_left import RobotControllerLeft
from gerri.operator.interface.construction_ui.controller_right import RobotControllerRight
from gerri.operator.interface.construction_ui.camera_rgb_realsense_view import CameraRGBRealsenseView
from gerri.operator.interface.construction_ui.camera_depth_realsense_view import CameraDepthRealsenseView
from gerri.operator.interface.construction_ui.log import RobotControllerLog
import threading,time

class ConstructionOperatorUI(QMainWindow):
    def __init__(self, widget_w, widget_h):
        super().__init__()
        self.setFixedSize(widget_w, widget_h)

        camera_view_width = widget_w*0.75
        # camera_view_width = widget_w 
        camera_view_height = int(widget_h * 0.6)

        connection_left_width = int(widget_w * 0.13)
        connection_left_height = int(widget_h * 0.4)

        controller_left_width = int((widget_w * 0.87)/2)-5
        controller_left_height = int(widget_h * 0.4)

        controller_right_width = int((widget_w * 0.87)/2)-5
        controller_right_height = int(widget_h * 0.4) 

        log_width = widget_w * 0.25
        log_height = camera_view_height

        self.webrtc_connection = ConnectionUi(widget_w=connection_left_width, widget_h=connection_left_height)
        self.robot_controller_left = RobotControllerLeft(widget_w=controller_left_width, widget_h=controller_left_height)
        self.robot_controller_right = RobotControllerRight(widget_w=controller_right_width, widget_h=controller_right_height)
        self.camera_rgb_realsense_view = CameraRGBRealsenseView(widget_w=camera_view_width, widget_h=camera_view_height)
        self.camera_depth_realsense_view = CameraDepthRealsenseView(widget_w=camera_view_width, widget_h=camera_view_height)
        self.log_message = RobotControllerLog(widget_w=log_width, widget_h=log_height)

        connection_webrtc_container = QWidget()
        layout_connection_full = QHBoxLayout(connection_webrtc_container)
        # layout_command_left_full.setContentsMargins(0, 0, 0, 0)  # ✅ 여백 제거  
        layout_connection_full.addWidget(self.webrtc_connection)

        connection_webrtc_box = QGroupBox("WebRTC Connection")
        connection_webrtc_box.setStyleSheet("""
                            QGroupBox {
                                border: 2px solid gray;
                                border-radius: 5px;
                                margin-top: 10px;
                                margin-bottom: 0px;
                                margin-left: 0px;
                                margin-right: 0px;
                                padding-top: 0px;
                                padding-bottom: 0px;
                                padding-left: 0px;
                                padding-right: 0px;
                            }
                            QGroupBox::title {
                                subcontrol-origin: margin;
                                left: 10px;
                                padding: 0 5px 0 5px;
                            }
                        """)
        connection_webrtc_box.setLayout(layout_connection_full)

        command_left_container = QWidget()
        layout_command_left_full = QHBoxLayout(command_left_container)
        # layout_command_left_full.setContentsMargins(0, 0, 0, 0)  # ✅ 여백 제거  
        layout_command_left_full.addWidget(self.robot_controller_left)

        command_left_box = QGroupBox("Left Arm")
        command_left_box.setStyleSheet("""
                            QGroupBox {
                                border: 2px solid gray;
                                border-radius: 5px;
                                margin-top: 10px;
                                margin-bottom: 0px;
                                margin-left: 0px;
                                margin-right: 0px;
                                padding-top: 0px;
                                padding-bottom: 0px;
                                padding-left: 0px;
                                padding-right: 0px;
                            }
                            QGroupBox::title {
                                subcontrol-origin: margin;
                                left: 10px;
                                padding: 0 5px 0 5px;
                            }
                        """)
        command_left_box.setLayout(layout_command_left_full)

        command_right_container = QWidget()
        layout_command_right_full = QHBoxLayout(command_right_container)
        # layout_command_right_full.setContentsMargins(0, 0, 0, 0)  # ✅ 여백 제거  
        layout_command_right_full.addWidget(self.robot_controller_right)

        command_right_box = QGroupBox("Right Arm")
        command_right_box.setStyleSheet("""
                    QGroupBox {
                                border: 2px solid gray;
                                border-radius: 5px;
                                margin-top: 10px;
                                margin-bottom: 0px;
                                margin-left: 0px;
                                margin-right: 0px;
                                padding-top: 0px;
                                padding-bottom: 0px;
                                padding-left: 0px;
                                padding-right: 0px;
                    }
                    QGroupBox::title {
                        subcontrol-origin: margin;
                        left: 10px;
                        padding: 0 5px 0 5px;
                    }
                """)
        command_right_box.setLayout(layout_command_right_full)

        Vline = QFrame()
        Vline.setFrameShape(QFrame.VLine)        # HLine: 수평선
        Vline.setFrameShadow(QFrame.Sunken)      # Sunken: 음각 효과 (옵션)
        Vline.setStyleSheet("margin: 5px 0;")     # 선 주변 여백 조정 (선택 사항)

        total_command_container = QWidget()
        layout_command_full = QHBoxLayout(total_command_container)
        layout_command_full.addWidget(connection_webrtc_box)
        layout_command_full.addWidget(command_left_box)
        layout_command_full.addWidget(command_right_box)
        
        rgb_camera_container = QWidget()
        layout_camera_rgb_full = QHBoxLayout(rgb_camera_container)
        layout_camera_rgb_full.addWidget(self.camera_rgb_realsense_view)
        layout_camera_rgb_full.setContentsMargins(0, 0, 0, 0)
        layout_camera_rgb_full.setSpacing(0)

        depth_camera_container = QWidget()
        layout_camera_depth_full = QHBoxLayout(depth_camera_container)
        layout_camera_depth_full.addWidget(self.camera_depth_realsense_view)
        layout_camera_depth_full.setContentsMargins(0, 0, 0, 0)
        layout_camera_depth_full.setSpacing(0)
        
        camera_tab = QTabWidget()
        camera_tab.addTab(rgb_camera_container, 'rgb_realsense')
        camera_tab.addTab(depth_camera_container, 'depth_realsense')

        log_container = QWidget()
        layout_log_full = QHBoxLayout(log_container)
        layout_log_full.addWidget(self.log_message)

        log_box = QGroupBox("Log")
        log_box.setStyleSheet("""
                    QGroupBox {
                                border: 2px solid gray;
                                border-radius: 5px;
                                margin-top: 10px;
                                margin-bottom: 0px;
                                margin-left: 0px;
                                margin-right: 0px;
                                padding-top: 0px;
                                padding-bottom: 0px;
                                padding-left: 0px;
                                padding-right: 0px;
                    }
                    QGroupBox::title {
                        subcontrol-origin: margin;
                        left: 10px;
                        padding: 0 5px 0 5px;
                    }
                """)
        log_box.setLayout(layout_log_full)


        bottom_container = QWidget()
        layout_bottom_full = QHBoxLayout(bottom_container)
        layout_bottom_full.addWidget(camera_tab)
        layout_bottom_full.addWidget(log_box)

        total_container = QWidget()
        layout_full = QVBoxLayout(total_container)
        layout_full.addWidget(total_command_container)
        layout_full.addStretch(0)
        layout_full.addWidget(bottom_container)
        layout_full.addStretch(0)

        self.setCentralWidget(total_container)

# 테스트용 UI 실행
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
    screen = app.primaryScreen()
    full_size = screen.geometry()
    print(f"전체 화면 크기: {full_size.width()} x {full_size.height()}")

    # 사용 가능한 영역 (작업 표시줄 등 제외)
    available_size = screen.availableGeometry()
    print(f"사용 가능한 크기: {available_size.width()} x {available_size.height()}")
    widget_w = available_size.width() -70
    widget_h = available_size.height() -60
    window = ConstructionOperatorUI(widget_w, widget_h)
    window.setWindowTitle("Construction Robot Controller")
    window.show()
    sys.exit(app.exec())