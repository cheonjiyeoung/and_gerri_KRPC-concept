from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QApplication, QLabel, QGroupBox, QTabWidget
from PySide6.QtWidgets import QFrame
from PySide6.QtGui import QPalette, QColor
from PySide6.QtWidgets import QWidget
from PySide6.QtWidgets import QLabel
from PySide6.QtCore import QThread, QTimer
from PySide6.QtGui import QImage, QPixmap
import cv2
import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.interface.sample_ui.SampleCameraUI import CameraView
from gerri.operator.interface.sample_ui.sample_manipulator_ui import sample_manipulator_ui
from gerri.operator.interface.sample_ui.mobile_sample_ui import MobileOperationSampleUi

class SampleOperatorUI(QWidget):
    def __init__(self, widget_w=1280, widget_h=720, recievers = None):
        super().__init__()
        self.setFixedSize(widget_w, widget_h)
        camera_view_width = widget_w * 0.7
        camera_view_height = widget_h

        controller_width = widget_w * 0.3
        controller_height = camera_view_height

        # if recievers is not None:
        self.front_cam_receiver = recievers[0]
        self.front_cam = CameraView(widget_w=camera_view_width, widget_h=camera_view_height,receiever=self.front_cam_receiver,process_func=None)

        self.sample_manipulator_ui =sample_manipulator_ui(widget_w=controller_width, widget_h=controller_height)
        self.sample_mobile_ui = MobileOperationSampleUi()

        front_container_camera_view = QWidget()
        layout_camera_view = QVBoxLayout(front_container_camera_view)
        layout_camera_view.addWidget(self.front_cam)

        command_tab = QTabWidget()
        command_tab.addTab(self.sample_manipulator_ui, 'manipulator')
        command_tab.addTab(self.sample_mobile_ui, 'mobile')

        right_container = QWidget()
        layout_right_full = QHBoxLayout(right_container)
        layout_right_full.addWidget(command_tab)

        total_container = QWidget()
        layout_full = QHBoxLayout(self)
        layout_full.addWidget(front_container_camera_view)
        layout_full.addWidget(right_container)




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
    window = SampleOperatorUI(widget_w, widget_h)
    window.setWindowTitle("Sample Operator UI")
    window.show()
    sys.exit(app.exec())