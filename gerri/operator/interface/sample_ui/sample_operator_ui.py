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

from gerri.operator.interface.sample_ui.SampleCameraUI import SampleCameraUI
from gerri.operator.interface.sample_ui.sample_manipulator_ui import sample_manipulator_ui
from gerri.operator.interface.sample_ui.mobile_sample_ui import MobileOperationSampleUi
from _and_.keti_rtc.operator.webrtc_operator_media_receiver import OperatorMediaReceiever
from gerri.operator.examples.sample_operator.hello_universe_config import ROBOT_INFO,OPERATOR_INFO


class VideoViewer(QLabel):
    def __init__(self,w,h,reciever):
        super().__init__()
        self.setFixedSize(w,h)
        self.reciever = reciever

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(10)

    def update_frame(self):
        frame = self.reciever.get_frame()
        if frame is not None:
            qt_img = convert_cv_to_qt(frame)
            pixmap = QPixmap.fromImage(qt_img)
            self.setPixmap(pixmap)
            self.setScaledContents(True)
    # self.setCentralWidget(total_container)
def convert_cv_to_qt(cv_img):
    """OpenCV BGR 이미지를 QImage로 변환"""
    # cv_img = cv2.cvtColor(cv_img,cv2.COLOR_BAYER_BG2RGB)
    height, width, channel = cv_img.shape
    bytes_per_line = 3 * width
    # BGR -> RGB 변환
    rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    return QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

class SampleOperatorUI(QWidget):
    def __init__(self, widget_w=1280, widget_h=720):
        super().__init__()
        self.setFixedSize(widget_w, widget_h)

        self.receiver = OperatorMediaReceiever(robot_info=ROBOT_INFO,operator_info=OPERATOR_INFO,channel_info="front_cam",kind="video")
        self.receiver.connect()


        camera_view_width = widget_w * 0.7
        camera_view_height = widget_h
        self.video_viewer = VideoViewer(w=camera_view_width,h=camera_view_height,reciever=self.receiver)

        controller_width = widget_w * 0.3
        controller_height = camera_view_height

        self.sample_camera_ui = SampleCameraUI(widget_w=camera_view_width, widget_h=camera_view_height)
        self.sample_manipulator_ui =sample_manipulator_ui(widget_w=controller_width, widget_h=controller_height)
        self.sample_mobile_ui = MobileOperationSampleUi()

        rgb_camera_container = QWidget()
        layout_camera_rgb_full = QHBoxLayout(rgb_camera_container)
        layout_camera_rgb_full.addWidget(self.video_viewer)

        command_tab = QTabWidget()
        command_tab.addTab(self.sample_manipulator_ui, 'manipulator')
        command_tab.addTab(self.sample_mobile_ui, 'mobile')

        right_container = QWidget()
        layout_right_full = QHBoxLayout(right_container)
        layout_right_full.addWidget(command_tab)

        total_container = QWidget()
        layout_full = QHBoxLayout(self)
        layout_full.addWidget(rgb_camera_container)
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