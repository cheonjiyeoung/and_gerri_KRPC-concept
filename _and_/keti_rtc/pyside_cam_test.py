import cv2
from PySide6.QtWidgets import QApplication, QLabel, QMainWindow
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QImage, QPixmap


import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.keti_rtc.stereo_camera_combiner import StereoCameraCombiner
from _and_.keti_rtc.stereo_vr_processor import StereoVRProcessor, StereoVROption
from _and_.keti_rtc.cam_manager import CameraManager


def convert_frame_to_pixmap(frame):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    bytes_per_line = ch * w
    return QPixmap.fromImage(QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888))


class StereoVRGuiTest(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("StereoVRProcessor GUI Test")

        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setScaledContents(True)  # 프레임이 윈도우 크기에 맞게 자동 리사이징

        # 카메라 설정
        # self.cam1 = CameraManager("left", camera_index=2, width=1920, height=1080, fps=30)
        # self.cam2 = CameraManager("right", camera_index=0, width=1920, height=1080, fps=30)
        # self.cam1.start()
        # self.cam2.start()

        self.cam0 = CameraManager(camera_index=0, width=9999, height=9999, fps=30)


        # self.combiner = StereoCameraCombiner(self.cam1, self.cam2)
        self.vr_option = StereoVROption(
            interocular_adjust=0,
            zoom_factor=1.0,
            aspect_mode="resize",
            target_aspect_ratio="2:1"
        )
        self.processor = StereoVRProcessor(self.cam0, self.vr_option)

        # self.processor = self.cam0

        # 타이머 프레임 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        # 전체화면 모드로 진입
        self.showFullScreen()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Left:
            self.vr_option.interocular_adjust -= 10
        elif key == Qt.Key_Right:
            self.vr_option.interocular_adjust += 10
        elif key == Qt.Key_R:
            self.vr_option.interocular_adjust = 0
        elif key == Qt.Key_Escape:
            QApplication.quit()
        print(f"[Adjust] interocular_adjust = {self.vr_option.interocular_adjust}")

    def resizeEvent(self, event):
        self.label.resize(self.size())  # 윈도우 크기에 따라 라벨도 자동 조정됨

    def update_frame(self):
        frame = self.processor.get_frame()
        if frame is not None:
            self.label.setPixmap(convert_frame_to_pixmap(frame))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = StereoVRGuiTest()
    window.show()
    sys.exit(app.exec())
