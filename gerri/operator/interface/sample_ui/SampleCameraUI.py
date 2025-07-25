from PySide6.QtWidgets import QVBoxLayout,QHBoxLayout,QGridLayout,QLabel,QLineEdit,QWidget, QApplication
from PySide6.QtGui import QPixmap, QImage
from PySide6.QtCore import QTimer
import cv2
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

def convert_cv_to_qt(cv_img):
    """OpenCV BGR 이미지를 QImage로 변환"""
    # cv_img = cv2.cvtColor(cv_img,cv2.COLOR_BAYER_BG2RGB)
    height, width, channel = cv_img.shape
    bytes_per_line = 3 * width
    # BGR -> RGB 변환
    rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    return QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

class CameraView(QLabel):
    def __init__(self, widget_w, widget_h, receiever, process_func=None):
        super().__init__()
        self.setFixedSize(widget_w, widget_h)
        self.process_func = process_func

        self.receiever = receiever

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)

    def update_frame(self):
        frame = self.receiever.get_frame()
        if frame is not None:
            qt_img = convert_cv_to_qt(frame)
            pixmap = QPixmap.fromImage(qt_img)
            if self.process_func:
                pixmap = self.process_func(pixmap)
            self.setPixmap(pixmap)
            self.setScaledContents(True)