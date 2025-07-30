from PySide6.QtWidgets import QLabel
from PySide6.QtCore import QThread, QTimer
from PySide6.QtGui import QImage, QPixmap
import cv2
import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.examples.spot.spot_config import OPERATOR_INFO,ROBOT_INFO

def convert_cv_to_qt(cv_img):
    """OpenCV BGR 이미지를 QImage로 변환"""
    # cv_img = cv2.cvtColor(cv_img,cv2.COLOR_BAYER_BG2RGB)
    height, width, channel = cv_img.shape
    bytes_per_line = 3 * width
    # BGR -> RGB 변환
    rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    return QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

class VideoViewer(QLabel):
    def __init__(self,w,h,reciever):
        super().__init__()
        self.w = w
        self.h = h
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