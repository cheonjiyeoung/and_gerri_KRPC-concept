from PySide6.QtWidgets import QVBoxLayout,QHBoxLayout,QGridLayout,QLabel,QLineEdit,QWidget, QApplication, QPushButton
from PySide6.QtGui import QPixmap, QImage
import sys


class CameraDepthRealsenseView(QWidget):
    def __init__(self, widget_w, widget_h):
        super().__init__()
        
        self.setFixedSize(widget_w, widget_h)

        self.cam_view = QLabel(self) # front_camera
        self.cam_view.setGeometry(0,0,widget_w, widget_h)
        # self.cam_view.setScaledContents(True)
        
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(self.cam_view)
        main_layout.addStretch(0)
        # self.setLayout(main_layout)


# 테스트용 UI 실행
if __name__ == "__main__":
    app = QApplication(sys.argv)
    # window = StatusView(400, 400)
    # window.setWindowTitle("Spot Robot Controller")
    # window.show()
    sys.exit(app.exec())
