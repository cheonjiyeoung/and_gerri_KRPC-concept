from PySide6.QtWidgets import QPushButton, QGridLayout,QWidget,QLabel,QApplication
from pubsub import pub
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.interface.utils.QPushButtonWithPub import QPushButtonWithPub

class MobileOperationSampleUi(QWidget):
    def __init__(self):
        super().__init__()
        btn_w = QPushButtonWithPub("W")
        btn_a = QPushButtonWithPub("A")
        btn_s = QPushButtonWithPub("S")
        btn_d = QPushButtonWithPub("D")
        btn_q = QPushButtonWithPub("Q")
        btn_e = QPushButtonWithPub("E")

        layout_main = QGridLayout(self)
        layout_main.addWidget(QLabel("[Mobile Controll]"),0,0,1,3)
        layout_main.addWidget(btn_q,1,0)
        layout_main.addWidget(btn_w,1,1)
        layout_main.addWidget(btn_e,1,2)
        layout_main.addWidget(btn_a,2,0)
        layout_main.addWidget(btn_s,2,1)
        layout_main.addWidget(btn_d,2,2)


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = MobileOperationSampleUi()
    window.show()
    sys.exit(app.exec())

