from PySide6.QtWidgets import QPushButton, QGridLayout,QWidget,QLabel,QApplication
from pubsub import pub

class MobileOperationSampleUi(QWidget):
    def __init__(self):
        super().__init__()
        btn_w = QPushButton("W")
        btn_w.clicked.connect(lambda:pub.sendMessage("ui",signel="clicked_W"))
        btn_a = QPushButton("A")
        btn_a.clicked.connect(lambda:pub.sendMessage("ui",signel="clicked_A"))
        btn_s = QPushButton("S")
        btn_s.clicked.connect(lambda:pub.sendMessage("ui",signel="clicked_S"))
        btn_d = QPushButton("D")
        btn_d.clicked.connect(lambda:pub.sendMessage("ui",signel="clicked_D"))
        btn_q = QPushButton("Q")
        btn_q.clicked.connect(lambda:pub.sendMessage("ui",signel="clicked_Q"))
        btn_e = QPushButton("E")
        btn_e.clicked.connect(lambda:pub.sendMessage("ui",signel="clicked_E"))

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

