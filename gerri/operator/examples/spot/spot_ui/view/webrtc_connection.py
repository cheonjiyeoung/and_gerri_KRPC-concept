from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QLineEdit, QPushButton, QGridLayout
from PySide6.QtCore import QTimer
import os,sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.examples.spot.spot_ui.utils.my_widgets import PubSubButton, PubSubDBSpinbox, PubSubComboBox, PubSubLineEdit

class WebrtcConnectionViewer(QWidget):
    def __init__(self,channels:dict):
        super().__init__()

        self.layout_webrtc_connection = QGridLayout()
        idx = 0
        for key in channels.keys():
            current_channel = channels[key]
            
            self.layout_webrtc_connection.addWidget(QLabel(f"{key}_channel"),idx,0,1,2)
            idx += 1

            # self.layout_webrtc_connection.addWidget(QLabel("connection :"),idx,0)
            # edit_connection = QLineEdit("Disconnected",readOnly=True)
            # timer = QTimer(self)
            # timer.timeout.connect(lambda ch=current_channel, le=edit_connection: self.update(line_edit=le,
            #                                                                 channel=ch))
            # timer.start(1000)    
            # self.layout_webrtc_connection.addWidget(edit_connection,idx,1)
            # idx += 1
            
            btn_connect = PubSubButton(f"{key}_connect")
            btn_disconnect = PubSubButton(f"{key}_disconnect")

            self.layout_webrtc_connection.addWidget(btn_connect,idx,0)
            self.layout_webrtc_connection.addWidget(btn_disconnect,idx,1)
            idx += 1

            self.layout_webrtc_connection.addWidget(QLabel(" "),idx,0)
            idx += 1

        layout_webrtc_connection = QVBoxLayout(self)
        layout_webrtc_connection.addWidget(QLabel("[ WebRTC Connection ]"))
        layout_webrtc_connection.addLayout(self.layout_webrtc_connection)
        layout_webrtc_connection.addStretch(0)


    def update(self,line_edit:QLineEdit,channel):
        if channel.dead_sign:
            line_edit.setText("Disconnected")
        else:
            line_edit.setText("Connected")

import time
import threading
class test_channel:
    def __init__(self,channel):
        self.channel = channel
        self.dead_sign = False
        threading.Thread(target=self.th,daemon=True).start()
    def connect(self):
        print(f"{self.channel}_connect")
    def disconnect(self):
        print(f"{self.channel}_disconnect")

    def th(self):
        while True:
            self.dead_sign = not self.dead_sign
            print(f"{self.channel}_channel dead : {self.dead_sign}")
            time.sleep(1)



if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication
    ch_1 = test_channel("1")  
    ch_2 = test_channel("2")  
    ch_3 = test_channel("3")  
    ch_4 = test_channel("4")  
    webrtc_channels = {"command":ch_1,
                    "ptz_cam":ch_2,
                    "rest_cam":ch_3,
                    "audio":ch_4}

    app = QApplication(sys.argv)
    window = WebrtcConnectionViewer(channels=webrtc_channels)  # 원하는 크기 지정
    window.show()
    sys.exit(app.exec())