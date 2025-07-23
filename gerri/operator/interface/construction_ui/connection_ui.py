from PySide6.QtWidgets import QVBoxLayout,QHBoxLayout,QGridLayout,QLabel,QLineEdit,QWidget, QApplication, QPushButton, QGroupBox
from PySide6.QtGui import QPixmap, QImage
import sys


class ConnectionUi(QWidget):
    def __init__(self, widget_w, widget_h):
        super().__init__()
        
        self.setFixedSize(widget_w, widget_h)

        command_connection_label = QLabel("Connection")
        command_connection_label.adjustSize()
        self.Connect_Command = QPushButton("Command Connect")
        self.Disconnect_Command = QPushButton("Command Disconnect")
        self.webrtc_connection_command_edit = QLineEdit("Disconnected",readOnly=True)
        self.webrtc_connection_command_edit.setFixedWidth(90)

        camera_connection_label = QLabel("Connection")
        camera_connection_label.adjustSize()
        self.Connect_Camera = QPushButton("Camera Connect")
        self.Disconnect_Camera = QPushButton("Camera Disconnect")
        self.webrtc_connection_camera_edit = QLineEdit("Disconnected",readOnly=True)
        self.webrtc_connection_camera_edit.setFixedWidth(90)

        command_connection_layout = QGridLayout(self)
        command_connection_layout.addWidget(command_connection_label, 0 ,0)
        command_connection_layout.addWidget(self.webrtc_connection_command_edit, 0, 1)
        command_connection_layout.addWidget(self.Connect_Command, 1, 0, 1, 2)
        command_connection_layout.addWidget(self.Disconnect_Command, 2, 0, 1, 2)

        command_connection_box = QGroupBox("WebRTC Command")
        command_connection_box.setLayout(command_connection_layout)
        command_connection_box.setStyleSheet("""
                        QGroupBox {
                            margin-left: 0px;
                            margin-right: 0px;
                            padding-top: 10px;
                            padding-bottom: 0px;
                            padding-left: 10px;
                            padding-right: 10px;
                        }
                    """)

        command_connection_widget = QWidget()
        command_connection_widget_label = QVBoxLayout(command_connection_widget)
        command_connection_widget_label.addWidget(command_connection_box)

        camera_connection_layout = QGridLayout(self)
        camera_connection_layout.addWidget(camera_connection_label, 0 ,0)
        camera_connection_layout.addWidget(self.webrtc_connection_camera_edit, 0, 1)
        camera_connection_layout.addWidget(self.Connect_Camera, 1, 0, 1, 2)
        camera_connection_layout.addWidget(self.Disconnect_Camera, 2, 0, 1, 2)

        camera_connection_box = QGroupBox("WebRTC Camera")
        camera_connection_box.setLayout(camera_connection_layout)
        camera_connection_box.setStyleSheet("""
                        QGroupBox {
                            margin-left: 0px;
                            margin-right: 0px;
                            padding-top: 10px;
                            padding-bottom: 0px;
                            padding-left: 10px;
                            padding-right: 10px;
                        }
                    """)

        camera_connection_widget = QWidget()
        camera_connection_widget_label = QVBoxLayout(camera_connection_widget)
        camera_connection_widget_label.addWidget(camera_connection_box)
        
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(command_connection_widget)
        main_layout.addWidget(camera_connection_widget)
        main_layout.addStretch(0)
        # self.setLayout(main_layout)


# 테스트용 UI 실행
if __name__ == "__main__":
    app = QApplication(sys.argv)
    # window = StatusView(400, 400)
    # window.setWindowTitle("Spot Robot Controller")
    # window.show()
    sys.exit(app.exec())
