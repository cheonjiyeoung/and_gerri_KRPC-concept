import os,sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from PySide6.QtWidgets import QTabWidget,QWidget,QVBoxLayout
from gerri.operator.examples.spot.spot_ui.view.spot_act_controller import SpotActController
from gerri.operator.examples.spot.spot_ui.view.spot_ptz_controller import SpotPTZController
from gerri.operator.examples.spot.spot_ui.view.spot_mission_controller import SpotMissionController
from gerri.operator.examples.spot.spot_ui.view.status_view import StatusView
from gerri.operator.examples.spot.spot_ui.view.webrtc_connection import WebrtcConnectionViewer
from PySide6.QtCore import QTimer

class SpotControllerPannel(QWidget):
    def __init__(self,w,h,webrtc_channels,status,spot_info):
        super().__init__()
        self.widget_width = w
        self.widget_height = h
        self.setFixedSize(w,h)
        self.channels = webrtc_channels
        self.webrtc_connection = WebrtcConnectionViewer(self.channels)

        tab_height = h * (0.7)
        self.tab = QTabWidget()
        act_controller = SpotActController(w,tab_height,spot_info=spot_info)
        self.mission_controller = SpotMissionController(status=status,spot_info=spot_info)
        self.tab.addTab(act_controller, "action")

        self.spot_info = spot_info

        if spot_info["using_ptz"]:
            self.ptz_controller = SpotPTZController(w,tab_height,status=status,spot_info=spot_info)
            self.tab.addTab(self.ptz_controller, "ptz")

        self.tab.addTab(self.webrtc_connection, "webrtc")
        self.tab.addTab(self.mission_controller, "mission")

        status_view_height = h * (0.3) - 5
        self.status_view = StatusView(w,status_view_height,status=status)

        layout_main = QVBoxLayout(self)
        layout_main.addWidget(self.tab)
        layout_main.addSpacing(5)
        layout_main.addWidget(self.status_view)
        layout_main.addSpacing(5)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(500)  # 500ms 간격 (0.5초마다 update_ui 호출)

    def update_ui(self):
        try:
            if self.spot_info["using_ptz"]:
                self.ptz_controller.update_ui()
            self.mission_controller.update_ui()
            self.status_view.update_ui()
        except:
            pass

if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication
    from PySide6.QtGui import QGuiApplication

    app = QApplication(sys.argv)

    screen_geometry = QGuiApplication.primaryScreen().availableGeometry()
    screen_width = screen_geometry.width()
    screen_height = screen_geometry.height()

    # 예: 전체 화면의 80%만 사용
    window_width = int(screen_width)
    window_height = int(screen_height)

    spot_controller_width = window_width * 0.1

    window = SpotControllerPannel(spot_controller_width, window_height)
    window.show()

    sys.exit(app.exec())
