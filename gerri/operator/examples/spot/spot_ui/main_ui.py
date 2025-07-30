import os,sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.examples.spot.spot_ui.view.video_viewer import VideoViewer
from gerri.operator.examples.spot.spot_ui.view.spot_controller_pannel import SpotControllerPannel
from gerri.operator.examples.spot.spot_ui.view.spot_mission_controller import SpotMissionController

from PySide6.QtGui import QPixmap, QImage, QColor
from PySide6.QtCore import Qt

from PySide6.QtWidgets import QApplication,QWidget,QVBoxLayout,QHBoxLayout

class SpotOperationUI(QWidget):
    def __init__(self,w,h,status,webrtc_channels,spot_info):
        super().__init__()
        self.w = w
        self.h = h
        # main_container = QWidget()
        self.setFixedSize(self.w,self.h)

        main_cam_veiwer_w = w*0.85 - 10
        main_cam_veiwer_h = h*0.7 - 5
        main_cam_veiwer = VideoViewer(main_cam_veiwer_w,
                                      main_cam_veiwer_h,
                                      reciever=webrtc_channels["ptz_cam"],)

        rest_cam_width = w*0.85 - 10
        rest_cam_height = h*0.3 - 5
        rest_cam_veiwer = VideoViewer(rest_cam_width,
                                      rest_cam_height,
                                      reciever=webrtc_channels["rest_cam"],)

        layout_cam_viewer = QVBoxLayout()
        layout_cam_viewer.addWidget(main_cam_veiwer)
        layout_cam_viewer.addSpacing(10)
        layout_cam_viewer.addWidget(rest_cam_veiwer)

        ctrl_pannel_width = self.w * 0.15
        ctrl_pannel_height = self.h
        controll_pannel = SpotControllerPannel(ctrl_pannel_width,
                                               ctrl_pannel_height,
                                               webrtc_channels=webrtc_channels,
                                               spot_info=spot_info,
                                               status=status)

        layout_full = QHBoxLayout(self)
        layout_full.addLayout(layout_cam_viewer)
        layout_full.addSpacing(10)
        layout_full.addWidget(controll_pannel)



            


if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication
    from PySide6.QtGui import QGuiApplication

    app = QApplication(sys.argv)

    # 전체 화면 해상도 가져오기
    screen_geometry = QGuiApplication.primaryScreen().availableGeometry()
    screen_width = screen_geometry.width()
    screen_height = screen_geometry.height()

    # SpotOperationUI 인스턴스 생성
    window = SpotOperationUI(screen_width, screen_height)

    # 창 전체화면으로 설정
    window.showFullScreen()

    sys.exit(app.exec())



