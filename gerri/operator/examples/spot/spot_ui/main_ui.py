import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.examples.spot.spot_ui.view.video_viewer import VideoViewer
from gerri.operator.examples.spot.spot_ui.view.spot_controller_pannel import SpotControllerPannel
from gerri.operator.examples.spot.spot_ui.view.spot_mission_controller import SpotMissionController
from gerri.operator.examples.spot.spot_ui.view.spot_question_dialog import SpotQuestionDialog

from PySide6.QtGui import QPixmap, QImage, QColor
from PySide6.QtCore import Qt, QTimer, QMetaObject, Q_ARG

from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout
from pubsub import pub

class SpotOperationUI(QWidget):
    def __init__(self, w, h, status, webrtc_channels, spot_info):
        super().__init__()
        self.w = w
        self.h = h
        self.last_question_id = None
        self.setFixedSize(self.w, self.h)

        main_cam_veiwer_w = w * 0.85 - 10
        main_cam_veiwer_h = h * 0.7 - 5
        main_cam_veiwer = VideoViewer(main_cam_veiwer_w,
                                      main_cam_veiwer_h,
                                      reciever=webrtc_channels["ptz_cam"])

        rest_cam_width = w * 0.85 - 10
        rest_cam_height = h * 0.3 - 5
        rest_cam_veiwer = VideoViewer(rest_cam_width,
                                      rest_cam_height,
                                      reciever=webrtc_channels["rest_cam"])

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

        pub.subscribe(self.handle_question, 'spot_question')

    def handle_question(self, value=None):
        print("sival", value)  # 로그로 value 확인
        """
        Handles a question from the robot, typically involving user interaction.
        """
        if self.last_question_id is None or value['id'] != self.last_question_id:
            self.last_question_id = value['id']
            
            # 메인 스레드에서 다이얼로그를 띄울 수 있도록 invokeMethod 사용
            QMetaObject.invokeMethod(self, "show_question_dialog", Qt.QueuedConnection, Q_ARG(object, value))
        else:
            print("\n\nDuplicate question received, ignoring.\n\n")

    def show_question_dialog(self, value):
        """
        다이얼로그를 메인 스레드에서 띄웁니다.
        """
        print("Showing dialog with value:", value)  # 이 부분도 로그로 출력하여 확인
        dialog = SpotQuestionDialog(text=value['text'], options=value['options'])
        dialog.setParent(self)  # 부모 위젯 설정
        dialog.show()  # 다이얼로그를 모달리스로 띄우기

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
