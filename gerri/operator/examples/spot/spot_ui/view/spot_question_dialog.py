import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.examples.spot.spot_ui.utils.my_widgets import PubSubButton, PubSubDBSpinbox, PubSubComboBox, PubSubLineEdit
from PySide6.QtWidgets import QDialog, QVBoxLayout, QLabel, QDialogButtonBox, QLineEdit, QMainWindow, QPushButton, QApplication, QGridLayout, QHBoxLayout
from PySide6 import QtCore
import json

class SpotQuestionDialog(QDialog):
    def __init__(self,text="test",options=None):
        super().__init__()
        
        # 대화상자 UI 구성
        self.setWindowTitle("Spot Question")

        layout = QVBoxLayout()
        options = json.dumps(options)

        layout.addWidget(QLabel(f"[Question]\n{text}"))
        layout.addWidget(QLabel(f"[options]\n{options}"))

        layout_answer_code = QHBoxLayout()
        layout_answer_code.addWidget(QLabel("Answer Code:"))
        self.input_field = PubSubLineEdit(name="answer_code")
        layout_answer_code.addWidget(self.input_field)

        layout.addLayout(layout_answer_code)

        # 대화상자에 버튼 추가
        answer_button = PubSubButton("Send Answer")
        answer_button.clicked.connect(self.accept)

        layout.addWidget(answer_button)

        self.setLayout(layout)



if __name__ == "__main__":
    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()

            # 메인 윈도우 설정
            self.setWindowTitle("PySide6 모달리스 예제")
            self.setFixedSize(400, 300)

            # 버튼을 눌러 대화상자를 띄우는 동작
            button = QPushButton("exac", self)
            button.clicked.connect(self.show_dialog)
            button.setGeometry(100, 100, 200, 50)

        def show_dialog(self):
            # CustomDialog 인스턴스를 생성하고 모달리스로 띄움
            dialog = SpotQuestionDialog(text="Spot got stuck. What do you want to do?\n\nReroute length: 3.9m\n\nSpot will [Try again then reroute] in 58 seconds.",
                                options={"Try again": 2, "Try to reroute": 7, "Skip": 3})
            dialog.setWindowModality(QtCore.Qt.NonModal)  # 모달리스 설정
            dialog.exec()  # exec()를 호출해 대화상자 실행
    app = QApplication(sys.argv)
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())