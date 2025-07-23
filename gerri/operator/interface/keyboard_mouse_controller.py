import sys
from PySide6.QtCore import Qt, QTimer, QUrl
from PySide6.QtGui import QCursor
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PySide6.QtWebEngineWidgets import QWebEngineView
from dns.name import empty
from pubsub import pub

class KeyboardMouseController(QMainWindow):
    def __init__(self, robot_id=None, polling_rate_ms=100, custom_ui=None):
        super().__init__()
        self.setWindowTitle("avatar web update")
        self.showMaximized()  # 창을 최대화합니다
        layout = QVBoxLayout()
        if custom_ui is None:
            self.browser = QWebEngineView()
            # self.browser_url = "http://175.126.123.199"
            self.browser_url = "https://rubberneck.kr/remote-monitor/{robot_id}?hide-menu-bar=true&show-delay=true".format(robot_id=robot_id)
            self.browser.load("https://rubberneck.kr")
            layout.addWidget(self.browser)
        else:
            layout.addWidget(custom_ui)

        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)


        self.command_dict = {}
        
        self.key_set = set()
        self.toggle_set = set()

        self.mx = 0
        self.my = 0

        self.delta = 0

        self.m_dx = 0
        self.m_dy = 0

        self.wheel_dx = 0
        self.wheel_dy = 0

        self.d_scale = 50

        self.speed = 100

        self.key_set = set()

        self.mouse_click = [0, 0, 0]
        self.control_status = False

        self.KMC_app = None

        self.login = False

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_robot_control)
        self.timer.start(polling_rate_ms)

        self.last_command_dict = None


    def start(self):
        self.KMC_app = QApplication(sys.argv)
        self.show()
        sys.exit(self.KMC_app.exec())

    def init_web(self):
        if not self.login:
            self.setMouseTracking(True)
            self.grabMouse()
            self.centerMouseCursor()
            self.browser.setUrl(QUrl(self.browser_url))
            self.login = True
        print("LOGIN")


    def toggle_control(self):
        self.control_status = not self.control_status
        pub.sendMessage('toggle_control', flag=self.control_status)

    def enable_control(self):
        if not self.login:
            self.init_web()
        self.control_status = True
        pub.sendMessage('toggle_control', flag=self.control_status)

    def disable_control(self):
        self.control_status = False
        pub.sendMessage('toggle_control', flag=self.control_status)


    def centerMouseCursor(self):
        screen_center = self.rect().center()
        QCursor.setPos(self.mapToGlobal(screen_center))

    def mouseMoveEvent(self, event):
        if self.control_status:
            screen_center = self.rect().center()
            dx = event.x() - screen_center.x()
            dy = event.y() - screen_center.y()

            # 화면 크기에 따라 비율적으로 dx, dy를 조정합니다.
            screen_width = self.rect().width()
            screen_height = self.rect().height()
            self.m_dx += dx / screen_width
            self.m_dy += dy / screen_height

            self.centerMouseCursor()


    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.mouse_click[0] = 1
        if event.button() == Qt.MouseButton.MiddleButton:
            self.mouse_click[1] = 1
        if event.button() == Qt.MouseButton.RightButton:
            self.mouse_click[2] = 1

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.mouse_click[0] = 0
        if event.button() == Qt.MouseButton.MiddleButton:
            self.mouse_click[1] = 0
        if event.button() == Qt.MouseButton.RightButton:
            self.mouse_click[2] = 0
    def wheelEvent(self, event):
        x_delta = event.angleDelta().x()
        y_delta = event.angleDelta().y()
        # 휠 스크롤 업/다운에 따라 로봇을 제어합니다.
        self.wheel_dx = x_delta / 1000
        self.wheel_dy = y_delta / 1000

    def keyPressEvent(self, event):
        self.handle_key_event(event.key(), pressed=True)

    def keyReleaseEvent(self, event):
        self.handle_key_event(event.key(), pressed=False)

    def get_key_name(self, key):
        """Qt 키 코드를 문자로 변환합니다."""
        # 알파벳 키 처리 (A~Z)
        if Qt.Key.Key_A <= key <= Qt.Key.Key_Z:
            return chr(key)

        # 숫자 키 처리 (0~9)
        elif Qt.Key.Key_0 <= key <= Qt.Key.Key_9:
            return chr(key)

        # MAPPING FOR OTHER KEYS (MANUAL DEFINITION)
        key_map = {
            Qt.Key.Key_Space: 'SPACE',
            Qt.Key.Key_Up: 'UP ARROW',
            Qt.Key.Key_Down: 'DOWN ARROW',
            Qt.Key.Key_Left: 'LEFT ARROW',
            Qt.Key.Key_Right: 'RIGHT ARROW',
            Qt.Key.Key_Enter: 'ENTER',
            Qt.Key.Key_Return: 'RETURN',
            Qt.Key.Key_Escape: 'ESC',
            Qt.Key.Key_Control: 'CONTROL',
            Qt.Key.Key_Shift: 'SHIFT',
            Qt.Key.Key_Alt: 'ALT',
            Qt.Key.Key_Tab: 'TAB',
            Qt.Key.Key_Backspace: 'BACKSPACE',
            Qt.Key.Key_Delete: 'DELETE',
            Qt.Key.Key_F1: 'F1',
            Qt.Key.Key_F2: 'F2',
            Qt.Key.Key_F3: 'F3',
            Qt.Key.Key_F4: 'F4',
            Qt.Key.Key_F5: 'F5',
            Qt.Key.Key_F6: 'F6',
            Qt.Key.Key_F7: 'F7',
            Qt.Key.Key_F8: 'F8',
            Qt.Key.Key_F9: 'F9',
            Qt.Key.Key_F10: 'F10',
            Qt.Key.Key_F11: 'F11',
            Qt.Key.Key_F12: 'F12',
            Qt.Key.Key_F13: 'F13',
            0: 'SHIFT'
        }

        if key in key_map:
            return key_map[key]
        else:
            return f"Unknown({key})"

    def handle_key_event(self, key, pressed):
        key_name = self.get_key_name(key)
        if pressed:
            self.key_set.add(key_name)  # 누른 키를 추가
            print(f"Pressed: {key_name}")
            self.toggle_key(key_name)

        else:
            self.key_set.discard(key_name)  # 뗀 키를 제거
            print(f"Released: {key_name}")
            if 'ESC' == key_name:
                self.disable_control()
                print('KEYBOARD AND MOUSE CONTROL OFF')

        # SHIFT + RETURN 조합이 눌렸는지 확인
        if {'SHIFT', 'RETURN'}.issubset(self.key_set):
            self.enable_control()
            print('KEYBOARD AND MOUSE CONTROL ON')


        if key == Qt.Key.Key_Up:
            if self.speed < 150:
                self.speed += 10
        if key == Qt.Key.Key_Down:
            if self.speed > 10:
                self.speed -= 10
        if key == Qt.Key.Key_BracketLeft:
            if self.d_scale > 1:
                self.d_scale = self.d_scale - 1
            print(self.d_scale)
        if key == Qt.Key.Key_BracketRight:
            self.d_scale = self.d_scale + 1
            print(self.d_scale)

    def toggle_key(self, key_name):
        """토글 키의 상태를 변경합니다."""
        if key_name in self.toggle_set:
            self.toggle_set.remove(key_name)
            print(f"{key_name} OFF")
        else:
            self.toggle_set.add(key_name)
            print(f"{key_name} ON")
        print(self.toggle_set)


    def update_robot_control(self):
        self.mx = self.mx + self.m_dx * self.d_scale / 50
        self.my = self.my + self.m_dy * self.d_scale / 50


        # x, y 값 리미터
        if self.mx < -1:
            self.mx = -1
        elif self.mx > 1:
            self.mx = 1

        if self.my < -1:
            self.my = -1
        elif self.my > 1:
            self.my = 1


        # print(self.mz)

        # print(self.mx, self.my, self.mz)

        self.command_dict['mouse_move'] = self.mx, self.my
        self.command_dict['mouse_d_move'] = [self.m_dx, self.m_dy]
        self.command_dict['mouse_d_wheel'] = [self.wheel_dx, self.wheel_dy]
        self.command_dict['mouse_click'] = self.mouse_click
        self.command_dict['key_control'] = self.key_set
        self.command_dict['toggle'] = self.toggle_set
        self.command_dict['speed'] = self.speed

        # print(self.command_dict)

        self.publish_message()
        self.init_command()

    def publish_message(self):
        pub.sendMessage('key_mouse_control', command=self.command_dict)

    def init_command(self):
        self.m_dx = 0
        self.m_dy = 0

        self.wheel_dx = 0
        self.wheel_dy = 0

# 실행 코드
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = KeyboardMouseController()
    window.show()
    sys.exit(app.exec())
