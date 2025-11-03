import json
from PySide6.QtWidgets import (
    QApplication, QWidget, QFormLayout, QLineEdit, QLabel,
    QPushButton, QVBoxLayout, QHBoxLayout, QGroupBox, QFileDialog, QMessageBox
)
from PySide6.QtCore import Qt
from PySide6.QtCore import Signal
import pprint


# ------------------------
# 필드 설명 매핑
# ------------------------
FIELD_HINTS = {
    # ROBOT_INFO
    "id": "Unique robot identifier (e.g., GERRI_01)",
    "model": "Robot model name (e.g., GerriBot-Mini)",
    "category": "Robot category or type (e.g., mobile, arm)",
    "api_key": "API key used for authentication with Rubberneck",

    # VIDEO_INFO
    "front_cam.source": "Front camera device index (0 for default camera)",
    "front_cam.width": "Front camera resolution width (pixels)",
    "front_cam.height": "Front camera resolution height (pixels)",
    "rear_cam.source": "Rear camera device index (2 for USB camera, etc.)",
    "rear_cam.width": "Rear camera resolution width (pixels)",
    "rear_cam.height": "Rear camera resolution height (pixels)",

    # AUDIO_INFO
    "audio.input": "Audio input device name (e.g., microphone ID)",
    "audio.output": "Audio output device name (e.g., speaker ID)",
}

SHARED_FIELDS = ["id", "model", "category"]
# ------------------------
# 기본 템플릿
# ------------------------
ROBOT_INFO = {
    "id": None,
    "model": None,
    "category": None,
    "api_key": None,
}

VIDEO_INFO = {
    "front_cam": {"source": 0, "width": 640, "height": 480},
    "rear_cam": {"source": 2, "width": 640, "height": 480},
}

AUDIO_INFO = {
    "audio": {"input": "default", "output": "default"},
}


# ------------------------
# ConfigEditor Class
# ------------------------
class RobotConfigEditor(QGroupBox):
    sharedFieldChanged = Signal(str, str)
    def __init__(self):
        super().__init__("Robot Config")
        layout = QVBoxLayout()
        layout.addWidget(self.create_robot_group())
        layout.addWidget(self.create_video_group())
        layout.addWidget(self.create_audio_group())
        layout.addStretch()
        self.setLayout(layout)

    # ------------------------
    # Group Sections
    # ------------------------
    def create_robot_group(self):
        group = QGroupBox("Robot Info")
        form = QFormLayout()
        self.robot_fields = {}

        for key, value in ROBOT_INFO.items():
            edit = QLineEdit("" if value is None else str(value))
            edit.setToolTip(FIELD_HINTS.get(key, f"Set value for {key}"))
            edit.textChanged.connect(lambda text, k=key: self.on_field_changed(k, text))
            self.robot_fields[key] = edit
            form.addRow(QLabel(key.capitalize()), edit)

        group.setLayout(form)
        return group

    def create_video_group(self):
        group = QGroupBox("Video Info")
        layout = QVBoxLayout()
        self.video_fields = {}

        for cam, info in VIDEO_INFO.items():
            sub_group = QGroupBox(cam)
            form = QFormLayout()
            self.video_fields[cam] = {}
            for key, value in info.items():
                edit = QLineEdit("" if value is None else str(value))
                tooltip_key = f"{cam}.{key}"
                edit.setToolTip(FIELD_HINTS.get(tooltip_key, f"Set value for {tooltip_key}"))
                self.video_fields[cam][key] = edit
                form.addRow(QLabel(key.capitalize()), edit)
            sub_group.setLayout(form)
            layout.addWidget(sub_group)
        group.setLayout(layout)
        return group

    def create_audio_group(self):
        group = QGroupBox("Audio Info")
        form = QFormLayout()
        self.audio_fields = {}
        for key, value in AUDIO_INFO["audio"].items():
            edit = QLineEdit("" if value is None else str(value))
            tooltip_key = f"audio.{key}"
            edit.setToolTip(FIELD_HINTS.get(tooltip_key, f"Set value for {tooltip_key}"))
            self.audio_fields[key] = edit
            form.addRow(QLabel(key.capitalize()), edit)
        group.setLayout(form)
        return group

    # ------------------------
    # Save and Load
    # ------------------------
    def save_config(self,path):
        """현재 UI 입력값을 Python 딕셔너리 형태(.py 파일)로 저장"""
        config = {
            "ROBOT_INFO": {k: v.text() or None for k, v in self.robot_fields.items()},
            "VIDEO_INFO": {
                cam: {k: v.text() or None for k, v in vals.items()}
                for cam, vals in self.video_fields.items()
            },
            "AUDIO_INFO": {"audio": {k: v.text() or None for k, v in self.audio_fields.items()}},
        }

        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write("# Auto-generated Robot Config\n")
                f.write("# Modify values below as needed.\n\n")

                # 보기 좋게 들여쓴 Python 포맷으로 출력
                for key, value in config.items():
                    f.write(f"{key} = ")
                    formatted = json.dumps(value, indent=4, ensure_ascii=False)
                    formatted = formatted.replace("true", "True").replace("false", "False").replace("null", "None")
                    f.write(formatted)
                    f.write("\n\n")

        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def on_field_changed(self, key, text):
        """공유 필드면 시그널 발생"""
        if key in SHARED_FIELDS:
            self.sharedFieldChanged.emit(key, text)


if __name__ == "__main__":
    app = QApplication([])
    editor = RobotConfigEditor()
    editor.show()
    app.exec()
