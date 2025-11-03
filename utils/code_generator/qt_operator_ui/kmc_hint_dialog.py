import json
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QScrollArea, QWidget, QComboBox, QFormLayout
)
from PySide6.QtCore import Qt
from keyboard_mouse_controller_map import KEYBOARD_MOUSE_CONTROLLER_MAP
import pprint

# -------------------------------
# 🔹 함수 + 파라미터 매핑 다이얼로그
# -------------------------------
class FunctionParameterDialog(QDialog):
    def __init__(self, functions_info: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Function and Parameters")
        self.functions_info = functions_info
        self.selected_function = None
        self.param_values = {}

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)

        # 함수 선택 콤보박스
        self.function_combo = QComboBox()
        self.function_combo.addItems(self.functions_info.keys())
        self.function_combo.addItem("Unmapped")
        self.function_combo.currentTextChanged.connect(self.update_params_ui)
        layout.addWidget(QLabel("Select Function:"))
        layout.addWidget(self.function_combo)

        # 파라미터 입력 구역
        self.param_area = QWidget()
        self.param_form = QFormLayout(self.param_area)
        layout.addWidget(self.param_area)

        # 버튼
        btn_layout = QHBoxLayout()
        btn_ok = QPushButton("OK")
        btn_cancel = QPushButton("Cancel")
        btn_ok.clicked.connect(self.accept)
        btn_cancel.clicked.connect(self.reject)
        btn_layout.addWidget(btn_ok)
        btn_layout.addWidget(btn_cancel)
        layout.addLayout(btn_layout)

        self.update_params_ui(self.function_combo.currentText())

    def update_params_ui(self, func_name):
        while self.param_form.count():
            item = self.param_form.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        func_info = self.functions_info.get(func_name, {})
        params = func_info.get("parameters", {})
        self.param_inputs = {}
        for pname, pinfo in params.items():
            edit = QLineEdit()
            edit.setPlaceholderText(f"{pinfo['type']} / default={pinfo['default']}")
            self.param_form.addRow(QLabel(pname), edit)
            self.param_inputs[pname] = edit

    def get_result(self):
        func_name = self.function_combo.currentText()
        if func_name == "Unmapped":
            return None, {}
        func_info = self.functions_info[func_name]
        params_info = func_info.get("parameters", {})

        params = {}
        for (pname, pinfo), widget in zip(params_info.items(), self.param_inputs.values()):
            text = widget.text().strip()
            params[pname] = text if text else str(pinfo["default"])

        return func_name, params


# -------------------------------
# 🔹 메인 다이얼로그 (키 매핑)
# -------------------------------
class KeyboardMouseMapDialog(QDialog):
    def __init__(self, key_map, functions_info=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Keyboard & Mouse Controller Map")
        self.resize(650, 700)
        self.key_map_file = "keyboard_mouse_controller_map.py"

        self.map_data = key_map.copy()
        self.functions_info = functions_info or {}

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)

        # 각 키 라인 생성
        for key_name, mapped_value in self.map_data.items():
            row_layout = QHBoxLayout()

            lbl_key = QLabel(key_name)
            lbl_key.setFixedWidth(140)
            row_layout.addWidget(lbl_key)

            line_edit = QLineEdit()
            if mapped_value is None:
                line_edit.setText("DoubleClick to Mapping")
                line_edit.setStyleSheet("color: gray;")
            elif isinstance(mapped_value, dict):
                func = mapped_value.get("function")
                params = mapped_value.get("params")
                summary = f"{func}({', '.join(f'{k}={v}' for k, v in params.items())})"
                line_edit.setText(summary)
            else:
                line_edit.setText(str(mapped_value))

            line_edit.setReadOnly(True)
            line_edit.mouseDoubleClickEvent = lambda e, k=key_name, le=line_edit: self.open_function_dialog(k, le)

            row_layout.addWidget(line_edit)
            scroll_layout.addLayout(row_layout)

        scroll_widget.setLayout(scroll_layout)
        scroll.setWidget(scroll_widget)
        main_layout.addWidget(scroll)

        btn_save = QPushButton("💾 Save")
        btn_save.clicked.connect(self.on_save_clicked)
        main_layout.addWidget(btn_save, alignment=Qt.AlignRight)

    def open_function_dialog(self, key_name, line_edit):
        dlg = FunctionParameterDialog(self.functions_info, self)
        if dlg.exec() == QDialog.Accepted:
            func, params = dlg.get_result()
            if func is None:
                line_edit.setText('DoubleClick to Mapping')
                line_edit.setStyleSheet("color: gray;")
                self.map_data[key_name] = None
            else:
                summary = f"{func}({', '.join(f'{k}={v}' for k, v in params.items())})"
                line_edit.setText(summary)
                self.map_data[key_name] = {"function": func, "params": params}

    def on_save_clicked(self):
        """저장 버튼 클릭 시"""
        self.save_to_file()
        self.accept()

    def save_to_file(self):
        """KEYBOARD_MOUSE_CONTROLLER_MAP 파일 덮어쓰기 (예쁘게 정렬)"""
        try:
            pretty = pprint.pformat(self.map_data, indent=4, width=120)
            # 👉 '{' 바로 뒤에 줄바꿈 추가
            pretty = pretty.replace("{   ", "{\n    ")

            with open(self.key_map_file, "w", encoding="utf-8") as f:
                f.write("# Auto-generated keyboard/mouse controller map\n")
                f.write("KEYBOARD_MOUSE_CONTROLLER_MAP = ")
                f.write(pretty)
                f.write("\n")

            print(f"✅ Saved mapping to {self.key_map_file}")
        except Exception as e:
            print(f"❌ Failed to save: {e}")


# -------------------------------
# 🔹 테스트 실행
# -------------------------------
if __name__ == "__main__":
    from PySide6.QtWidgets import QApplication
    import sys

    CAPECITY_INFO_TEMPLATE = {
        "topic": "capecity_info",
        "value": {
            "move": {
                "parameters": {
                    "vx": {"default": 0.0, "type": "float"},
                    "vy": {"default": 0.0, "type": "float"},
                    "vth": {"default": 0.0, "type": "float"},
                }
            },
            "move_poi": {
                "parameters": {
                    "poi": {"default": None, "type": "string"},
                }
            },
        },
    }

    app = QApplication(sys.argv)
    dlg = KeyboardMouseMapDialog(
        key_map=KEYBOARD_MOUSE_CONTROLLER_MAP,
        functions_info=CAPECITY_INFO_TEMPLATE["value"],
    )
    dlg.exec()
