from PySide6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QHBoxLayout, QLineEdit, QGroupBox
from kmc_hint_dialog import KeyboardMouseMapDialog
from keyboard_mouse_controller_map import KEYBOARD_MOUSE_CONTROLLER_MAP

class IntTypeParmeterInput(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)

    def get_value(self):
        text = self.text()
        try:
            value = int(text)
            return value
        except ValueError:
            return None

class StringTypeParmeterInput(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)

    def get_value(self):
        return self.text()
    
class FloatTypeParmeterInput(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)

    def get_value(self):
        text = self.text()
        try:
            value = float(text)
            return value
        except ValueError:
            return None
        
class AnyTypeParameterInput(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)

    def get_value(self):
        text = self.text()
        try:
            return float(text)
        except:
            return text
        
class FunctionButton(QGroupBox):
    def __init__(self, function:dict, params:dict):
        super().__init__(function)
        self.function = function
        self.params = params
        self.param_inputs = {}

        self.layout_main = QVBoxLayout()
        self.init_ui()
        self.setLayout(self.layout_main)

    def init_ui(self):
        layout_params = QHBoxLayout()
        param_texts = []

        # 파라미터 입력창 구성
        for param_name, param_info in self.params['parameters'].items():
            label_param = QLabel(param_name)
            layout_params.addWidget(label_param)

            if param_info['type'] == 'int':
                input_widget = IntTypeParmeterInput()
            elif param_info['type'] == 'string':
                input_widget = StringTypeParmeterInput()
            elif param_info['type'] == 'float':
                input_widget = FloatTypeParmeterInput()
            else:
                input_widget = AnyTypeParameterInput()

            input_widget.setToolTip(str(param_info['type']))
            if param_info['default'] is not None:
                input_widget.setText(str(param_info['default']))

            layout_params.addWidget(input_widget)
            self.param_inputs[param_name] = input_widget  # 저장

            default_value = param_info['default']
            if default_value is not None:
                param_texts.append(f"'{param_name}'={default_value}")
            else:
                param_texts.append(f"'{param_name}'")
            
        
        parameters_info_label = QLabel("params: " + ", ".join(param_texts))

        # 실행 버튼
        btn_func = QPushButton("exec")
        btn_func.clicked.connect(self.on_button_clicked)  # 클릭 이벤트 연결

        layout_main = QVBoxLayout()
        layout_main.addWidget(parameters_info_label)
        layout_main.addLayout(layout_params)
        layout_main.addWidget(btn_func)
        self.layout_main.addLayout(layout_main)

    def on_button_clicked(self):
        params_data = {}
        for param_name, widget in self.param_inputs.items():
            params_data[param_name] = widget.get_value()

        self.send_command(params_data)

    def send_command(self, params):
        command = {"topic":self.function,
                   "value":params}
        
        print(f"Sending command: {command}")


class LayoutFunctions(QWidget):
    def __init__(self, functions:dict):
        super().__init__()
        self.functions = functions
        self.layout_main = QVBoxLayout()
        self.init_ui()
        self.setLayout(self.layout_main)

    def init_ui(self):
        layout_main = QVBoxLayout()
        for func_name, params in self.functions.items():
            print(f"Adding function button for: {func_name}")
            layout_func_button = FunctionButton(func_name, params)
            layout_main.addWidget(layout_func_button)

        btn_exac_kmc_dialog = QPushButton("Keyboard Mouse\nConfig")
        btn_exac_kmc_dialog.clicked.connect(lambda:KeyboardMouseMapDialog(key_map=KEYBOARD_MOUSE_CONTROLLER_MAP,
                                                     functions_info=self.functions).exec())
        layout_main.addWidget(btn_exac_kmc_dialog)
        self.layout_main = layout_main

if __name__ == "__main__":
    from PySide6.QtWidgets import QApplication
    import sys
    CAPECITY_INFO_TEMPLATE = {
        'topic' : 'capecity_info',
        'value' : {
            'move':{
                'parameters' : {
                    'vx' : {"default" : 0.0, "type" : "float"},
                    'vy' : {"default" : 0.0, "type" : "float"},
                    'vth' : {"default" : 0.0, "type" : "float"},
                }
            },
            'move_poi':{
                'parameters' : {
                    'poi' : {"default" : None, "type" : "str"},
                }
            }

        }
    }

    app = QApplication(sys.argv)

    window = LayoutFunctions(CAPECITY_INFO_TEMPLATE['value'])
    window.show()

    sys.exit(app.exec())