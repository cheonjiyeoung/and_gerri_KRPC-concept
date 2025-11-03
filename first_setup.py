from utils.fast_setup.ui.operator_config_widget import OperatorConfigEditor
from utils.fast_setup.ui.robot_config_widget import RobotConfigEditor
from utils.code_generator import robot_commander_builder
from PySide6.QtWidgets import (
    QApplication, QGridLayout, QGroupBox, QMainWindow, QHBoxLayout,
    QVBoxLayout, QFileDialog, QPushButton, QLineEdit, QWidget, QTextBrowser, QLabel
)


class FirstSetupApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("First Setup Wizard")

        # 상태 변수
        self.robot_interface_path = None
        self.project_dir_path = None
        self.robot_name = None

        layout_robot_name = QHBoxLayout()
        label_robot_name = QLabel("Your Robot Name: ")
        edit_name = QLineEdit()
        edit_name.textChanged.connect(self.on_update_robot_name)
        layout_robot_name.addWidget(label_robot_name)
        layout_robot_name.addWidget(edit_name)


        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout_main = QVBoxLayout(central_widget)

        # 구성 요소
        self.operator_config_editor = OperatorConfigEditor()
        self.robot_config_editor = RobotConfigEditor()
        self.operator_config_editor.sharedFieldChanged.connect(self.sync_robot_editor)
        self.robot_config_editor.sharedFieldChanged.connect(self.sync_operator_editor)

        groupbox_log = QGroupBox("Logs")
        self.log_browser = QTextBrowser()
        layout_log = QVBoxLayout()
        layout_log.addWidget(self.log_browser)
        groupbox_log.setLayout(layout_log)

        layout_config_editor = QHBoxLayout()
        layout_config_editor.addWidget(self.operator_config_editor)
        layout_config_editor.addWidget(self.robot_config_editor)

        layout_path_configuration = QVBoxLayout()

        # ===============================
        # Robot Interface File Path
        # ===============================
        btn_browse_robot_interface_file = QPushButton("Browse")
        btn_browse_robot_interface_file.clicked.connect(self.browse_robot_interface_file_path)
        self.edit_robot_interface_file_path = QLineEdit()
        self.edit_robot_interface_file_path.setReadOnly(True)

        layout_robot_interface_path = QHBoxLayout()
        layout_robot_interface_path.addWidget(self.edit_robot_interface_file_path)
        layout_robot_interface_path.addWidget(btn_browse_robot_interface_file)

        groupbox_robot_interface_file = QGroupBox("Set Robot Interface File Path")
        groupbox_robot_interface_file.setLayout(layout_robot_interface_path)

        # ===============================
        # Project Directory Path
        # ===============================
        btn_browse_project_dir = QPushButton("Browse")
        btn_browse_project_dir.clicked.connect(self.browse_project_dir_path)
        self.edit_project_dir_path = QLineEdit()
        self.edit_project_dir_path.setReadOnly(True)

        layout_project_dir_path = QHBoxLayout()
        layout_project_dir_path.addWidget(self.edit_project_dir_path)
        layout_project_dir_path.addWidget(btn_browse_project_dir)

        groupbox_project_dir_path = QGroupBox("Set Project Directory Path")
        groupbox_project_dir_path.setLayout(layout_project_dir_path)

        layout_buttons = QHBoxLayout()
        btn_exec = QPushButton("Done")
        btn_exec.clicked.connect(self.main)
        btn_cancel = QPushButton("Cancel")
        layout_buttons.addWidget(btn_exec)
        layout_buttons.addWidget(btn_cancel)

        # ===============================
        # 메인 레이아웃에 추가
        # ===============================
        layout_path_configuration.addWidget(groupbox_robot_interface_file)
        layout_path_configuration.addWidget(groupbox_project_dir_path)
        layout_path_configuration.addStretch()

        layout_main.addLayout(layout_robot_name)
        layout_main.addLayout(layout_config_editor)
        layout_main.addLayout(layout_path_configuration)
        layout_main.addLayout(layout_buttons)
        layout_main.addWidget(groupbox_log)

    def on_update_robot_name(self, text):
        self.robot_name = text


    # ------------------------
    # 파일 및 폴더 브라우저
    # ------------------------
    def browse_robot_interface_file_path(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Robot Interface File",
            "./",
            "Python Files (*.py);;All Files (*)"
        )
        if file_path:
            self.edit_robot_interface_file_path.setText(file_path)
            self.robot_interface_path = file_path

    def browse_project_dir_path(self):
        folder_path = QFileDialog.getExistingDirectory(
            self,
            "Select Project Directory",
            "./"
        )
        if folder_path:
            self.edit_project_dir_path.setText(folder_path)
            self.project_dir_path = folder_path

    def sync_robot_editor(self, key, value):
        """Operator → Robot 동기화"""
        if key in self.robot_config_editor.robot_fields:
            self.robot_config_editor.robot_fields[key].blockSignals(True)
            self.robot_config_editor.robot_fields[key].setText(value)
            self.robot_config_editor.robot_fields[key].blockSignals(False)

    def sync_operator_editor(self, key, value):
        """Robot → Operator 동기화"""
        if key in self.operator_config_editor.robot_fields:
            self.operator_config_editor.robot_fields[key].blockSignals(True)
            self.operator_config_editor.robot_fields[key].setText(value)
            self.operator_config_editor.robot_fields[key].blockSignals(False)

    def main(self):
        import os
        self.log_browser.append("Starting SetUp For AND_GERRI...")
        robot_dir = self.project_dir_path + "/" + "robot"
        operator_dir = self.project_dir_path + "/" + "operator"
        if not (os.path.exists(robot_dir) and os.path.isdir(robot_dir)):
            os.makedirs(robot_dir)
        if not (os.path.exists(operator_dir) and os.path.isdir(operator_dir)):
            os.makedirs(operator_dir)

        robot_commander_path = operator_dir + "/" + self.robot_name + "_commander.py"
        robot_config_path = robot_dir + "/" + self.robot_name + "_config.py"
        operator_config_path = operator_dir + "/" + self.robot_name + "_config.py"

        # build Robot Commander
        self.log_browser.append(f"Reading Robot InterfaceFile. ({self.robot_interface_path})")
        self.log_browser.append(f"Generate Robot Commander... save_path = {robot_commander_path}")
        robot_commander_builder.main(self.robot_interface_path,self.robot_name,robot_commander_path)

        # build Robot Config
        self.log_browser.append(f"Generate Robot Config... save_path = {robot_config_path}")
        self.robot_config_editor.save_config(robot_config_path)

        # build Operator Config
        self.log_browser.append(f"Generate Operator Config... save_path = {operator_config_path}")
        self.operator_config_editor.save_config(operator_config_path)

        # build Robot Launch Code
        self.log_browser.append(f"Generate Robot Launch Code... save_path = {robot_commander_path}")
        pass
        self.log_browser.append(f"Generate Operator Launch Code... save_path = {robot_commander_path}")
        pass
        self.log_browser.append(f"Done! you can check your project dir ({self.project_dir_path})")




if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = FirstSetupApp()
    window.show()
    sys.exit(app.exec())
