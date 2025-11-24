import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QComboBox, QTextEdit, QFormLayout, QMessageBox, QHBoxLayout
)
from PySide6.QtCore import Qt

from wizard.robot_setup import *   # 너의 기존 CLI 로직 파일


class GerriRobotSetupGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GERRI Robot Setup Wizard")
        self.setMinimumWidth(550)

        # ------------------------
        # Input Widgets
        # ------------------------
        self.edit_robot_model = QLineEdit()
        self.edit_robot_id = QLineEdit()

        self.combo_robot_type = QComboBox()
        self.combo_robot_type.addItems([
            "Mobile",
            "Manipulator",
            "Mobile + Manipulator",
            "Another (Unsupport)"
        ])
        self.combo_robot_type.currentIndexChanged.connect(self.on_robot_type_change)

        self.edit_api_key = QLineEdit()

        # --- Mobile Params ---
        self.edit_max_lin = QLineEdit("1.0")
        self.edit_min_lin = QLineEdit("0.1")
        self.edit_max_ang = QLineEdit("1.0")
        self.edit_min_ang = QLineEdit("0.1")

        # --- Manipulator Params ---
        self.edit_joint_count = QLineEdit()

        # Visibility control
        self.mobile_fields = [self.edit_max_lin, self.edit_min_lin, self.edit_max_ang, self.edit_min_ang]
        self.manip_fields = [self.edit_joint_count]

        # ------------------------
        # Buttons + Log
        # ------------------------
        self.btn_run = QPushButton("Run Setup")
        self.btn_run.clicked.connect(self.run_setup)

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        # ------------------------
        # Layout
        # ------------------------
        form = QFormLayout()
        form.addRow("Robot Model:", self.edit_robot_model)
        form.addRow("Robot ID:", self.edit_robot_id)
        form.addRow("Robot Type:", self.combo_robot_type)
        form.addRow("RubberNeck API Key:", self.edit_api_key)

        # Mobile fields
        form.addRow("MAX Linear Velocity:", self.edit_max_lin)
        form.addRow("MIN Linear Velocity:", self.edit_min_lin)
        form.addRow("MAX Angular Velocity:", self.edit_max_ang)
        form.addRow("MIN Angular Velocity:", self.edit_min_ang)

        # Manipulator fields
        form.addRow("Manipulator Joint Count:", self.edit_joint_count)

        layout = QVBoxLayout()
        layout.addLayout(form)
        layout.addWidget(self.btn_run)
        layout.addWidget(QLabel("Logs:"))
        layout.addWidget(self.log)

        self.setLayout(layout)

        self.on_robot_type_change()

    # ------------------------
    # Robot Type에 따라 입력폼 숨기기
    # ------------------------
    def on_robot_type_change(self):
        idx = self.combo_robot_type.currentIndex() + 1

        # Mobile 관련
        visible_mobile = idx in (1, 3)
        for w in self.mobile_fields:
            w.setVisible(visible_mobile)

        # Manipulator 관련
        visible_manip = idx in (2, 3)
        for w in self.manip_fields:
            w.setVisible(visible_manip)

    # ------------------------
    # Run Setup Wizard
    # ------------------------
    def run_setup(self):
        try:
            robot_model = self.edit_robot_model.text().strip()
            robot_id = self.edit_robot_id.text().strip()
            robot_type_id = self.combo_robot_type.currentIndex() + 1
            api_key = self.edit_api_key.text().strip()

            if not robot_model or not robot_id or not api_key:
                QMessageBox.warning(self, "Missing Input", "Fill all required fields.")
                return

            config = {
                "robot_id": robot_id,
                "robot_model": robot_model,
                "robot_type": ROBOT_TYPE_MAP[robot_type_id],
                "rubberneck_api_key": api_key,
                "sub_controller_template": robot_type_id,
            }

            # Mobile parameters
            if robot_type_id in (1, 3):
                config["max_linear_velocity"] = float(self.edit_max_lin.text() or "1.0")
                config["min_linear_velocity"] = float(self.edit_min_lin.text() or "0.1")
                config["max_angluar_velocity"] = float(self.edit_max_ang.text() or "1.0")
                config["min_angluar_velocity"] = float(self.edit_min_ang.text() or "0.1")
            else:
                config["max_linear_velocity"] = None
                config["min_linear_velocity"] = None
                config["max_angluar_velocity"] = None
                config["min_angluar_velocity"] = None

            # Manipulator parameters
            if robot_type_id in (2, 3):
                config["manipulator_joint_count"] = int(self.edit_joint_count.text() or "1")
            else:
                config["manipulator_joint_count"] = None

            # ------------------------
            # Run Core Logic
            # ------------------------
            self.log.append("Running setup...\n")
            main(config)
            self.log.append("Setup completed.\n")

            QMessageBox.information(self, "Success", "Robot setup completed successfully!")

        except Exception as e:
            self.log.append(f"Error: {e}\n")
            QMessageBox.critical(self, "Error", str(e))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GerriRobotSetupGUI()
    window.show()
    sys.exit(app.exec())
