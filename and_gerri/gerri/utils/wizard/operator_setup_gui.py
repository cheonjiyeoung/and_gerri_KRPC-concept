import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QComboBox, QTextEdit, QFormLayout, QMessageBox
)
from PySide6.QtCore import Qt

from wizard.operator_setup import *  # 기존 로직 모듈


class WizardWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GERRI Operator Setup Wizard")
        self.setMinimumWidth(500)

        # ------------------------
        # UI Components
        # ------------------------
        self.edit_robot_model = QLineEdit()
        self.edit_robot_id = QLineEdit()

        self.combo_robot_type = QComboBox()
        self.combo_robot_type.addItems([
            "Mobile", "Manipulator", "Mobile + Manipulator", "Another (Unsupport)"
        ])

        self.edit_rubberneck_id = QLineEdit()
        self.edit_rubberneck_pwd = QLineEdit()
        self.edit_rubberneck_pwd.setEchoMode(QLineEdit.Password)

        self.btn_run = QPushButton("Generate Operator Package")
        self.btn_run.clicked.connect(self.run_wizard)

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        # ------------------------
        # Layout
        # ------------------------
        form = QFormLayout()
        form.addRow("Robot Model:", self.edit_robot_model)
        form.addRow("Robot ID:", self.edit_robot_id)
        form.addRow("Robot Type:", self.combo_robot_type)
        form.addRow("RubberNeck ID:", self.edit_rubberneck_id)
        form.addRow("RubberNeck Password:", self.edit_rubberneck_pwd)

        layout = QVBoxLayout()
        layout.addLayout(form)
        layout.addWidget(self.btn_run)
        layout.addWidget(QLabel("Logs:"))
        layout.addWidget(self.log)

        self.setLayout(layout)

    # ------------------------
    # Wizard Run
    # ------------------------
    def run_wizard(self):
        model = self.edit_robot_model.text().strip()
        rid = self.edit_robot_id.text().strip()
        rtype_index = self.combo_robot_type.currentIndex() + 1
        rn_id = self.edit_rubberneck_id.text().strip()
        rn_pwd = self.edit_rubberneck_pwd.text().strip()

        if not (model and rid and rn_id and rn_pwd):
            QMessageBox.warning(self, "Input Error", "Please fill all fields.")
            return

        config = {
            "robot_model": model,
            "robot_id": rid,
            "robot_type": ROBOT_TYPE_MAP[rtype_index],
            "rubberneck_id": rn_id,
            "rubberneck_pwd": rn_pwd,
        }

        try:
            self.log.append("Running setup wizard...\n")
            main(config)
            self.log.append("Completed! Operator package created.\n")

            QMessageBox.information(self, "Success", "Operator package generated!")

        except Exception as e:
            self.log.append(f"Error: {e}\n")
            QMessageBox.critical(self, "Error", str(e))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = WizardWindow()
    window.show()
    sys.exit(app.exec())
