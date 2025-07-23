from PySide6.QtWidgets import (
    QApplication, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout, QWidget, QLabel, QDoubleSpinBox, QLineEdit, QGroupBox, QTextEdit, QTextBrowser
)
from PySide6.QtCore import Qt
import sys
import datetime
class RobotControllerLog(QWidget):
    def __init__(self, widget_w, widget_h):
        super().__init__()
        # self.setFixedSize(widget_w, widget_h)
        print(f"widget_w: {widget_w}, widget_h: {widget_h}")
        
        main_layout = QVBoxLayout(self)
        self.log_msg = QTextBrowser()
        self.log_msg.setFixedSize(widget_w, widget_h)
        main_layout.addWidget(self.log_msg)
        main_layout.addStretch(0)
    
    def log_msg_input(self, value:str):
        # print(f"log_msg_input: {value}")
        timestamp = datetime.datetime.now()
        text = f"[{timestamp}] {value}"
        self.log_msg.append(text)