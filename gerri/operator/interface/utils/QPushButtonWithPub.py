from PySide6.QtWidgets import QPushButton
from pubsub import pub

class QPushButtonWithPub(QPushButton):
    def __init__(self,name:str):
        super().__init__(name)
        self.name = name
        self.clicked.connect(lambda:pub.sendMessage("ui_signal",signal=self.name))