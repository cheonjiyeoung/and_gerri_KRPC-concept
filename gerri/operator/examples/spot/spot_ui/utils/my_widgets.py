from PySide6.QtWidgets import QPushButton, QDoubleSpinBox, QComboBox, QLineEdit
from PySide6.QtCore import Qt
from pubsub import pub

class PubSubButton(QPushButton):
    def __init__(self, name):
        super().__init__(name)
        self.clicked.connect(lambda:pub.sendMessage("ui_signal", signal=name, value=None))

class PubSubDBSpinbox(QDoubleSpinBox):
    def __init__(self,name):
        super().__init__()
        self.setFocusPolicy(Qt.NoFocus)
        self.name = name
        self.valueChanged.connect(lambda:pub.sendMessage("ui_signal", signal=name, value=round(self.value(),1)))

class PubSubComboBox(QComboBox):
    def __init__(self,name):
        super().__init__()
        self.setFocusPolicy(Qt.NoFocus)
        self.name = name
        self.activated.connect(lambda:pub.sendMessage("ui_signal", signal=name, value=self.currentText()))

class PubSubLineEdit(QLineEdit):
    def __init__(self,name):
        super().__init__()
        self.name = name
        self.textChanged.connect(lambda:pub.sendMessage("ui_signal", signal=name, value=self.text()))
        self.textEdited.connect(lambda:pub.sendMessage("ui_signal", signal=name, value=self.text()))