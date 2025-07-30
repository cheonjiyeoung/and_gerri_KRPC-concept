from PySide6.QtWidgets import QWidget,QVBoxLayout,QPushButton,QLabel,QLineEdit,QComboBox,QGridLayout,QDoubleSpinBox
import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from pubsub import pub
from gerri.operator.examples.spot.spot_ui.utils.my_widgets import PubSubButton, PubSubDBSpinbox

class SpotActController(QWidget):
    def __init__(self,w,h,spot_info):
        super().__init__()
        self.widget_width = w
        self.widget_height = h
        self.ijkl_mode = "STANCE"
        # self.setFixedSize(w,h)
        self.spot_info = spot_info

        btn_stand = PubSubButton("Stand")
        btn_sit = PubSubButton("Sit")
        btn_walk = PubSubButton("Walk")
        btn_stop = PubSubButton("Stop")
        btn_docking = PubSubButton("Docking")
        btn_undocking = PubSubButton("UnDocking")
        
        spot_stance = QWidget()
        layout_spot_stance = QGridLayout(spot_stance)
        layout_spot_stance.addWidget(QLabel("Spot Stance"),0,0,1,2)
        layout_spot_stance.addWidget(btn_stand,1,0)
        layout_spot_stance.addWidget(btn_sit,1,1)
        layout_spot_stance.addWidget(btn_walk,2,0)
        layout_spot_stance.addWidget(btn_stop,2,1)
        layout_spot_stance.addWidget(btn_docking,3,0)
        layout_spot_stance.addWidget(btn_undocking,3,1)

        self.spin_default_linear_speed = PubSubDBSpinbox("spin_default_linear_speed")
        self.spin_default_linear_speed.setRange(0.1, 1.0)
        self.spin_default_linear_speed.setSingleStep(0.1)
        self.spin_default_linear_speed.setDecimals(1)
        self.spin_default_linear_speed.setValue(spot_info["default_linear_speed"])

        self.spin_low_linear_speed = PubSubDBSpinbox("spin_low_linear_speed")
        self.spin_low_linear_speed.setRange(0.1, 1.0)
        self.spin_low_linear_speed.setSingleStep(0.1)
        self.spin_low_linear_speed.setDecimals(1)
        self.spin_low_linear_speed.setValue(spot_info["default_linear_low_speed"])

        self.spin_default_angular_speed = PubSubDBSpinbox("spin_default_angular_speed")
        self.spin_default_angular_speed.setRange(0.1, 1.0)
        self.spin_default_angular_speed.setSingleStep(0.1)
        self.spin_default_angular_speed.setDecimals(1)
        self.spin_default_angular_speed.setValue(spot_info["default_angular_speed"])

        self.spin_low_angular_speed = PubSubDBSpinbox("spin_low_angular_speed")
        self.spin_low_angular_speed.setRange(0.1, 1.0)
        self.spin_low_angular_speed.setSingleStep(0.1)
        self.spin_low_angular_speed.setDecimals(1)
        self.spin_low_angular_speed.setValue(spot_info["default_angular_low_speed"])

        set_velocity = QWidget()
        layout_set_velocity = QGridLayout(set_velocity)
        layout_set_velocity.addWidget(QLabel("Set Velocity"),0,0,1,4)

        layout_set_velocity.addWidget(QLabel("Vel :"),1,0)
        layout_set_velocity.addWidget(self.spin_default_linear_speed,1,1)
        layout_set_velocity.addWidget(QLabel("Vel_Low :"),1,2)
        layout_set_velocity.addWidget(self.spin_low_linear_speed,1,3)
        layout_set_velocity.addWidget(QLabel(" "),3,0,1,4)
        
        layout_set_velocity.addWidget(QLabel("Ang :"),4,0)
        layout_set_velocity.addWidget(self.spin_default_angular_speed,4,1)
        layout_set_velocity.addWidget(QLabel("Ang_Low :"),4,2)
        layout_set_velocity.addWidget(self.spin_low_angular_speed,4,3)

        if spot_info["using_ptz"]:
            ijkl_mode = QWidget()
            layout_ijkl_mode = QGridLayout(ijkl_mode)
            layout_ijkl_mode.addWidget(QLabel("IJKL_MODE : "),0,0)
            self.edit_current_ijkl_mode = QLineEdit(self.ijkl_mode,readOnly=True)
            layout_ijkl_mode.addWidget(self.edit_current_ijkl_mode,0,1)
            btn_toggle_ijkl_stance = PubSubButton("Stance")
            btn_toggle_ijkl_stance.clicked.connect(lambda:self.set_ijkl_mode("STANCE"))
            btn_toggle_ijkl_ptz = PubSubButton("PTZ")
            btn_toggle_ijkl_ptz.clicked.connect(lambda:self.set_ijkl_mode("PTZ"))
            layout_ijkl_mode.addWidget(btn_toggle_ijkl_stance,1,0)
            layout_ijkl_mode.addWidget(btn_toggle_ijkl_ptz,1,1)

        layout_main = QVBoxLayout(self)
        layout_main.addWidget(spot_stance)
        layout_main.addSpacing(5)
        layout_main.addWidget(set_velocity)
        if spot_info["using_ptz"]:
            layout_main.addWidget(ijkl_mode)
        layout_main.addStretch(0)
        layout_main.setContentsMargins(0, 0, 0, 0)
        
    def set_ijkl_mode(self,target):
        self.ijkl_mode = target

if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    window = SpotActController(400, 800)  # 원하는 크기 지정
    window.show()
    sys.exit(app.exec())

        