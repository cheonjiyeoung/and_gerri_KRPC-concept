import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QLineEdit, QGridLayout, QHBoxLayout

class StatusView(QWidget):
    def __init__(self,w,h,status):
        super().__init__()
        self.widget_width = w
        self.widget_height = h
        self.setFixedSize(w,h)

        self.status = status

        self.edit_power = QLineEdit(readOnly=True)
        self.edit_battery_amount = QLineEdit(readOnly=True)
        self.edit_charging = QLineEdit(readOnly=True)
        self.edit_mode = QLineEdit(readOnly=True)
        self.edit_sw_estop = QLineEdit(readOnly=True)
        self.edit_hw_estop = QLineEdit(readOnly=True)
        self.edit_payload_estop = QLineEdit(readOnly=True)
        self.edit_pose_x = QLineEdit(readOnly=True)
        self.edit_pose_y = QLineEdit(readOnly=True)
        self.edit_pose_z = QLineEdit(readOnly=True)

        status_4col = QWidget()
        layout_status_4col = QGridLayout(status_4col)
        layout_status_4col.addWidget(QLabel("[ Robot Status ]"),0,0,1,4)

        layout_status_4col.addWidget(QLabel("Power"),1,0,)
        layout_status_4col.addWidget(self.edit_power,1,1)

        layout_status_4col.addWidget(QLabel("Mode"),1,2)
        layout_status_4col.addWidget(self.edit_mode,1,3)

        layout_status_4col.addWidget(QLabel("Battery"),2,0)
        layout_status_4col.addWidget(self.edit_battery_amount,2,1)

        layout_status_4col.addWidget(QLabel("Charging"),2,2)
        layout_status_4col.addWidget(self.edit_charging,2,3)

        status_6col = QWidget()
        layout_status_6col = QGridLayout(status_6col)
        layout_status_6col.addWidget(QLabel("x"),0,0)
        layout_status_6col.addWidget(self.edit_pose_x,0,1)

        layout_status_6col.addWidget(QLabel("y"),0,2)
        layout_status_6col.addWidget(self.edit_pose_y,0,3)

        layout_status_6col.addWidget(QLabel("z"),0,4)
        layout_status_6col.addWidget(self.edit_pose_z,0,5)

        layout_status_6col.addWidget(QLabel("[ESTOP]"),1,0,1,6)

        layout_status_6col.addWidget(QLabel("sw"),2,0)
        layout_status_6col.addWidget(self.edit_sw_estop,2,1)

        layout_status_6col.addWidget(QLabel("hw"),2,2)
        layout_status_6col.addWidget(self.edit_hw_estop,2,3)

        layout_status_6col.addWidget(QLabel("pl"),2,4)
        layout_status_6col.addWidget(self.edit_payload_estop,2,5)

        fiducial_info = QWidget()
        layout_fiducial_info = QHBoxLayout(fiducial_info)
        layout_fiducial_info.addWidget(QLabel("Nearist QR : "))
        self.edit_qr = QLineEdit(readOnly=True)
        layout_fiducial_info.addWidget(self.edit_qr)

        layout_main = QVBoxLayout(self)
        layout_main.addWidget(status_4col)
        layout_main.addWidget(status_6col)
        layout_main.addWidget(fiducial_info)
        layout_main.addStretch(0)

    def update_ui(self):
        self.edit_power.setText(str(self.status.motor_power_state))
        self.edit_battery_amount.setText(str(self.status.percentage))
        self.edit_charging.setText(str(self.status.now_charging))
        x = round(self.status.pose['position']['x'],2)
        y = round(self.status.pose['position']['y'],2)
        z = round(self.status.pose['position']['z'],2)
        self.edit_pose_x.setText(str(x))
        self.edit_pose_y.setText(str(y))
        self.edit_pose_z.setText(str(z))
        self.edit_sw_estop.setText(str(self.status.sw_estop_state))
        self.edit_hw_estop.setText(str(self.status.hw_estop_state))
        self.edit_payload_estop.setText(str(self.status.payload_estop_state))
        self.edit_qr.setText(str(self.status.fiducial_info))
        self.edit_mode.setText(str(self.status.spot_mode))

        self.update()




        



        


if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    window = StatusView(400, 800)  # 원하는 크기 지정
    window.show()
    sys.exit(app.exec())

        