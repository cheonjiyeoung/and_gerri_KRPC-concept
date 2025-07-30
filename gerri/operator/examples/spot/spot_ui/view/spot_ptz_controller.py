import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from PySide6.QtWidgets import QWidget,QVBoxLayout,QPushButton,QLabel,QLineEdit,QComboBox,QGridLayout,QDoubleSpinBox
from gerri.operator.examples.spot.spot_ui.utils.my_widgets import PubSubButton, PubSubDBSpinbox, PubSubComboBox, PubSubLineEdit

class SpotPTZController(QWidget):
    def __init__(self,w,h,status,spot_info):
        super().__init__()
        default_pt_relative_step = spot_info["default_pt_relative_step"]
        default_zoom_relative_step = spot_info["default_zoom_relative_step"]

        self.widget_width = w
        self.widget_height = h
        self.status = status

        self.cb_compositor = PubSubComboBox("cb_compositor")
        compositors = ["mech_full", "mech_full_ir", "c0", "c1", "c2", "c3", "c4","digi_full"]
        self.cb_compositor.addItems(compositors)
        btn_exec_set_compositor = PubSubButton("set_compositor")

        set_compositor = QWidget()
        layout_set_compositor = QGridLayout(set_compositor)
        layout_set_compositor.addWidget(QLabel("카메라 선택"),0,0)
        layout_set_compositor.addWidget(self.cb_compositor,1,0)
        layout_set_compositor.addWidget(btn_exec_set_compositor,1,1)

        self.edit_pan_pose = PubSubLineEdit("edit_pan_pose")
        self.edit_tilt_pose = PubSubLineEdit("edit_tilt_pose")
        self.edit_zoom_pose = PubSubLineEdit("edit_zoom_pose")
        btn_ptz_absolute = PubSubButton("ptz_absolute")

        self.edit_current_pan_pose = QLineEdit(readOnly=True)
        self.edit_current_tilt_pose = QLineEdit(readOnly=True)
        self.edit_current_zoom_pose = QLineEdit(readOnly=True)
        
        btn_tilt_up = PubSubButton("Tilt UP")
        btn_tilt_down = PubSubButton("Tilt UP")
        btn_pan_left = PubSubButton("Pan LEFT")
        btn_pan_right = PubSubButton("Pan RIGHT")


        self.spin_pt_step = PubSubDBSpinbox("spin_pt_step")
        self.spin_pt_step.setRange(0.1, 10.0)
        self.spin_pt_step.setSingleStep(0.1)
        self.spin_pt_step.setDecimals(1)
        self.spin_pt_step.setValue(default_pt_relative_step)

        self.spin_zoom_step = PubSubDBSpinbox("spin_zoom_step")
        self.spin_zoom_step.setRange(-1.0, 1.0)
        self.spin_zoom_step.setSingleStep(0.1)
        self.spin_zoom_step.setDecimals(1)
        self.spin_zoom_step.setValue(default_zoom_relative_step)    

        btn_zoom = PubSubButton("Zoom")

        ptz_step = QWidget()
        layout_ptz_step = QGridLayout(ptz_step)
        layout_ptz_step.addWidget(QLabel("Pan Tilt step 이동"),0,0,1,3)
        layout_ptz_step.addWidget(QLabel(""),1,0)
        layout_ptz_step.addWidget(btn_tilt_up,1,1)
        layout_ptz_step.addWidget(QLabel(""),1,2)
        layout_ptz_step.addWidget(btn_pan_left,2,0)
        layout_ptz_step.addWidget(btn_tilt_down,2,1)
        layout_ptz_step.addWidget(btn_pan_right,2,2)
        layout_ptz_step.addWidget(QLabel("Step :"),3,0)
        layout_ptz_step.addWidget(self.spin_pt_step,3,1)
        layout_ptz_step.addWidget(QLabel(""),4,0)
        layout_ptz_step.addWidget(QLabel("Zoom 이동"),5,0)
        layout_ptz_step.addWidget(QLabel("Step :"),6,0)
        layout_ptz_step.addWidget(self.spin_zoom_step,6,1)
        layout_ptz_step.addWidget(btn_zoom,6,2)
        

        ptz_absolute = QWidget()
        layout_ptz_absolute = QGridLayout(ptz_absolute)
        layout_ptz_absolute.addWidget(QLabel("절댓값 PTZ_이동"),0,0,1,6)
        layout_ptz_absolute.addWidget(QLabel("P :"),1,0)
        layout_ptz_absolute.addWidget(self.edit_pan_pose,1,1)
        layout_ptz_absolute.addWidget(QLabel("T :"),1,2)
        layout_ptz_absolute.addWidget(self.edit_tilt_pose,1,3)
        layout_ptz_absolute.addWidget(QLabel("Z :"),1,4)
        layout_ptz_absolute.addWidget(self.edit_zoom_pose,1,5)
        layout_ptz_absolute.addWidget(btn_ptz_absolute,2,0,1,6)
        layout_ptz_absolute.addWidget(QLabel("현재 PTZ 위치"),3,0,1,6)
        layout_ptz_absolute.addWidget(QLabel("P :"),4,0)
        layout_ptz_absolute.addWidget(self.edit_current_pan_pose,4,1)
        layout_ptz_absolute.addWidget(QLabel("T :"),4,2)
        layout_ptz_absolute.addWidget(self.edit_current_tilt_pose,4,3)
        layout_ptz_absolute.addWidget(QLabel("Z :"),4,4)
        layout_ptz_absolute.addWidget(self.edit_current_zoom_pose,4,5)

        btn_acquire_frame = PubSubButton("acquire_frame")


        focus = QWidget()
        layout_focus = QGridLayout(focus)
        layout_focus.addWidget(QLabel("[Focus]"),0,0,1,4)

        layout_focus.addWidget(QLabel("Mode : "),1,0)
        self.edit_focus_mode = QLineEdit(readOnly=True)
        layout_focus.addWidget(self.edit_focus_mode,1,1)

        layout_focus.addWidget(QLabel("Dist : "),1,2)
        self.edit_focus_dist = QLineEdit(readOnly=True)
        layout_focus.addWidget(self.edit_focus_dist,1,3)

        layout_focus.addWidget(QLabel("Pose : "),1,4)
        self.edit_focus_pose = QLineEdit(readOnly=True)
        layout_focus.addWidget(self.edit_focus_pose,1,5)

        # 레이아웃 병합
        layout_full = QVBoxLayout(self)
        layout_full.addWidget(set_compositor)
        layout_full.addWidget(ptz_step)
        layout_full.addWidget(ptz_absolute)
        layout_full.addWidget(btn_acquire_frame)
        layout_full.addWidget(focus)
        layout_full.addStretch(0)
        layout_full.setContentsMargins(0, 0, 0, 0)

    def update_ui(self):
        self.edit_current_pan_pose.setText(str(self.status.ptz_pan))
        self.edit_current_tilt_pose.setText(str(self.status.ptz_tilt))
        self.edit_current_zoom_pose.setText(str(self.status.ptz_zoom))
        self.edit_focus_mode.setText(str(self.status.ptz_focus_mode))
        self.edit_focus_dist.setText(str(self.status.ptz_focus_distance))
        self.edit_focus_pose.setText(str(self.status.ptz_focus_position))
        self.update()

if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    window = SpotPTZController(400, 800)  # 원하는 크기 지정
    window.show()
    sys.exit(app.exec())

        