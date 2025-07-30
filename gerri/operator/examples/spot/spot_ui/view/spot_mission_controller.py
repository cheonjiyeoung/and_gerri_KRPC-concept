import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.operator.examples.spot.spot_ui.utils.my_widgets import PubSubButton, PubSubDBSpinbox, PubSubComboBox, PubSubLineEdit
from PySide6.QtWidgets import QGridLayout, QPushButton, QLabel, QWidget, QComboBox, QLineEdit, QVBoxLayout
from pubsub import pub

class SpotMissionController(QWidget):
    def __init__(self,status,spot_info):
        super().__init__()

        layout_main = QVBoxLayout(self)
        self.status = status
        self.spot_info = spot_info

        mission_controller = QWidget()
        layout_mission_controller = QGridLayout(mission_controller)

        self.mission_status = None
        self.mission_tick = None

        self.cb_mission_list = PubSubComboBox("cb_mission_list")
        missions = spot_info["missions"]
        for mission in missions:
            self.cb_mission_list.addItem(mission)

        btn_mission_start = PubSubButton("Mission Start")
        btn_mission_pause = PubSubButton("Mission Pause")
        btn_mission_stop = PubSubButton("Mission Stop")
        btn_mission_resume = PubSubButton("Mission Restart")

        layout_mission_controller.addWidget(QLabel("[Spot Missions]"),0,0,1,2)
        layout_mission_controller.addWidget(QLabel("Mission :"),1,0)
        layout_mission_controller.addWidget(self.cb_mission_list,1,1)
        layout_mission_controller.addWidget(btn_mission_start,2,0)
        layout_mission_controller.addWidget(btn_mission_pause,2,1)
        layout_mission_controller.addWidget(btn_mission_stop,3,0)
        layout_mission_controller.addWidget(btn_mission_resume,3,1)
        
        mission_state = QWidget()
        layout_mission_state = QGridLayout(mission_state)
        layout_mission_state.addWidget(QLabel("[Mission State]"),0,0,1,2)
        layout_mission_state.addWidget(QLabel("Current_state : "),1,0)
        self.edit_mission_state = QLineEdit(self.mission_status,readOnly=True)
        layout_mission_state.addWidget(self.edit_mission_state,1,1)

        layout_mission_state.addWidget(QLabel("Tick : "),2,0)
        self.edit_mission_tick = QLineEdit(self.mission_tick,readOnly=True)
        layout_mission_state.addWidget(self.edit_mission_tick,2,1)

        layout_main.addWidget(mission_controller)
        layout_main.addWidget(mission_state)
        layout_main.addStretch(0)


    def update_ui(self):
        self.edit_mission_state.setText(str(self.status.spot_mission_status))
        self.edit_mission_tick.setText(str(self.status.spot_mission_tick))
        self.update()

