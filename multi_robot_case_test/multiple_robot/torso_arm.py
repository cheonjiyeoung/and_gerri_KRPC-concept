
class TorsoArm:
    def __init__(self,idx):
        self.idx = idx

    def move(self,j1,j2,j3,j4,j5,j6):
        print(f"({self.idx})_Arm : Moving with joints j1={j1},j2={j2},j3={j3},j4={j4},j5={j5},j6={j6}")

    def some_manifulator_method(self,text):
        print(f"({self.idx})_Arm : some_manifulator_method : {text}")

    def move_preset(self,preset):
        print(f"({self.idx})_Arm : Moving to preset: {preset}")

    def home_pose(self):
        print(f"({self.idx})_Arm : go to home_pose")

