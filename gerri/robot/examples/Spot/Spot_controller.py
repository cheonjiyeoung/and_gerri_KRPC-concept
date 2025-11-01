
class SpotController:
    def __init__(self):
        pass

    def move(self,vx=0.0,vy=0.0,vth=0.0):
        print(f"Moving with velocities vx: {vx}, vy: {vy}, vth: {vth}")

    def move_waypoint(self,poi):
        print(f"Moving to waypoint: {poi}")

    def test_method(self):
        print("This is a test method in SpotController")

    def _some_private_method(self):
        print("This is a private method and should not be called directly.")