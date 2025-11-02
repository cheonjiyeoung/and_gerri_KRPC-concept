
class SpotController:
    def __init__(self):
        pass

    # method with default parameters example
    def move(self,vx=0.0,vy=0.0,vth=0.0):
        print(f"Moving with velocities vx: {vx}, vy: {vy}, vth: {vth}")

    # method with non-default parameters example
    def move_waypoint(self,poi):
        print(f"Moving to waypoint: {poi}")

    # method without parameters example
    def test_method(self):
        print("This is a test method in SpotController")

    # private method example (for commander_builder test)
    def _some_private_method(self):
        print("This is a private method and should not be called directly.")
