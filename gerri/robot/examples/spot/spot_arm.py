from spot_controller import SpotController
from bosdyn.client.robot_command import RobotCommandBuilder

class SpotArm(SpotController):
    def __init__(self, spot_info):
        super().__init__(spot_info)

    def stow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

    def unstow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())