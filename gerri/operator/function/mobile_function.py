#mobile_function.py
from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.function import mobile_command

class MobileFunction:
    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def move(self,move, target='all'):
        command = mobile_command.move(move, target=target)

    def move_waypoint(self, waypoint, target='all'):
        command = mobile_command.move_waypoint(waypoint, target=target)
        self.send_message(command)

    def move_coord(self, coord, target='all'):
        command = mobile_command.move_coord(coord, target=target)
        self.send_message(command)

    def dock(self, docking_station, target='all'):
        command = mobile_command.dock(docking_station, target=target)
        self.send_message(command)

    def change_mode(self, mode, target='all'):
        command = mobile_command.change_mode(mode, target=target)
        self.send_message(command)

    def change_map(self, waypoint, target='all'):
        command = mobile_command.change_map(waypoint, target=target)
        self.send_message(command)

    def relocate(self, waypoint, target='all'):
        command = mobile_command.relocate(waypoint, target=target)
        self.send_message(command)