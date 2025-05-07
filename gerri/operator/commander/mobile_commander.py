from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.commander import mobile_command

class MobileCommander:
    def __init__(self, robot_info, **params):
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        if 'sub_commander' in params:
            self.sub_commander = params['sub_commander']
        else:
            self.sub_commander = self.init_sub_commander(**params)


        if hasattr(self.sub_commander, 'init_base_commander'):
            self.sub_commander.init_base_commander(self)

        pub.subscribe(self.receive_message, 'receive_message')

    def init_sub_commander(self, **params):
        if self.robot_model == 'gyd_mobile':
            # from gerri.operator.examples.piper.piper_commander import PiperCommander
            return "### TODO : ADD SUB COMMANDER"
        else:
            raise ValueError(f"Unsupported robot model: {self.robot_model}")

    def connect(self):
        self.sub_commander.connect()

    def disconnect(self):
        self.sub_commander.disconnect()

    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                target = message['target']

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
