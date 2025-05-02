from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.examples.sample_operator import sample_base_command

class SampleBaseCommander:
    def __init__(self, robot_info, **params):
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']

        if 'commander' in params:
            self.commander = params['commander']
        else:
            self.commander = self._initialize_commander(**params)

        self.commander.set_base_commander(self)

        pub.subscribe(self.received_message, 'received_message')

    def _initialize_commander(self, **params):
        if self.robot_model == 'gerri':
            from gerri.operator.examples.sample_operator.sample_sub_commander import SampleSubCommander
            return SampleSubCommander()

        else:
            raise ValueError(f"Unsupported robot model: {self.robot_model}")

    def connect(self):
        self.commander.connect()

    def disconnect(self):
        self.commander.disconnect()

    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def received_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                target = message['target']

    def hello_universe(self, message):
        command = sample_base_command.hello_universe(message, target='all')
        self.send_message(command)
