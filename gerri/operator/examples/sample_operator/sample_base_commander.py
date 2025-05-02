from pubsub import pub
import datetime

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.operator.examples.sample_operator import sample_base_command
from utils.time_sync_manager import time_sync

print(time_sync.timestamp())

def timestamp():
    return time_sync.timestamp()


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

        pub.subscribe(self.receive_message, 'receive_message')

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


    """
    Receives and handles messages from the robot.
    Specifically tracks robot status messages to calculate latency.
    """
    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if topic == 'robot_status':
                ts = value['metadata'].get('timestamp')
                time_sync.record_latency(ts)
                avg, worst = time_sync.get_latency_stats()
                print(f"📡 Ping: {avg:.2f} ms / 1% slow: {worst:.2f} ms")
            if 'target' in message:
                target = message['target']



    """
    Sends a "hello_universe" command message to all connected targets.
    """
    def hello_universe(self, message):
        command = sample_base_command.hello_universe(message, target='all')
        self.send_message(command)