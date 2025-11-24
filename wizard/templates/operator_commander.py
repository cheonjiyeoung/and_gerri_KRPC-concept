from pubsub import pub
import os, sys
from and_gerri.gerri.utils.time_sync_manager import time_sync

def timestamp():
    return time_sync.timestamp()


class __COMMANDER__():
    def __init__(self, robot_info, operator_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.operator_info = operator_info

        self.sub_commander = None
        pub.subscribe(self.receive_message, 'receive_message')

    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    """
    Receives and handles messages from the robot.
    Specifically tracks robot status messages to calculate latency.
    """
    def receive_message(self, message):
        # print("status message:", message)
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if topic == "spot_question":
                pub.sendMessage('spot_question', value=value)
            if topic == 'robot_status':
                ts = value['metadata'].get('timestamp')
                time_sync.record_latency(ts)
                avg, worst = time_sync.get_latency_stats()
                # print(f"📡 Ping: {avg:.2f} ms / 1% slow: {worst:.2f} ms")
                pub.sendMessage('status_update', data=value)
            if 'target' in message:
                target = message['target']

    """
    Sends a "hello_universe" command message to all connected targets.
    """
    def hello_universe(self, target="all"):
        command = {"topic":"hello","value":"universe","target":target}
        self.send_message(command)