from pubsub import pub
import datetime
import time

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

# from gerri_spot import spot_base_command
from gerri.operator.function.manipulator_function import ManipulatorFunction
from gerri.operator.function.pan_tilt_zoom_function import PTZFunction
from utils.time_sync_manager import time_sync

print(time_sync.timestamp())

def timestamp():
    return time_sync.timestamp()


class PiperBaseCommander(ManipulatorFunction, PTZFunction):
    def __init__(self, robot_info, operator_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.operator_info = operator_info

        self.sub_commander = None
        pub.subscribe(self.receive_message, 'receive_message')

    def connect(self):
        self.sub_commander.connect()

    def disconnect(self):
        self.sub_commander.disconnect()

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
            if topic == 'robot_status':
                ts = value['metadata'].get('timestamp')
                time_sync.record_latency(ts)
                avg, worst = time_sync.get_latency_stats()
                # print(f"📡 Ping: {avg:.2f} ms / 1% slow: {worst:.2f} ms")
                self.sub_commander.get_status(value)
            if 'target' in message:
                target = message['target']

