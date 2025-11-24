from pubsub import pub
import os, sys
from and_gerri.gerri.utils.time_sync_manager import time_sync

def timestamp():
    return time_sync.timestamp()


class GydCommander():
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
#################################################################
# Auto-generated methods by sub_controller.py
#################################################################

    def move_forward(self):
        topic = "move_forward"
        value = {}
        self.send_message(topic=topic, value=value)

    def move_backward(self):
        topic = "move_backward"
        value = {}
        self.send_message(topic=topic, value=value)

    def shift_left(self):
        topic = "shift_left"
        value = {}
        self.send_message(topic=topic, value=value)

    def shift_right(self):
        topic = "shift_right"
        value = {}
        self.send_message(topic=topic, value=value)

    def turn_left(self):
        topic = "turn_left"
        value = {}
        self.send_message(topic=topic, value=value)

    def turn_right(self):
        topic = "turn_right"
        value = {}
        self.send_message(topic=topic, value=value)

    def stop(self):
        topic = "stop"
        value = {}
        self.send_message(topic=topic, value=value)

    def estop(self):
        topic = "estop"
        value = {}
        self.send_message(topic=topic, value=value)

    def dock(self):
        topic = "dock"
        value = {}
        self.send_message(topic=topic, value=value)

    def undock(self):
        topic = "undock"
        value = {}
        self.send_message(topic=topic, value=value)

    def set_auto_mode(self):
        topic = "set_auto_mode"
        value = {}
        self.send_message(topic=topic, value=value)

    def set_teleop_mode(self):
        topic = "set_teleop_mode"
        value = {}
        self.send_message(topic=topic, value=value)

    def move_to_waypoint(self, name):
        topic = "move_to_waypoint"
        value = {}
        value["name"] = name
        self.send_message(topic=topic, value=value)

    def move_to_goal(self, x, y, theta):
        topic = "move_to_goal"
        value = {}
        value["x"] = x
        value["y"] = y
        value["theta"] = theta
        self.send_message(topic=topic, value=value)

    def increase_linear_speed(self):
        topic = "increase_linear_speed"
        value = {}
        self.send_message(topic=topic, value=value)

    def decrease_linear_speed(self):
        topic = "decrease_linear_speed"
        value = {}
        self.send_message(topic=topic, value=value)

    def increase_angular_speed(self):
        topic = "increase_angular_speed"
        value = {}
        self.send_message(topic=topic, value=value)

    def decrease_angular_speed(self):
        topic = "decrease_angular_speed"
        value = {}
        self.send_message(topic=topic, value=value)

    def reset_speed_to_default(self):
        topic = "reset_speed_to_default"
        value = {}
        self.send_message(topic=topic, value=value)

