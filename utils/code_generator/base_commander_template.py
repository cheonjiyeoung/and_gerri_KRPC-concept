SCRIPT_HEADER = """\
from pubsub import pub
import datetime
import time
from utils.time_sync_manager import time_sync
print(time_sync.timestamp())

def timestamp():
    return time_sync.timestamp()

"""

CLASS_INIT = """\
    def __init__(self, robot_info, operator_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.operator_info = operator_info
        pub.subscribe(self.receive_message, 'receive_message')

"""

SEND_COMMAND_METHOD = """\
    def send_command(self, topic, value):
        command = {"topic": topic, "value": value}
        pub.sendMessage('send_command', command=command)

"""