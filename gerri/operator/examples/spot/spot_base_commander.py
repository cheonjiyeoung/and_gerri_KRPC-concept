from pubsub import pub
import datetime
import time

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

# from gerri_spot import spot_base_command
from gerri.operator.function.manipulator_function import ManipulatorFunction
from gerri.operator.function.mobile_function import MobileFunction
from gerri.operator.examples.spot import spot_base_command
from gerri.operator.examples.sample_operator.sample_base_command import hello_universe
from utils.time_sync_manager import time_sync

print(time_sync.timestamp())

def timestamp():
    return time_sync.timestamp()


class SpotBaseCommander(ManipulatorFunction, MobileFunction):
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

    def walk(self,value=None,option=None):
        command = spot_base_command.walk(value=value, target=self.robot_id, option=option)
        self.send_message(command)

    def stand(self,value=None,option=None):
        command = spot_base_command.stand(value=value, target=self.robot_id, option=option)
        self.send_message(command)

    def sit(self,value=None,option=None):
        command = spot_base_command.sit(value=value, target=self.robot_id, option=option)
        self.send_message(command)
    
    def action_stop(self,value=None,option=None):
        command = spot_base_command.action_stop(value=value, target=self.robot_id, option=option)
        self.send_message(command)   

    def ptz_relative(self,value=None,option=None):
        command = spot_base_command.ptz_relative(value=value,target=self.robot_id, option=option)
        self.send_message(command)

    def set_main_cam(self,value=None,option=None):
        command = spot_base_command.set_compositor(value=value,target=self.robot_id, option=option)
        self.send_message(command)

    def ptz_absolute(self,value=None,option=None):
        command = spot_base_command.ptz_absolute(value=value,target=self.robot_id, option=option)
        self.send_message(command)

    def run_mission(self,value=None,option=None):
        command = spot_base_command.run_mission(value=value,target=self.robot_id, option=option)
        self.send_message(command)

    def stop_mission(self,value=None,option=None):
        command = spot_base_command.stop_mission(value=value,target=self.robot_id, option=option)
        self.send_message(command)

    def pause_mission(self,value=None,option=None):
        command = spot_base_command.stop_mission(value=value,target=self.robot_id, option=option)
        self.send_message(command)

    def restart_mission(self,value=None,option=None):
        command = spot_base_command.restart_mission(value=value,target=self.robot_id, option=option)
        self.send_message(command)

    def dock(self,value=None,option=None):
        command = spot_base_command.dock(value=value,target=self.robot_id, option=option)
        self.send_message(command)

    def undock(self,value=None,option=None):
        command = spot_base_command.dock(value=True,target=self.robot_id, option=option)
        self.send_message(command)
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
                pub.sendMessage('status_update', data=value)
            if 'target' in message:
                target = message['target']



    """
    Sends a "hello_universe" command message to all connected targets.
    """
    def hello_universe(self, message):
        command = hello_universe(message, target='all')
        self.send_message(command)