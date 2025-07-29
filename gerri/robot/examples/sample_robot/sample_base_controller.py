from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.function.manipulator_function import ManipulatorFunction
from gerri.robot.function.mobile_function import MobileFunction
from gerri.robot.status_manager import StatusManager

class SampleBaseController(ManipulatorFunction,MobileFunction):
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.sub_controller = None

    def receive_message(self, message):
        ManipulatorFunction.receive_message(self,message=message)
        MobileFunction.receive_message(self,message=message)
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == 'hello_universe':
                            self.sub_controller.hello_universe(value)
                        else:
                            self.send_message({'topic': 'callback_'+topic, 'value': 'callback_'+value, 'target': 'all'})
                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")


    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def connect(self):
        self.sub_controller.connect()
        self.status_manager = StatusManager(self.robot_info, self.sub_controller)

    def disconnect(self):
        self.sub_controller.disconnect()
