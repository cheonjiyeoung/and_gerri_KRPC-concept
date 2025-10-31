from pubsub import pub
import copy

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.zimmer.zimmer_controller import ZimmerController
from gerri.robot.interface.vr_controller import VRController

class ZimmerBaseController():
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.sub_controller = ZimmerController(robot_info['ip'], robot_info['port'])
        self.last_gripper_value = 0

        if 'interface' in params:
            self.interface:VRController = params['interface']
            self.last_interface_value = copy.deepcopy(self.interface)
        else:
            self.interface = None
            self.last_interface_value = None

        pub.subscribe(self.receive_message,"receive_message")

    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == self.interface.name:
                            self.interface.update(value)
                            right_trigger_value = round(self.interface.right_trigger * 100, -1)
                            print(self.interface.right_trigger, right_trigger_value)
                            self.sub_controller.move_to_percentage(right_trigger_value, sync=False)
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

    def disconnect(self):
        self.sub_controller.disconnect()
