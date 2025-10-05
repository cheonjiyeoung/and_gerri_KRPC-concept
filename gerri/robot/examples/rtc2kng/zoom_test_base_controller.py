from pubsub import pub
import copy
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.function.manipulator_function import ManipulatorFunction
from gerri.robot.function.mobile_function import MobileFunction
from gerri.robot.status_manager import StatusManager

class ZoomBaseController(ManipulatorFunction,MobileFunction):
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.sub_controller = None

        self.zoom_level = 1.0


        if 'interface' in params:
            self.interface = params['interface']
            self.last_interface_value = copy.deepcopy(self.interface)
        else:
            self.interface = None
            self.last_interface_value = None

        pub.subscribe(self.receive_message,"receive_message")
        pub.subscribe(self.set_zoom_level, "zoom_level")

    def receive_message(self, message):
        ManipulatorFunction.receive_message(self,message=message)
        MobileFunction.receive_message(self,message=message)
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == self.interface.name:
                            self.interface.update(value)
                            print(value)
                            if self.interface.button_left_thumbstick:
                                pub.sendMessage('zoom_step_control', step=self.interface.left_axis_Y/10)

                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")

    def set_zoom_level(self, level):
        self.zoom_level = level
        print("zoom_level:", self.zoom_level)

    def send_message(self, message):
        pub.sendMessage('send_message', message=message)

    def connect(self):
        self.sub_controller.connect()
        self.status_manager = StatusManager(self.robot_info, self.sub_controller)

    def disconnect(self):
        self.sub_controller.disconnect()
