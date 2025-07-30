from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.controller.manipulator_controller import ManipulatorController
from gerri.robot.controller.mobile_controller import MobileController
from gerri.robot.examples.spot.spot_sub_controller import SpotSubController
from gerri.robot.status_manager import StatusManager

class SpotBaseController(MobileController,ManipulatorController):
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.sub_controller:SpotSubController = None
        pub.subscribe(self.receive_message,"receive_message")

    def receive_message(self, message):
        print(message)
        MobileController.receive_message(self,message=message)
        ManipulatorController.receive_message(self,message=message)
        if 'topic' in message:
            topic = message['topic']
            value = message.get('value')
            option = message.get('option')
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == 'walk':
                            self.sub_controller.walk(value=value,option=option)
                        if topic == 'stand':
                            print("!!!!!!!!!!!!!!!!!!")
                            self.sub_controller.stand(value=value,option=option)
                        if topic == 'sit':
                            self.sub_controller.sit(value=value,option=option)
                        if topic == 'action_stop':
                            self.sub_controller.action_stop(value=value,option=option)
                        if topic == 'run_mission':
                            self.sub_controller.run_mission(value=value,option=option)
                        if topic == 'stop_mission':
                            self.sub_controller.stop_mission(value=value,option=option)
                        if topic == 'pause_mission':
                            self.sub_controller.pause_mission(value=value,option=option)
                        if topic == 'resume_mission':
                            self.sub_controller.resume_mission(value=value,option=option)
                        if topic == 'spot_ptz_absolute':
                            self.sub_controller.spot_ptz_absolute(value=value,option=option)
                        if topic == 'spot_ptz_initialize':
                            self.sub_controller.spot_ptz_initialize(value=value,option=option)
                        if topic == 'spot_ptz_set_focus':
                            self.sub_controller.spot_ptz_set_focus(value=value,option=option)
                        if topic == 'spot_ptz_set_focus_relative':
                            self.sub_controller.spot_ptz_set_focus_relative(value=value,option=option)
                        if topic == 'set_compositor':
                            self.sub_controller.set_compositor(value=value,option=option)
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
