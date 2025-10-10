from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.function.mobile_function import MobileFunction
from gerri.robot.status_manager import StatusManager

class GydBaseController(MobileFunction):
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info['id']
        self.robot_category = robot_info['category']
        self.robot_model = robot_info['model']
        self.sub_controller = None
        pub.subscribe(self.receive_message, "receive_message")

    def receive_message(self, message):
        MobileFunction.receive_message(self,message=message)
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            option = message.get("option")
            if topic == '/joy':
                self.sub_controller.handle_joy_command(value=value)
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == 'hello_universe':
                            self.sub_controller.hello_universe(value)
                        if topic == 'mode_auto':
                            self.sub_controller.mode_auto()
                        if topic == 'mode_teleop':
                            self.sub_controller.mode_teleop()
                        if topic == 'stop':
                            self.sub_controller.stop()
                        if topic == 'nav_pause':
                            self.sub_controller.nav_pause()
                        if topic == 'nav_cancel':
                            self.sub_controller.nav_cancel()
                        if topic == 'nav_resume':
                            self.sub_controller.nav_resume()
                        if topic == 'reloc':
                            self.sub_controller.reloc(value=value,option=option)
                        ### 수정 시작
                        if topic == 'turn_on_power':
                            self.sub_controller.turn_on_power()
                        if topic == 'turn_off_power':
                            self.sub_controller.turn_off_power()   
                        if topic == 'move_waypoint_async':
                            self.sub_controller.move_waypoint_async(value=value,option=option) 
                        if topic == "demo_coffee":
                            self.sub_controller.demo_coffee(value)
                        ### 수정 완료
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
