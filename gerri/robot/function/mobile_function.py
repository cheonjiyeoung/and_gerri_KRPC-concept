from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

class MobileFunction:
    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            option = message.get("option")
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == 'change_mode':
                            self.sub_controller.change_mode(value,option)
                        elif topic == 'move_waypoint':
                            self.sub_controller.move_waypoint(value,option)
                        elif topic == 'move_coord':
                            self.sub_controller.move_coord(value,option)
                        elif topic == 'dock':
                            self.sub_controller.dock(value,option)
                        elif topic == 'move':
                            self.sub_controller.move(value,option)
                        elif topic == 'relocate':
                            self.sub_controller.relocate(value,option)
                        elif topic == 'move_floor':
                            self.sub_controller.move_floor(value,option)
                        elif topic == 'map_change':
                            self.sub_controller.map_change(value,option)
                    except AttributeError as e:
                        print(f"❌ Controller does not support topic '{topic}': {e}")
                    except Exception as e:
                        print(f"❌ Error processing topic '{topic}': {e}")