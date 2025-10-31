from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

class PTZFunction:
    def receive_message(self, message):
        if 'topic' in message:
            topic = message['topic']
            value = message['value']
            option = message.get('option')
            if 'target' in message:
                if message['target'] in self.robot_id or message['target'] == 'all':
                    try:
                        if topic == 'pan_tilt':
                            self.sub_controller.pan_tilt(value)
                        elif topic == 'pan_tilt_step':
                            self.sub_controller.pan_tilt_step(value,option)
                        elif topic == 'pan_tilt_zoom':
                            self.sub_controller.pan_tilt_zoom(value,option)
                        elif topic == 'zoom':
                            self.sub_controller.zoom(value,option)
                    except AttributeError as e:
                        pass
                        # print(f"❌ Command '{topic}' not supported by controller: {e}")
                    except Exception as e:
                        print(f"❌ Error while handling topic '{topic}': {e}")