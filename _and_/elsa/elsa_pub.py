import random
import time
import paho.mqtt.client as mqtt

from typing import Optional, Dict, Any
import os,sys
CURRENT_FILE = os.path.abspath(__file__)
VENV_DIR = os.path.dirname(sys.executable)
PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
sys.path.insert(0, PROJECT_ROOT)
from avatar_darm.robot.network_tools.elsa.config import config
import json

class ELSA_PUB(mqtt.Client):
    def __init__(self,
                 elevator_id: int,
                 robot_id: str = 'dummy',
                 elsa_id: int = 1):
        super().__init__(mqtt.CallbackAPIVersion.VERSION1)
        
        self.ev_id = elevator_id
        self.elsa_id = elsa_id
        self.robot_id = robot_id
        
        # MQTT configuration
        self.broker = config.mqtt.BROKER
        self.port = config.mqtt.PORT
        self.id = f'keti-mqtt-{random.randint(0, 1000)}'
        
        # Set up connection parameters
        self.username_pw_set(config.mqtt.USERNAME, config.mqtt.PASSWORD)
        
        # Set up callbacks
        self.on_connect = self.connect_handler
        self.on_disconnect = self.disconnect_handler
        
        # Connect to broker
        self.connect_mqtt()

    def connect_mqtt(self) -> None:
        """Connect to the MQTT broker."""
        try:
            self.connect(self.broker, self.port)
            print(f"Connecting to MQTT broker at {self.broker}:{self.port}")
        except Exception as e:
            print(f"Failed to connect to MQTT broker: {e}")

    def connect_handler(self, client: mqtt.Client, userdata: Any, flags: Dict[str, Any], rc: int) -> None:
        """Handle MQTT connection events."""
        if rc == 0 and client.is_connected():
            print("MQTT broker connected successfully")
        else:
            print(f"Failed to connect to MQTT broker, return code: {rc}")

    def disconnect_handler(self, client: mqtt.Client, userdata: Any, rc: int) -> None:
        """Handle MQTT disconnection events."""
        reconnect_count = 0
        reconnect_delay = config.mqtt.RECONNECT_DELAY
        
        while reconnect_count < config.mqtt.MAX_RECONNECT_COUNT:
            time.sleep(reconnect_delay)
            
            try:
                client.reconnect()
                print("Successfully reconnected to MQTT broker")
                return
            except Exception as err:
                print(f"Reconnection attempt {reconnect_count + 1} failed: {err}")
            
            reconnect_delay *= config.mqtt.RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, config.mqtt.MAX_RECONNECT_DELAY)
            reconnect_count += 1
            
        print("Maximum reconnection attempts reached. Giving up.")
        client.flagexit = True

    def ev_call(self,f,t):
        topic = f"/elsa/1/elevator/{self.ev_id}/robot/{self.robot_id}/call"

        msg_dict = {
            'id': "ketitest",
            'type': 0,
            'origin':f,
            'destination':t
        }
        msg = json.dumps(msg_dict)
        result = self.publish(topic,msg)  

    def ev_in_wait(self):
        topic = f"/elsa/1/elevator/{self.ev_id}/robot/{self.robot_id}/state"
        msg_dict = {
            'id': "ketitest",
            'state': 0
        }
        msg = json.dumps(msg_dict)
        result = self.publish(topic,msg)  
    def ev_in(self):
        topic = f"/elsa/1/elevator/{self.ev_id}/robot/{self.robot_id}/state"
        msg_dict = {
            'id': "ketitest",
            'state': 1
        }
        msg = json.dumps(msg_dict)
        result = self.publish(topic,msg)  
    def ev_out_wait(self):
        topic = f"/elsa/1/elevator/{self.ev_id}/robot/{self.robot_id}/state"
        msg_dict = {
            'id': "ketitest",
            'state': 3
        }
        msg = json.dumps(msg_dict)
        result = self.publish(topic,msg)  
    def ev_out(self):
        topic = f"/elsa/1/elevator/{self.ev_id}/robot/{self.robot_id}/state"
        msg_dict = {
            'id': "ketitest",
            'state': 4
        }
        msg = json.dumps(msg_dict)
        result = self.publish(topic,msg)  

if __name__ == '__main__':
    broker = ELSA_PUB(elevator_id=1)
    broker.loop_forever()