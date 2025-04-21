import random
import time
import paho.mqtt.client as mqtt
import json
from pubsub import pub
from typing import Optional, Dict, Any
import os,sys
CURRENT_FILE = os.path.abspath(__file__)
VENV_DIR = os.path.dirname(sys.executable)
PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
sys.path.insert(0, PROJECT_ROOT)
from avatar_darm.robot.network_tools.elsa.config import config
# operator side

"""
ip : 172.20.1.161 (내부) / 10.17.7.198 (외부) 
port : 1883
username : keti
password : 3mbg6pocuvd8mr3u

elsa id : 1 (인승용)
elevator id : 1 (인승용) / 2 (화물용)
"""


class ELSA_SUB(mqtt.Client):
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
        self.on_message = self.message_handler
        self.on_disconnect = self.disconnect_handler
        
        # Connect to broker
        self.connect_mqtt()
        
        # Subscribe to topics
        self._subscribe_to_topics()

    def _subscribe_to_topics(self) -> None:
        """Subscribe to all relevant MQTT topics."""
        topics = {
            'system_status': config.MQTT_TOPICS['system_status'].format(elsa_id=self.elsa_id),
            'call_response': config.MQTT_TOPICS['call_response'].format(
                elsa_id=self.elsa_id,
                elevator_id=self.ev_id,
                robot_id=self.robot_id
            ),
            'state_response': config.MQTT_TOPICS['state_response'].format(
                elsa_id=self.elsa_id,
                elevator_id=self.ev_id,
                robot_id=self.robot_id
            ),
            'door': config.MQTT_TOPICS['door'].format(
                elsa_id=self.elsa_id,
                elevator_id=self.ev_id
            ),
            'floor': config.MQTT_TOPICS['floor'].format(
                elsa_id=self.elsa_id,
                elevator_id=self.ev_id
            ),
            'arrival': config.MQTT_TOPICS['arrival'].format(
                elsa_id=self.elsa_id,
                elevator_id=self.ev_id,
                robot_id=self.robot_id
            ),
            'exception': config.MQTT_TOPICS['exception'].format(
                elsa_id=self.elsa_id,
                elevator_id=self.ev_id,
                robot_id=self.robot_id
            ),
            'queue_empty': config.MQTT_TOPICS['queue_empty'].format(
                elsa_id=self.elsa_id,
                elevator_id=self.ev_id
            ),
            'queue': config.MQTT_TOPICS['queue'].format(
                elsa_id=self.elsa_id,
                elevator_id=self.ev_id
            )
        }
        
        for topic in topics.values():
            self.subscribe(topic)

    def connect_mqtt(self) -> None:
        """Connect to the MQTT broker."""
        try:
            self.connect(self.broker, self.port)
        except Exception as e:
            raise

    def message_handler(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
        """Handle incoming MQTT messages."""
        try:
            str_msg = str(msg.payload.decode("utf-8"))
            topic = msg.topic
            dict_msg = json.loads(str_msg)
            
            if "/floor" in topic:
                self._handle_floor_message(dict_msg)
            elif "call/response" in topic:
                self._handle_call_response(dict_msg)
            elif "/arrival" in topic:
                self._handle_arrival_message(dict_msg)
            elif "/door" in topic:
                self._handle_door_message(dict_msg)
            elif "/queue/empty/response" in topic:
                self._handle_queue_empty_message(dict_msg)
            else:
                print(f"Unhandled topic: {topic}, Message: {dict_msg}")
                
        except json.JSONDecodeError:
            print(f"Failed to decode JSON message: {str_msg}")
        except Exception as e:
            print(f"Error handling message: {e}")

    def _handle_floor_message(self, msg: Dict[str, Any]) -> None:
        """Handle floor-related messages."""
        current_floor = msg['floor']
        msg = {"topic":"/floor","value":msg}
        message = f"[ev_id : {self.ev_id}] current_floor = {current_floor}"
        pub.sendMessage("ev_msg", msg=msg)

    def _handle_call_response(self, msg: Dict[str, Any]) -> None:
        """Handle call response messages."""
        if msg['result'] == 0:
            message = f"[ev_id : {self.ev_id}] call success"
        else:
            message = f"[ev_id : {self.ev_id}] call fail.. (code : {msg['result']})"
        pub.sendMessage("ev_msg", msg=msg)

    def _handle_arrival_message(self, msg: Dict[str, Any]) -> None:
        """Handle arrival messages."""
        code = msg['type']
        current_floor = msg['floor']
        
        if code == 0:
            message = f"[ev_id : {self.ev_id}] from_floor arrived (current_floor = {current_floor})"
        else:
            message = f"[ev_id : {self.ev_id}] to_floor arrived (current_floor = {current_floor})"
            
        pub.sendMessage("ev_msg", msg=msg)

    def _handle_door_message(self, msg: Dict[str, Any]) -> None:
        """Handle door state messages."""
        state = msg['state']
        state_messages = {
            0: "door opened",
            1: "door oppenning...",
            2: "door closed",
            3: "door closing..."
        }
        
        if state in state_messages:
            message = f"[ev_id : {self.ev_id}] {state_messages[state]}"
            msg = {"topic":"/door","value":msg}
            pub.sendMessage("ev_msg", msg=msg)

    def _handle_queue_empty_message(self, msg: Dict[str, Any]) -> None:
        """Handle queue empty response messages."""
        result = msg['result']

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
                return
            except Exception as err:
                print(f"Reconnection attempt {reconnect_count + 1} failed: {err}")
            
            reconnect_delay *= config.mqtt.RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, config.mqtt.MAX_RECONNECT_DELAY)
            reconnect_count += 1
            
        print("Maximum reconnection attempts reached. Giving up.")
        client.flagexit = True

if __name__ == '__main__':
    broker = ELSA_SUBSCRIBER(elevator_id=1)
    broker.loop_forever()