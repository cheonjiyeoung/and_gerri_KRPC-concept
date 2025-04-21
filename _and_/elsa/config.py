from typing import Dict, Any
from dataclasses import dataclass, field

@dataclass
class MQTTConfig:
    BROKER: str = '125.131.105.165'
    PORT: int = 1883
    USERNAME: str = 'keti'
    PASSWORD: str = '3mbg6pocuvd8mr3u'
    RECONNECT_DELAY: int = 1
    RECONNECT_RATE: int = 2
    MAX_RECONNECT_COUNT: int = 20
    MAX_RECONNECT_DELAY: int = 60

@dataclass
class Config:
    mqtt: MQTTConfig = MQTTConfig()
    # Topic templates
    MQTT_TOPICS: Dict[str, str] = field(default_factory=lambda: {
        'system_status': '/elsa/{elsa_id}/system/status',
        'call_response': '/elsa/{elsa_id}/elevator/{elevator_id}/robot/{robot_id}/call/response',
        'state_response': '/elsa/{elsa_id}/elevator/{elevator_id}/robot/{robot_id}/state/response',
        'door': '/elsa/{elsa_id}/elevator/{elevator_id}/door',
        'floor': '/elsa/{elsa_id}/elevator/{elevator_id}/floor',
        'arrival': '/elsa/{elsa_id}/elevator/{elevator_id}/robot/{robot_id}/arrival',
        'exception': '/elsa/{elsa_id}/elevator/{elevator_id}/robot/{robot_id}/exception',
        'queue_empty': '/elsa/{elsa_id}/elevator/{elevator_id}/queue/empty/response',
        'queue': '/elsa/{elsa_id}/elevator-group/{elevator_id}/queue'
    })

config = Config() 