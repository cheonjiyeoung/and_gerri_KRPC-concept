from ketirtc_operator.RemoteRobotController import RemoteRobotController
from pubsub import pub
import json

class WebRtcOperatorCommand:
    def __init__(self, robot_info, operator_info=None):
        self.robot = None
        self.robot_id = robot_info["id"] + '_command'

        self.server_host = '175.126.123.199'
        self.server_port = 9980

        self.user_id = operator_info['id']
        self.password = operator_info['password']

        pub.subscribe(self.send_message, 'send_message')

    def connect(self):
        self.robot = RemoteRobotController(self.robot_id, self.server_host, self.server_port, user_id=self.user_id, password=self.password)
        print(self.robot_id)
        self.robot.set_message_received_handler(self.handle_robot_message)
        try:
            self.robot.start_thread()
        except Exception as e:
            print(f"Error connect in while: {e}")

    def disconnect(self):
        self.robot.close()

    def send_message(self, message):
        self.robot.send_json(message)
        print(f"Sent message: {message}")

    async def handle_robot_message(self, message, channel):
        print(f"robot message received: {message}")

        # 안전하게 JSON 파싱
        if isinstance(message, str):
            try:
                message = json.loads(message)
            except json.JSONDecodeError as e:
                print(f"❌ Failed to parse JSON: {e}")
                return

        if not isinstance(message, dict):
            print(f"❌ Unexpected message format: {type(message)}")
            return

        pub.sendMessage('receive_message', message=message)
