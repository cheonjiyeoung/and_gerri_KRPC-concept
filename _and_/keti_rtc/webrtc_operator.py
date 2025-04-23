from ketirtc_operator.RemoteRobotController import RemoteRobotController
from pubsub import pub

class WebRtcOperator:
    def __init__(self, ROBOT_INFO, OPERATOR_INFO=None):
        self.robot = None
        self.robot_id = ROBOT_INFO["id"]

        self.server_host = '175.126.123.199'
        self.server_port = 9980

        self.user_id = OPERATOR_INFO['id'] if OPERATOR_INFO else 'python_operator'

        pub.subscribe(self.send_message, 'send_message')

    def connect(self):
        self.robot = RemoteRobotController(self.robot_id, self.server_host, self.server_port, user_id=self.user_id)
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
        pub.sendMessage('receive_message', message=message)

