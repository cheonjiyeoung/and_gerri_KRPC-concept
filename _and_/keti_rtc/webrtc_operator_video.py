# from ketirtc_operator.RemoteRobotController import RemoteRobotController
from pubsub import pub

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.keti_rtc.webrtc_operator_video_track import VideoReceiverTrack, video_receiver
from _and_.keti_rtc.remote_robot_controller import RemoteRobotController

class WebRtcOperatorVideo:
    def __init__(self, robot_info, operator_info=None, video_info=None):
        self.robot = None
        self.robot_id = robot_info["id"]+ "_" + video_info

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
        pub.sendMessage('receive_message', message=message)
