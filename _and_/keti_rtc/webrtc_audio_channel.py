import asyncio
import json
import time

from ketirtc_robot import UserClient, ConnectionChannel
from ketirtc_robot.camera.cam_dummy import DummyCamera
from ketirtc_robot.connection_manager import UserClientConnectionManager
from ketirtc_robot.webrtc.webrtc_connection_manager import WebRtcConnectionManager
from ketirtc_robot.microphone.pyaudio_mic import PyAudioMicrophone
from ketirtc_robot.audio.MultipleAudioTrackPlayer import MultipleAudioTrackPlayer


import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from gerri.robot.gerri_config import ROBOT_ID, AUDIO_INPUT, AUDIO_OUTPUT


class WebRtcAudioChannel:
    def __init__(self, robot_id, password, server_host, server_port, audio_input=AUDIO_INPUT, audio_output=AUDIO_OUTPUT):
        """
        WebRTC 기반 원격 오디오 스트리밍 및 메시지 처리를 담당하는 클래스
        """
        self.robot_id = robot_id
        self.password = password
        self.server_host = server_host
        self.server_port = server_port

        self.camera = DummyCamera()

        mic_config = {
            "device_name": audio_input,
        }
        self.microphone = PyAudioMicrophone(mic_config)

        speaker_config = {
            "device_name": audio_output,
        }
        self.audio_recorder = MultipleAudioTrackPlayer(config=speaker_config)

        self.signalling_server = WebRtcConnectionManager(
            robot_id=self.robot_id,
            signalling_password=self.password,
            camera=self.camera,
            microphone=self.microphone,
            audio_recorder=self.audio_recorder,
            signalling_server_host=self.server_host,
            signalling_server_port=self.server_port
        )

        self.user_clients_manager = UserClientConnectionManager()
        self.user_clients_manager.add_connection_manager_channel(self.signalling_server)

        self.user_clients_manager.add_listener('user_connected', self.user_connected_handler)
        self.user_clients_manager.add_listener('user_disconnected', self.user_disconnected_handler)
        self.user_clients_manager.add_listener('user_message', self.user_message_handler)

        print("[✅] WebRTC Audio Streamer Initialized.")

    def user_connected_handler(self, usr_client: UserClient):
        print(f"[+] WebRTC Audio Streamer User Connected: {usr_client.client_id}")
        print(f"Active Users: {self.user_clients_manager.clients_count}")

    def user_disconnected_handler(self, usr_client: UserClient):
        print(f"[-] WebRTC Audio Streamer User Disconnected: {usr_client.client_id}")
        print(f"Active Users: {self.user_clients_manager.clients_count}")

    def user_message_handler(self, message, channel: ConnectionChannel, usr_client: UserClient):
        try:
            msg_data = json.loads(message)
            print(f"[📩] Message from {usr_client.client_id} via {channel.channel_name}: {msg_data}")
        except json.JSONDecodeError:
            print(f"[⚠️] Invalid JSON Message Received: {message}")

    def connect(self):
        """WebRTC 오디오 연결을 시작합니다."""
        self.user_clients_manager.start()
        print("[🔄] WebRTC Audio Streamer Running...")

    def disconnect(self):
        """WebRTC 오디오 연결을 해제합니다."""
        self.user_clients_manager.stop()
        print("[✅] WebRTC Audio Streamer Stopped.")


if __name__ == "__main__":
    robot = WebRtcAudioChannel(
        robot_id=ROBOT_ID + "_audio",
        password="",
        server_host="175.126.123.199",
        server_port="8180"
    )
    robot.connect()
    # 필요에 따라 disconnect()를 호출하여 연결 해제할 수 있습니다.
