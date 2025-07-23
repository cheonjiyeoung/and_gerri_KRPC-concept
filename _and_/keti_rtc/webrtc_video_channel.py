import json
import asyncio
import time
import os
import sys
from asyncio import AbstractEventLoop
from ketirtc_robot import UserClient, ConnectionChannel
from ketirtc_robot.connection_manager import UserClientConnectionManager
from ketirtc_robot.webrtc.webrtc_connection_manager import WebRtcConnectionManager
from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from _and_.keti_rtc.cv2_camera_new import CV2StreamCamera


class WebRtcVideoChannel:
    def __init__(self, robot_id, password, api_key, server_host, server_port, camera, robot_group_id=None,
                 loop: AbstractEventLoop = None, message_duration_time=100):
        """
        WebRTC 기반 원격 스트리밍 및 메시지 처리를 담당하는 클래스
        """
        self.loop = loop or asyncio.get_event_loop()
        self.robot_id = robot_id
        self.password = password
        self.api_key = api_key
        self.server_host = server_host
        self.server_port = server_port
        self.message_duration_time = message_duration_time
        self.last_message_time = time.time()
        self.camera = camera
        self.signalling_server = WebRtcConnectionManager(
            robot_id=self.robot_id,
            signalling_password=self.password,
            camera=self.camera,
            signalling_server_host=self.server_host,
            signalling_server_port=self.server_port,
            is_control_channel=False,
            robot_group_id=robot_group_id,
            api_key = self.api_key
        )

        self.user_clients_manager = UserClientConnectionManager()
        self.user_clients_manager.add_connection_manager_channel(self.signalling_server)

        self.user_clients_manager.add_listener('user_connected', self.user_connected_handler)
        self.user_clients_manager.add_listener('user_disconnected', self.user_disconnected_handler)
        self.user_clients_manager.add_listener('user_message', self._remote_message_received)

        print("[✅] WebRTC Camera Streamer Initialized.")

    def user_connected_handler(self, usr_client: UserClient):
        print(f"[+] WebRTC Camera Streamer New User Connected: {usr_client.client_id}")
        print(f"Active Users: {self.user_clients_manager.clients_count}")

    def user_disconnected_handler(self, usr_client: UserClient):
        print(f"[-] WebRTC Camera Streamer User Disconnected: {usr_client.client_id}")
        print(f"Active Users: {self.user_clients_manager.clients_count}")

    def _remote_message_received(self, message, channel: ConnectionChannel, usr_client: UserClient):
        try:
            msg_data = json.loads(message)
            print(f"[📩] Message from {usr_client.client_id} via {channel.channel_name}: {msg_data}")
        except json.JSONDecodeError:
            print(f"[⚠️] Invalid JSON Message Received: {message}")
            return

        time_gap = time.time() - self.last_message_time
        if time_gap > 0.3:
            print(f"[⚠️] Message Delay Detected: {msg_data} (Time: {time_gap:.2f}s)")
            self.send_message('message too slow')

        self.last_message_time = time.time()
        pub.sendMessage('receive_message', message=msg_data)

    async def _send_topic_to_remote(self, message):
        await self.user_clients_manager.send_to_all_users(message)

    def send_message(self, message):
        asyncio.run_coroutine_threadsafe(self._send_topic_to_remote(message), self.loop)

    def connect(self):
        """WebRTC 연결을 시작합니다."""
        self.user_clients_manager.start()
        print("[🔄] WebRTC Camera Streamer Running...")

    def disconnect(self):
        """WebRTC 연결을 해제합니다."""
        self.user_clients_manager.stop()
        print("[✅] WebRTC Camera Streamer Stopped.")


if __name__ == "__main__":
    robot = WebRtcVideoChannel(
        robot_id=ROBOT_ID + "_camera",
        password="",
        server_host="175.126.123.199",
        server_port="8180"
    )               
    robot.connect()
    # 필요에 따라 disconnect()를 호출하여 연결 해제할 수 있습니다.
