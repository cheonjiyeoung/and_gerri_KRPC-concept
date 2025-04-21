import json
import threading

from asyncio import AbstractEventLoop
from ketirtc_robot import UserClientConnectionManager, ConnectionChannel, UserClient
from ketirtc_robot.webrtc.webrtc_connection_manager import WebRtcConnectionManager
from ketirtc_robot.camera.cam_dummy import DummyCamera
from pubsub import pub
import asyncio
import time

# old version port = 8180
# new version port = 9980

class WebRtcCommandChannel:
    def __init__(self, robot_id, password, server_host, server_port, camera=DummyCamera(), loop: AbstractEventLoop = None, message_duration_time=100):
        self.loop = loop
        self.message_duration_time = message_duration_time
        self.robot_id = robot_id
        self.password = password
        self.server_host = server_host
        self.server_port = server_port
        self.camera = camera
        self.last_message_time = time.time()

        self.signalling_server = WebRtcConnectionManager(robot_id=robot_id, signalling_password=password,
                                                         camera=camera,
                                                         signalling_server_host=server_host,
                                                         signalling_server_port=server_port,
                                                         )

        self.user_clients_manager = UserClientConnectionManager()
        self.user_clients_manager.add_connection_manager_channel(self.signalling_server)
        self.user_clients_manager.add_listener('user_message', self._remote_message_received)

        self.user_clients_manager.add_listener('user_connected', self.user_connected_handler)
        self.user_clients_manager.add_listener('user_disconnected', self.user_disconnected_handler)

        pub.subscribe(self.send_message, 'send_message')

        print("[✅] WebRtcCommandChannel Initialized.")

    def user_connected_handler(self, usr_client: UserClient):
        print(f'new user client connected: user_id: {usr_client.client_id}, {usr_client}')
        print(f'# of user clients: {self.user_clients_manager.clients_count}')

    def user_disconnected_handler(self, usr_client: UserClient):
        print(f'user client disconnected: user_id: {usr_client.client_id}, {usr_client}')
        print(f'# of user clients: {self.user_clients_manager.clients_count}')


    def _remote_message_received(self, message, channel: ConnectionChannel, usr_client: UserClient):
        msg_data = json.loads(message)
        print(f'message in json: {msg_data}')
        time_gap = time.time() - self.last_message_time
        if time_gap > 0.3:
            print(f'message is too slow: {msg_data}\n', 'time :', time_gap)
            self.send_message('message too slow')

        self.last_message_time = time.time()
        pub.sendMessage('receive_message', message=msg_data)

    async def _send_topic_to_remote(self, message):
        await self.user_clients_manager.send_to_all_users(message)

    def send_message(self, message):
        asyncio.run_coroutine_threadsafe(self.user_clients_manager.send_to_all_users(message), self.loop)

    def _run_loop_forever(self):
        """이벤트 루프를 백그라운드 스레드에서 실행"""
        asyncio.set_event_loop(self.loop)
        self.user_clients_manager.start()
        self.loop.run_forever()

    def connect(self):
        """WebRTC 브릿지 연결을 시작합니다."""
        self.loop_thread = threading.Thread(target=self._run_loop_forever, daemon=True)
        self.loop_thread.start()
        print("[🔄] WebRtcCommandChannel Running in Background...")

    def disconnect(self):
        """WebRTC 브릿지 연결을 해제합니다."""
        if self.loop.is_running():
            self.user_clients_manager.stop()
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.loop_thread.join()  # 스레드 종료 대기
            print("[✅] WebRtcCommandChannel Stopped.")
