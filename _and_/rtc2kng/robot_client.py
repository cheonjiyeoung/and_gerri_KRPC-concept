import asyncio
import json
import time
import websockets
import requests
import threading
import uuid

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from _and_.rtc2kng.robot_peer_connection import RobotPeerConnectionHandler

### 마이스터 센터 내부 '172.20.1.250'
### 외부 WAN           '125.131.105.165'

# SERVER_IP = '125.131.105.165'
SERVER_IP = '172.20.1.250'
# SERVER_IP = 'localhost'
SERVER_PORT = 25000

ROBOT_ID = 'test_robot'
ROOM_ID = 'test_room'

# ✅ RobotClient
class RobotClient:
    def __init__(self, *, room_id,  robot_id, server_ip='localhost', server_port=8000,
                 video_track=None, audio_track=None, audio_player=None):
        self.room_id = room_id
        self.robot_id = robot_id
        self.ws_url = f"ws://{server_ip}:{server_port}/ws/{room_id}"
        self.register_url = f"http://{server_ip}:{server_port}/register_robot/{room_id}"
        self.video_track = video_track
        self.audio_track = audio_track
        self.audio_player = audio_player

        self.connections = {}
        self.ice_candidate_queue = {}
        self.running = False
        self.thread = None
        self.loop = None
        self.ws = None

    def connect(self):
        if self.running:
            print("⚠️ 이미 실행 중입니다.")
            return
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._start())

    async def _start(self):
        if self.register_url:
            await self._register()

        print("🎥/🎙️ 미디어 준비 완료")
        async with websockets.connect(self.ws_url) as ws:
            self.ws = ws
            print(f"🔌 WebSocket 연결됨 (로봇 ID: {self.robot_id}, 방: {self.room_id})")

            # WebSocket 연결 직후 로봇 자신도 "ready" 메시지를 보내 서버에 등록
            await ws.send(json.dumps({
                "type": "ready",
                "sender": self.robot_id
            }))
            print(f"🤖 로봇 '{self.robot_id}' 서버에 'ready' 상태 전송 완료")

            async for msg in ws:
                await self._handle_message(json.loads(msg), ws)

    async def _register(self):
        res = requests.post(self.register_url)
        if res.status_code == 200:
            print(f"✅ 로봇 '{self.robot_id}' (방: '{self.room_id}') 등록 완료")
        else:
            raise RuntimeError(f"❌ 등록 실패: {res.status_code} {res.text}")

    async def _handle_message(self, data, ws):

        message_sender = data.get("sender")  # 메시지를 보낸 쪽의 ID
        message_receiver = data.get("receiver")  # 메시지를 받을 쪽의 ID (없을 수도 있음)
        message_type = data.get("type")

        # ✅ Ready 상태에서 Offer 생성
        if message_type == "ready":
            if not message_sender:  # "ready" 메시지는 sender 필드에 오퍼레이터 ID가 있어야 함
                print(f"⚠️ 'ready' 메시지에 sender 정보가 없습니다. 무시합니다.")
                return

            operator_id = message_sender  # 이 오퍼레이터와 연결 시작
            print(f"✅ 오퍼레이터 '{operator_id}' 준비 완료. WebRTC 연결 설정을 시작합니다.")

            video_tracks = {}  # video_tracks 변수 초기화
            if self.video_track:
                video_tracks = {
                    label: vm.create_video_track()
                    for label, vm in self.video_track.items()
                }

            handler = RobotPeerConnectionHandler(
                robot_id=self.robot_id,
                operator_id=operator_id,
                ws=ws,
                video_track=video_tracks,
                audio_track=self.audio_track,
                audio_player=self.audio_player.play if self.audio_player else None,
                loop=asyncio.get_event_loop(),
                on_disconnect=self._handle_disconnect
            )
            self.connections[operator_id] = handler

            for label, track in video_tracks.items():
                await ws.send(json.dumps({
                    "type": "track_info",
                    "sender": self.robot_id,
                    "receiver": operator_id,
                    "track": {
                        "label": label,
                        "track_id": track.id
                    }
                }))
                print(
                    f"📤 비디오 트랙 정보 전송 (from: {self.robot_id} to: {operator_id}): {label} ({track.id})")

            await handler.create_and_send_offer()


        # ✅ Answer 수신 처리
        elif data.get("type") == "answer":
            if message_receiver and message_receiver != self.robot_id:
                print(f"⚠️ Answer 메시지의 receiver({message_receiver})가 로봇 ID({self.robot_id})와 다릅니다. 무시합니다.")
                return
            if message_sender and message_sender in self.connections:
                print(f"📥 Answer 수신 (from: {message_sender} for: {self.robot_id})")
                print(data)  # 상세 데이터 로그
                await self.connections[message_sender].set_remote_description(data["sdp"], data["type"])
            else:
                print(f"⚠️ Answer 수신 오류: 해당 sender({message_sender})에 대한 연결이 없거나 sender 정보 누락.")


        # ✅ Candidate
        elif message_type == "candidate":  # 수정 후
            # message_sender는 Candidate를 보낸 오퍼레이터의 ID
            # message_receiver는 이 로봇의 ID 여야 함
            if message_receiver and message_receiver != self.robot_id:
                print(f"⚠️ Candidate 메시지의 receiver({message_receiver})가 로봇 ID({self.robot_id})와 다릅니다. 무시합니다.")
                return

            candidate_payload_obj = data.get("candidate")  # Unity가 보낸 중첩된 candidate 객체
            if not candidate_payload_obj or not isinstance(candidate_payload_obj, dict):
                print(f"⚠️ 잘못된 Candidate 메시지 형식 (페이로드 누락 또는 타입 오류 from {message_sender}): {data}")
                return

            if message_sender and message_sender not in self.connections:
                if not hasattr(self, 'ice_candidate_queue'): self.ice_candidate_queue = {}
                if message_sender not in self.ice_candidate_queue: self.ice_candidate_queue[message_sender] = []
                self.ice_candidate_queue[message_sender].append(candidate_payload_obj)  # 수정: 페이로드 객체 저장
                print(f"⏳ ICE 후보 저장 (Offer/Answer 교환 대기 중 from {message_sender})")
            elif message_sender and message_sender in self.connections:
                await self.connections[message_sender].add_ice_candidate(candidate_payload_obj)  # 수정: 페이로드 객체 전달
            else:
                print(f"⚠️ Candidate 수신 오류: sender({message_sender})가 없거나 해당 연결 없음.")

        elif message_type == "offer":
            # message_sender는 Offer를 보낸 peer의 ID
            # message_receiver는 이 로봇의 ID 여야 함
            if message_receiver and message_receiver != self.robot_id:
                print(f"⚠️ Offer 메시지의 receiver({message_receiver})가 현재 로봇 ID({self.robot_id})와 다릅니다. 무시합니다.")
                return

            print(
                f"📥 Offer 수신 (from: {message_sender} for: {self.robot_id}) - (주의: 일반적인 오퍼레이터-로봇 시나리오에서는 로봇이 Offer를 보냄)")

            video_tracks = {}
            if self.video_track:
                video_tracks = {
                    label: vm.create_video_track()
                    for label, vm in self.video_track.items()
                }

            handler = RobotPeerConnectionHandler(
                robot_id=self.robot_id,
                operator_id=message_sender,
                ws=ws,
                video_track=video_tracks,
                audio_track=self.audio_track,
                audio_player=self.audio_player.play if self.audio_player else None,
                loop=asyncio.get_event_loop(),
                on_disconnect=self._handle_disconnect
            )
            self.connections[message_sender] = handler

            await handler.set_remote_description(data["sdp"], data["type"])
            answer = await handler.create_answer()
            if answer:
                await ws.send(json.dumps({
                    "type": "answer",
                    "sdp": answer.sdp,
                    "sender": self.robot_id,
                    "receiver": message_sender
                }))
                print(f"📤 Answer 전송 완료 (from: {self.robot_id} to: {message_sender})")

            if hasattr(self, 'ice_candidate_queue') and message_sender in self.ice_candidate_queue:
                for candidate_payload in self.ice_candidate_queue[message_sender]:
                    await handler.add_ice_candidate(candidate_payload)
                del self.ice_candidate_queue[message_sender]


    def _handle_disconnect(self, peer_id):
        if peer_id in self.connections:
            del self.connections[peer_id]

if __name__ == "__main__":
    from camera_manager import CameraManager
    from dummy_robot_controller import DummyController
    from status_manager import StatusManager
    from audio_track import AudioSendTrack
    from pyaudio_manager import AudioRecorder, AudioPlayer

    recorder = AudioRecorder()
    audio_player = AudioPlayer(volume=0, silence_threshold=50)
    audio_track = AudioSendTrack(recorder)

    front_cam = CameraManager(index=0, width=9999, height=9999, debug=False)
    # rear_cam = CameraManager(index=1, width=1920, height=1080, debug=False)


    client = RobotClient(
        room_id=ROOM_ID,
        robot_id=ROBOT_ID,
        video_track={"front_cam": front_cam},
        audio_track=audio_track,
        audio_player=audio_player,
        server_ip=SERVER_IP,
        server_port=SERVER_PORT,
    )


    controller = DummyController()
    status_manager = StatusManager(
        name="Piper",
        category="quadruped",
        model="Piper-X",
        controller=controller,
        interval=1
    )

    client.connect()

    try:
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        print("🧼 종료됨")
