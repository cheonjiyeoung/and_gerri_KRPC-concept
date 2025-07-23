import ketirtc.core
import ketirtc.core.audio
from ketirtc_operator.RemoteRobotController import RemoteRobotController
import os,sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from aiortc.contrib.media import MediaStreamTrack
from av import VideoFrame,AudioFrame
import cv2
import asyncio
import time
from pubsub import pub
import datetime
import threading
from ketirtc.core.audio.MultipleAudioTrackPlayer import AudioPlayerTrackContext

class OperatorMediaReceiever:
    def __init__(self,robot_info,operator_info,channel_info,kind="video"):
        self._tracks = []
        self.kind = kind
        self.server_host = '175.126.123.199'
        self.server_port = 9980
        self.user_id = operator_info['id']
        self.password = operator_info['password']
        self.full_id = robot_info['id'] + "_" + channel_info
        print(self.full_id)
        
        self.exit_flag = False
        self.dead_sign = False
        self.last_frame = None
        

        if kind == "video":
            self.channel = RemoteRobotController(target_id=self.full_id,
                                                    server_host=self.server_host,
                                                    server_port=self.server_port,
                                                    user_id=self.user_id,
                                                    password=self.password,
                                                    video_receiver=self)
        elif kind == "audio":
            audio_player = AudioPlayerTrackContext()
            self.channel = RemoteRobotController(target_id=self.full_id,
                                                    server_host=self.server_host,
                                                    server_port=self.server_port,
                                                    user_id=self.user_id,
                                                    password=self.password,
                                                    audio_receiver=audio_player)
        pub.subscribe(self.send_message, 'send_message')

    def addTrack(self, track: MediaStreamTrack):
        self._tracks.append(track)
        asyncio.ensure_future(self._run(track))

    async def _run(self, track: MediaStreamTrack):
        self.dead_sign = False
        while True:
            if self.kind == "video":
                try:
                    frame: VideoFrame = await track.recv()
                    img = frame.to_ndarray(format="bgr24")
                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.7
                    color = (0, 255, 0)
                    thickness = 1
                    position = (10, 30)  # 왼쪽 상단 좌표

                    cv2.putText(img, timestamp, position, font, font_scale, color, thickness)
                    self.last_frame = img
                    # print(f"{self.channel} : {img.shape}")
                except Exception as e:
                    print(f"Exception in video recv: {e}")
                    break
        if not self.exit_flag:
            self.dead_sign = True
            
    def reconnect(self):
        self.disconnect()
        self.channel._thread.join(timeout=30)
        if self.channel._thread.is_alive():
            print("재접속 실패")
        else:
            self.connect()

    def connect(self):
        self.channel.set_message_received_handler(self.handle_robot_message)
        try:
            threading.Thread(target=self.health_check,daemon=True).start()
            self.channel.start_thread()
        except Exception as e:
            print(f"Error connect in while: {e}")

    def disconnect(self):
        self.exit_flag = True
        self.channel.close()
        def shutdown():
            for task in asyncio.all_tasks(loop=self.channel._event_loop):
                task.cancel()
        self.channel._event_loop.call_soon_threadsafe(shutdown)

    def send_message(self, message):
        self.channel.send_json(message)
        print(f"Sent message: {message}")

    async def handle_robot_message(self, message, channel):
        print(f"robot message received: {message}")
        pub.sendMessage('receive_message', message=message)

    async def start(self):
        pass

    async def stop(self):
        pass

    def health_check(self):
        while True:
            if self.dead_sign and not self.exit_flag:
                self.reconnect()
            time.sleep(1)

    def get_frame(self):
        if self.last_frame is not None:
            return self.last_frame
        else:
            # print("None frame")
            return None



if __name__ == "__main__":
    test = OperatorMediaReceiever(robot_info=ROBOT_INFO,
                                  operator_info=OPERATOR_INFO,
                                  channel_info="audio",
                                  kind="audio")
    # test = OperatorMediaReciever(robot_info=ROBOT_INFO,
    #                               operator_info=OPERATOR_INFO,
    #                               channel_info="front_camera",
    #                               kind="video")
    # test2 = OperatorMediaReciever(robot_info=ROBOT_INFO,
    #                               operator_info=OPERATOR_INFO,
    #                               channel_info="rear_camera",
    #                               kind="video")
    test.connect()
    # test2.connect()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        test.disconnect()
        # test2.disconnect()
