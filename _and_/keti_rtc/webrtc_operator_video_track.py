# operator_video.py
from pubsub import pub
import cv2

class VideoReceiverTrack:
    kind = "video"

    def __init__(self, track, label="video", debug=False):
        self.track = track
        self.label = label
        self.last_frame = None
        self.debug = debug

    async def recv(self):
        frame = await self.track.recv()
        frame = frame.to_ndarray(format="bgr24")
        if self.debug:
            safe_label = self.label.encode("ascii", "ignore").decode()
            cv2.imshow(safe_label, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt
        pub.sendMessage('receive_frame', frame=frame)
        self.last_frame = frame
        return frame

async def video_receiver(track):
    while True:
        try:
            await track.recv()
        except KeyboardInterrupt:
            print(f"🛑 {track.label} 비디오 수신 종료")
            break

class VideoManager:
    def __init__(self):
        self.last_frame = None
        pub.subscribe(self.update_image, 'receive_frame')

    def update_image(self, frame):
        self.last_frame = frame

    def get_frame(self):
        return self.last_frame
