from aiortc import VideoStreamTrack
from av import VideoFrame
import numpy as np
import cv2

class VideoSendTrack(VideoStreamTrack):
    def __init__(self, camera):
        super().__init__()
        self.camera = camera
        self.kind = "video"


    async def recv(self):
        pts, time_base = await self.next_timestamp()
        frame = self.camera.get_last_frame()

        if frame is None:
            frame = np.zeros((self.camera.height, self.camera.width, 3), dtype=np.uint8)

        try:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        except Exception as e:
            print(f"⚠️ 프레임 변환 오류: {e}")
            frame = np.zeros((self.camera.height, self.camera.width, 3), dtype=np.uint8)

        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        return video_frame
