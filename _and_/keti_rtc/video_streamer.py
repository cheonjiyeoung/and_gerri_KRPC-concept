import cv2
import numpy as np
import time
from aiortc import VideoStreamTrack
from aiortc.mediastreams import MediaStreamError
from av import VideoFrame


class VideoStreamTrackWrapper(VideoStreamTrack):
    """
    ✅ 어떤 객체든 .last_frame or .get_frame()이 있으면 WebRTC 스트림으로 변환
    """
    def __init__(self, source, fps=30):
        super().__init__()
        self.source = source
        self.frame_time = 1.0 / fps

    def _get_frame(self):
        # get_frame() 메서드가 있으면 그걸 쓰고, 없으면 last_frame 속성 사용
        if hasattr(self.source, "get_frame") and callable(self.source.get_frame):
            return self.source.get_frame()
        elif hasattr(self.source, "last_frame"):
            return self.source.last_frame
        else:
            raise ValueError("Source must have .get_frame() or .last_frame")

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        frame = self._get_frame()

        if frame is None:
            raise MediaStreamError("No image available")

        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame


class VideoStreamer:
    """
    ✅ 범용 스트리머
    - CameraManager, StereoCameraCombiner, VRProcessor 등 어떤 영상 클래스든 스트리밍 가능
    """
    def __init__(self, source, fps=30):
        """
        :param source: .last_frame 또는 .get_frame()을 가진 객체
        :param fps: 스트림 프레임률
        """
        self.source = source
        self.fps = fps

    def get_video_stream_track(self):
        return VideoStreamTrackWrapper(self.source, fps=self.fps)
