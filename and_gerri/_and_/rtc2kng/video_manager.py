import abc
from aiortc import VideoStreamTrack
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from _and_.rtc2kng.video_send_track import VideoSendTrack

class VideoManager(abc.ABC):
    def __init__(self, width=640, height=480, fps=30, debug=False):
        self.width = width
        self.height = height
        self.fps = fps
        self.debug = debug
        self.last_frame = None
        self._task = None

    def get_last_frame(self):
        return self.last_frame

    def create_video_track(self):
        return VideoSendTrack(self)
