import math

import numpy
from av import VideoFrame

from aiortc import VideoStreamTrack
from ketirtc_robot.camera import Camera


class DummyCamera(Camera):
    def __init__(self):
        super().__init__()

    def get_video_stream_track(self) -> VideoStreamTrack:
        return DummyVideoStreamTrack()


class DummyVideoStreamTrack(VideoStreamTrack):
    """
    A video track that returns an animated flag.
    """

    def __init__(self):
        super().__init__()  # don't forget this!
        self.counter = 0
        height, width = int(480/10), int(640/10)

        self.frames = []
        for k in range(30):
            img = self._generate_random_frame(width, height)
            self.frames.append(
                VideoFrame.from_ndarray(img, format="bgr24")
            )

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        frame = self.frames[self.counter % 30]
        frame.pts = pts
        frame.time_base = time_base
        self.counter += 1
        return frame

    def _generate_random_frame(self, width, height):
        return numpy.random.randint(0, 256, (height, width, 3), dtype=numpy.uint8)

    def _create_rectangle(self, width, height, color):
        data_bgr = numpy.zeros((height, width, 3), numpy.uint8)
        data_bgr[:, :] = color
        return data_bgr
