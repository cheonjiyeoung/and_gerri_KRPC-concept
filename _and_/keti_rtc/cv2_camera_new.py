import asyncio
import datetime
import logging
import threading
import time
import traceback
from typing import Any, Dict

import cv2
from aiortc import VideoStreamTrack
from aiortc.mediastreams import MediaStreamError
from av import VideoFrame

from ketirtc_robot.camera.cam_base import Camera
logger = logging.getLogger(__name__)


class CV2StreamCamera(Camera):
    __started_devices: Dict[str, Any] = {}

    def __new__(cls, device_index=None, *args, **kwargs):
        if device_index is None:
            device_index = 0

        if device_index not in cls.__started_devices:
            logger.info(f'new CV2StreamCamera device_index={device_index}')
            cam = super(CV2StreamCamera, cls).__new__(cls)
            cls.__started_devices[device_index] = cam
        else:
            cam = cls.__started_devices[device_index]
        return cam

    def __init__(self, camera_index=0, width=None, height=None, fps=None):
        super(CV2StreamCamera, self).__init__()
        if hasattr(self, "_camera_index"):
            return
        if camera_index is None:
            self._camera_index = 0
        else:
            self._camera_index = camera_index

        self.tracks = set()

        self._video = None
        self._width = width
        self._height = height
        self._fps = fps

        self._last_image = None
        self._last_image_time = None

        self._started = False
        self.start()

        self._reading_lock = asyncio.Lock()
        self._reading_in_progress = False
        self._read_event = asyncio.Event()

    def start(self,):
        if self._started:
            return
        self._started = True
        logger.info(f'starting webrtc_camera camera_index={self._camera_index}')
        self._video = cv2.VideoCapture(self._camera_index)  # name
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self._video.set(cv2.CAP_PROP_FOURCC, fourcc)

        if self._width is not None:
            self._video.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        else:
            self._width = self._video.get(cv2.CAP_PROP_FRAME_WIDTH)

        self._fps = self._video.get(cv2.CAP_PROP_FPS)

        if self._height is not None:
            self._video.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        else:
            self._height = self._video.get(cv2.CAP_PROP_FRAME_HEIGHT)

    def capture(self):
        if self._video is None:
            raise Exception("self.__video is None. Camera not started!")
        ret, image = self._video.read()
        if image is not None:
            getimagetime = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3] + str(f'    ') + str(
                f'size: {int(self._width)}X{int(self._height)}')
            cv2.putText(image, getimagetime, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
        return image

    async def read(self):
        if self._reading_in_progress:
            await self._read_event.wait()
        else:
            self._reading_in_progress = True
            self._read_event.clear()
            try:
                image = self.capture()
                retry_count = 0
                while image is None and retry_count < 3:
                    image = self.capture()
                    retry_count += 1
                    if image is None:
                        await asyncio.sleep(0.01)

                if image is None:
                    raise Exception("Camera capture None image")
                self._last_image = image
                self._last_image_time = time.time()
            except Exception as ex:
                logger.exception(f"exception: {ex}")
                raise ex
            finally:
                self._reading_in_progress = False
                self._read_event.set()
        return self._last_image

    async def get_image(self, exp_time=None):
        if self._last_image is not None and self._last_image_time is not None and self._last_image_time >= exp_time:
            return self._last_image
        return await self.read()

    def get_video_stream_track(self) -> VideoStreamTrack:
        if not self._started:
            self.start()

        track = CV2VideoStreamTrack(self)

        def on_track_end():
            self.tracks.discard(track)
            logger.debug(f"webrtc_camera track ended: video_track_count = {len(self.tracks)}")
            if len(self.tracks) == 0:
                self.stop()

        self.tracks.add(track)
        logger.debug(f"track created. video_track_count = {len(self.tracks)}")
        track.on("ended", on_track_end)
        return track

    def stop(self):
        if not self._started:
            logger.info(f'webrtc_camera {self._camera_index} already stopped, skip')
            return

        logger.info(f'stop webrtc_camera {self._camera_index}')
        self._started = False
        if self._video is not None:
            self._video.release()
            self._video = None

    @classmethod
    def _release_camera(cls, cam_id):
        cls.__started_devices.pop(cam_id)


class CV2VideoStreamTrack(VideoStreamTrack):
    __next_track_id = 0
    __tracks_count = 0
    lock = threading.Lock()

    def __init__(self, camera: CV2StreamCamera, fps=30):
        super(CV2VideoStreamTrack, self).__init__()
        with CV2VideoStreamTrack.lock:
            self._track_id = CV2VideoStreamTrack.__next_track_id
            CV2VideoStreamTrack.__next_track_id += 1
            CV2VideoStreamTrack.__tracks_count += 1
        self._camera = camera
        self._fps = fps
        self.frame_time = 1.0/fps
        logger.debug(f"new CV2VideoStreamTrack. track_id={self._track_id}, instance count: {CV2VideoStreamTrack.__tracks_count}")

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        if self.readyState != "live":
            raise MediaStreamError
        try:
            lasted_frame_time = time.time() - self.frame_time
            image = await self._camera.get_image(lasted_frame_time)
            if image is None:
                raise MediaStreamError

            frame = VideoFrame.from_ndarray(image, format='bgr24')
            frame.pts = pts
            frame.time_base = time_base
            return frame
        except MediaStreamError as ex:
            print(f'webrtc_camera MediaStream reading error')
            logger.debug(f"CV2VideoStreamTrack MediaStream: {ex}")
        except Exception as ex:
            print(f'webrtc_camera exception: {ex}')
            logger.exception(f"error: {ex}", exc_info=True, stack_info=True)
            traceback.print_exc()
            raise MediaStreamError

    def stop(self):
        with CV2VideoStreamTrack.lock:
            CV2VideoStreamTrack.__tracks_count -= 1
        logger.debug(f"webrtc_camera track {self._track_id} stopped, instance count={CV2VideoStreamTrack.__tracks_count}")
        super().stop()
