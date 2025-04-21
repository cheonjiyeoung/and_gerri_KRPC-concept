import cv2
import time
import threading
import logging
from aiortc import VideoStreamTrack
from av import VideoFrame
from aiortc.mediastreams import MediaStreamError

logger = logging.getLogger(__name__)


class CameraManager:
    """
    ✅ 공통 카메라 인터페이스 (OpenCV 기반) + WebRTC 비종속
    - .last_frame or .get_frame() 제공으로 WebRTC 브릿지에 사용 가능
    - 내부 스레드로 영상 스트리밍 루프 수행
    """
    def __init__(self, camera_index=0, width=640, height=480, fps=30, camera_name='cam'):
        self.camera_name = camera_name
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.running = False
        self.last_frame = None
        self.last_frame_time = None
        self.cap = None
        self.thread = None

    def start(self):
        if self.running:
            logger.warning("📷 이미 실행 중")
            return

        self.running = True
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.info(f"📷 카메라 {self.camera_index} 시작됨")

    def _capture_loop(self):
        while self.running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    raise RuntimeError("❌ 프레임 읽기 실패")
                self.last_frame = frame
                self.last_frame_time = time.time()
                time.sleep(1 / self.fps)
            except Exception as e:
                logger.error(f"⚠️ 캡처 오류 발생: {e}")
                self.restart()

    def get_frame(self):
        return self.last_frame

    def restart(self):
        self.stop()
        time.sleep(2)
        self.start()

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
        logger.info("🛑 카메라 중지됨")
