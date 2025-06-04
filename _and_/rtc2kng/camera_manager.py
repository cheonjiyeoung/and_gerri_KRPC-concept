import time
import cv2
import logging
import threading
import platform


import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from _and_.rtc2kng.video_manager import VideoManager

logger = logging.getLogger(__name__)

class CameraManager(VideoManager):
    def __init__(self, index=0, name='cam', **kwargs):
        super().__init__(**kwargs)
        self.index = index
        self.name = name

        self.running = False
        self.start()

    def _open_capture(self, index):
        system = platform.system()
        if system == "Windows":
            return cv2.VideoCapture(index, cv2.CAP_DSHOW)
        elif system == "Linux":
            return cv2.VideoCapture(index, cv2.CAP_V4L2)
        else:
            return cv2.VideoCapture(index)

    def start(self):
        if self.running:
            return

        self.running = True
        self.cap = cv2.VideoCapture(self.index)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        self.print_feature()

    def _capture_loop(self):
        while self.running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    raise RuntimeError("❌ 프레임 읽기 실패")
                self.last_frame = frame
                self.last_frame_time = time.time()
                # if self.debug:
                #     cv2.imshow(self.name, frame)
                time.sleep(0.01)
            except Exception as e:
                logger.error(f"⚠️ 캡처 오류 발생: {e}")
                self.restart()

    def print_feature(self):
        print("🎥 카메라 속성 정보")
        print(f" - Width: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)} px")
        print(f" - Height: {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} px")
        print(f" - FPS: {self.cap.get(cv2.CAP_PROP_FPS)}")

        # ✅ FourCC 코드 변환
        fourcc_int = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4)])
        print(f" - FourCC: {fourcc_str}")

        # ✅ Auto Exposure 값 확인
        auto_exp = self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        auto_exp = "지원되지 않음" if auto_exp == -1 else auto_exp
        print(f" - Auto Exposure: {auto_exp}")

        # ✅ 기타 설정 확인
        params = {
            "Exposure": self.cap.get(cv2.CAP_PROP_EXPOSURE),
            "Brightness": self.cap.get(cv2.CAP_PROP_BRIGHTNESS),
            "Contrast": self.cap.get(cv2.CAP_PROP_CONTRAST),
            "Saturation": self.cap.get(cv2.CAP_PROP_SATURATION),
            "Gain": self.cap.get(cv2.CAP_PROP_GAIN),
            "Buffer Size": self.cap.get(cv2.CAP_PROP_BUFFERSIZE)
        }
        for key, value in params.items():
            print(f" - {key}: {'지원되지 않음' if value == -1 else value}")


    def restart(self):
        self.stop()
        time.sleep(2)
        self.start()

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
        logger.info("🛑 카메라 중지됨")