from typing import Tuple

import cv2
import time
import threading
import logging
import os
from pubsub import pub

logger = logging.getLogger(__name__)


class CameraManager:
    """
    ✅ 공통 카메라 인터페이스 (OpenCV 기반) + WebRTC 비종속
    - .last_frame or .get_frame() 제공으로 WebRTC 브릿지에 사용 가능
    - 내부 스레드로 영상 스트리밍 루프 수행
    """
    def __init__(self, camera_index=0, width=640, height=480, fps=30, camera_name='cam', auto_start=True):
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


        # ✅ PubSub 구독 (외부 요청이 오면 save_frame 실행)
        pub.subscribe(self.save_frame, "save_frame")

        if auto_start:
            self.start()

    def set_resolution(self, width: int, height: int) -> Tuple[int, int]:
        if self.cap is not None and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        else:
            self.width = width
            self.height = height
        logger.info(f'camera {self.camera_index} set resolution to {self.width}X{self.height}')
        return int(self.width), int(self.height)

    def set_fps(self, fps: int) -> int:
        if self.cap is not None and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        else:
            self.fps = fps
        logger.info(f'camera {self.camera_index} set fps to {self.fps}')
        return int(self.fps)

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
        self.print_feature()
        logger.info(f"📷 카메라 {self.camera_index} 시작됨")

    def _capture_loop(self):
        while self.running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    raise RuntimeError("❌ 프레임 읽기 실패")
                self.last_frame = frame
                self.last_frame_time = time.time()
                # time.sleep(1 / self.fps)
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

    def save_frame(self, file_path, file_name):
        """
        ✅ PubSub을 통해 호출될 때만 프레임 저장
        """
        if self.last_frame is None:
            logger.warning("🚨 저장할 프레임이 없습니다!")
            return

        # ✅ 파일명 생성
        filename = f"{self.camera_name}_{file_name:05d}.jpg"
        file_path = os.path.join(file_path, filename)

        # ✅ 프레임 저장
        cv2.imwrite(file_path, self.last_frame)
        logger.info(f"📸 프레임 저장됨: {file_path}")