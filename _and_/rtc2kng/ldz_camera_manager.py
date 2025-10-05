import time
import cv2
import logging
import threading
import numpy as np
import platform
from pubsub import pub
import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from _and_.rtc2kng.video_manager import VideoManager

logger = logging.getLogger(__name__)


class LDZCameraManager(VideoManager):
    """
    무손실 디지털 줌(LDZ) 기능을 제공하는 독립적인 SBS 카메라 매니저.
    """

    def __init__(self, source=0, input_width=3840, input_height=1080, fps=30,
                 output_width=1280, output_height=360, debug=False, **kwargs):
        super().__init__(width=output_width, height=output_height, fps=fps, debug=debug)

        self.source = source
        self.input_width = input_width
        self.input_height = input_height
        self.output_width = output_width
        self.output_height = output_height
        self.fps = fps
        self.running = False

        self.zoom_level = 1.0
        self.max_zoom_level = 1.0

        self.start()
        pub.subscribe(self.zoom_step_control, "zoom_step_control")

    def _open_capture(self, source):
        system = platform.system()
        if system == "Windows":
            return cv2.VideoCapture(source, cv2.CAP_DSHOW)
        elif system == "Linux":
            return cv2.VideoCapture(source, cv2.CAP_V4L2)
        else:
            return cv2.VideoCapture(source)

    def start(self):
        if self.running: return
        self.running = True
        self.cap = self._open_capture(self.source)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.input_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.input_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

        time.sleep(1)
        self.actual_input_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_input_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._calculate_max_zoom()
        self.print_feature()

    def _capture_loop(self):
        while self.running:
            try:
                ret, raw_frame = self.cap.read()
                if not ret: raise RuntimeError("❌ 프레임 읽기 실패")
                self.last_frame = self._ldz_zoom_process(raw_frame)
                self.last_frame_time = time.time()

            except Exception as e:
                logger.error(f"⚠️ 캡처 오류 발생: {e}")
                self.restart()

    def _ldz_zoom_process(self, raw_frame):
        """고해상도 원본 프레임을 받아 줌/리사이즈 처리된 최종 프레임을 반환합니다."""
        mid_point = raw_frame.shape[1] // 2
        left_high_res, right_high_res = raw_frame[:, :mid_point], raw_frame[:, mid_point:]

        processed_left = self._process_single_eye(left_high_res)
        processed_right = self._process_single_eye(right_high_res)

        return cv2.hconcat([processed_left, processed_right])

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive(): self.thread.join()
        if hasattr(self, 'cap') and self.cap.isOpened(): self.cap.release()
        logger.info("🛑 LDZCameraManager 중지됨")

    def restart(self):
        self.stop()
        time.sleep(1)
        self.start()

    def _calculate_max_zoom(self):
        source_w_eye = self.actual_input_width // 2
        source_h_eye = self.actual_input_height
        target_w_eye = self.output_width // 2
        target_h_eye = self.output_height
        if target_w_eye > 0 and target_h_eye > 0:
            ratio_w, ratio_h = source_w_eye / target_w_eye, source_h_eye / target_h_eye
            self.max_zoom_level = min(ratio_w, ratio_h)
        if self.max_zoom_level < 1.0: self.max_zoom_level = 1.0

    def _process_single_eye(self, eye_frame):
        target_w, target_h = self.output_width // 2, self.output_height
        h, w, _ = eye_frame.shape
        if self.zoom_level <= 1.0:
            return cv2.resize(eye_frame, (target_w, target_h), cv2.INTER_AREA)
        else:
            crop_w, crop_h = int(w / self.zoom_level), int(h / self.zoom_level)
            cx, cy = w / 2, h / 2
            x1, y1 = int(cx - crop_w / 2), int(cy - crop_h / 2)
            cropped = eye_frame[y1:y1 + crop_h, x1:x1 + crop_w]
            return cv2.resize(cropped, (target_w, target_h), cv2.INTER_LINEAR)

    def set_zoom(self, level=1.0):
        new_level = max(1.0, min(level, self.max_zoom_level))
        if new_level != self.zoom_level: self.zoom_level = new_level

    def zoom_step_control(self, step=0):
        if step:
            self.set_zoom(self.zoom_level + step)
            pub.sendMessage('zoom_level', level=self.zoom_level)

    def print_feature(self):
        print("🎥 LDZ 카메라 속성 정보")
        print(f" - Input Resolution (Actual): {self.actual_input_width} x {self.actual_input_height} px")
        print(f" - Output Resolution: {self.output_width} x {self.output_height} px")
        print(f" - Lossless Max Zoom: {self.max_zoom_level:.2f}x")
        print(f" - FPS: {self.cap.get(cv2.CAP_PROP_FPS)}")
        print("-" * 40)