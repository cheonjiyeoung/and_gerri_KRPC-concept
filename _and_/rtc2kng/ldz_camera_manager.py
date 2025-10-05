import time
import cv2
import logging
import threading
import numpy as np
from pubsub import pub

# 같은 폴더에 있는 camera_manager에서 CameraManager 클래스를 가져옴
from .camera_manager import CameraManager

logger = logging.getLogger(__name__)

# 클래스 이름: LDZCameraManager
class LDZCameraManager(CameraManager):
    """
    기존 CameraManager를 상속받아 무손실 디지털 줌(LDZ) 기능을 추가한 특화 클래스.
    """

    def __init__(self, source=0, input_width=3840, input_height=1080, fps=30,
                 output_width=1280, output_height=360, debug=False, **kwargs):

        super().__init__(source=source, width=input_width, height=input_height, fps=fps, debug=debug)

        self.input_width = input_width
        self.input_height = input_height
        self.output_width = output_width
        self.output_height = output_height

        self.zoom_level = 1.0
        self.max_zoom_level = 1.0
        self.actual_input_width = 0
        self.actual_input_height = 0


        pub.subscribe(self.zoom_step_control, "zoom_step_control")

    def start(self):
        super().start()
        time.sleep(1)
        self.actual_input_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_input_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if self.actual_input_width == 0:
            logger.warning("⚠️ 카메라 실제 해상도 획득 실패. 요청 값으로 대체합니다.")
            self.actual_input_width = self.input_width
            self.actual_input_height = self.input_height
        
        self._calculate_max_zoom()
        self.print_feature()

    def _calculate_max_zoom(self):
        source_w_eye = self.actual_input_width // 2
        source_h_eye = self.actual_input_height
        target_w_eye = self.output_width // 2
        target_h_eye = self.output_height

        if target_w_eye > 0 and target_h_eye > 0:
            ratio_w = source_w_eye / target_w_eye
            ratio_h = source_h_eye / target_h_eye
            self.max_zoom_level = min(ratio_w, ratio_h)
        
        if self.max_zoom_level < 1.0: self.max_zoom_level = 1.0

    def _process_single_eye(self, eye_frame):
        target_w = self.output_width // 2
        target_h = self.output_height
        h, w, _ = eye_frame.shape

        if self.zoom_level <= 1.0:
            return cv2.resize(eye_frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
        else:
            crop_w = int(w / self.zoom_level)
            crop_h = int(h / self.zoom_level)
            center_x, center_y = w / 2, h / 2
            x1 = int(center_x - crop_w / 2)
            y1 = int(center_y - crop_h / 2)
            x2 = x1 + crop_w
            y2 = y1 + crop_h
            cropped_frame = eye_frame[y1:y2, x1:x2]
            return cv2.resize(cropped_frame, (target_w, target_h), interpolation=cv2.INTER_LINEAR)

    def get_processed_sbs_frame(self):
        high_res_frame = self.get_last_frame()
        if high_res_frame is None: return None
        
        h, w, _ = high_res_frame.shape
        if w < h: 
            return cv2.resize(high_res_frame, (self.output_width, self.output_height))

        mid_point = w // 2
        left_high_res = high_res_frame[:, :mid_point]
        right_high_res = high_res_frame[:, mid_point:]
        
        processed_left = self._process_single_eye(left_high_res)
        processed_right = self._process_single_eye(right_high_res)
        
        return cv2.hconcat([processed_left, processed_right])
        
    def set_zoom(self, level=1.0):
        new_level = max(1.0, min(level, self.max_zoom_level))
        if new_level != self.zoom_level:
            self.zoom_level = new_level
            logger.info(f"🔎 줌 레벨 변경: {self.zoom_level:.2f}x")
        if self.zoom_level == self.max_zoom_level:
            logger.info("👍 최대 줌 레벨에 도달했습니다.")

    def zoom_control(self, level):
        if level:
            self.set_zoom(level)
            pub.sendMessage('zoom_level', self.zoom_level)

    def zoom_step_control(self, step = 0):
        if step:
            zoom_level = self.zoom_level + step
            self.set_zoom(zoom_level)
            pub.sendMessage('zoom_level', self.zoom_level)

    def print_feature(self):
        super().print_feature()
        print(f" - Output Resolution: {self.output_width} x {self.output_height} px")
        print(f" - Lossless Max Zoom: {self.max_zoom_level:.2f}x")
        print("-" * 40)