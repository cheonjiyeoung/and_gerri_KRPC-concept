import cv2
import numpy as np
from aiortc import VideoStreamTrack
from aiortc.mediastreams import MediaStreamError
from av import VideoFrame
import time

import os
import sys
CURRENT_FILE = os.path.abspath(__file__)
VENV_DIR = os.path.dirname(sys.executable)
PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
sys.path.insert(0, PROJECT_ROOT)

from avatar_darm.robot.network_tools.webrtc.cam_manager import CameraManager


class StereoCameraCombiner:
    def __init__(self,
                 cam1:CameraManager, cam2:CameraManager,
                 width=None, height=None, fps=30,
                 rotate1_angle=0,  # 각도로 변경 (시계방향 양수)
                 rotate2_angle=0):  # 각도로 변경 (반시계방향 음수)
        self.cam1 = cam1
        self.cam2 = cam2
        self.rotate1_angle = rotate1_angle
        self.rotate2_angle = rotate2_angle
        self.output_width = width
        self.output_height = height


    def _resize_for_split_screen(self, frame, target_width, target_height):
        return cv2.resize(frame, (target_width, target_height))

    def _rotate_frame(self, frame, angle):
        """각도에 따라 이미지를 회전"""
        if angle == 0:
            return frame
        elif angle == 90:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif angle == -90:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif angle == 180:
            return cv2.rotate(frame, cv2.ROTATE_180)
        else:
            # 임의 각도 회전을 위한 코드
            height, width = frame.shape[:2]
            center = (width/2, height/2)
            rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
            return cv2.warpAffine(frame, rotation_matrix, (width, height))

    @property
    def last_frame(self):
        if self.cam1.last_frame is None or self.cam2.last_frame is None:
            return None

        # 프레임 회전
        frame1 = self._rotate_frame(self.cam1.last_frame, self.rotate1_angle)
        frame2 = self._rotate_frame(self.cam2.last_frame, self.rotate2_angle)

        # 두 프레임을 수축하여 한 화면에 표시 (16:9 유지)
        if self.output_width is not None and self.output_height is not None:
            target_width = self.output_width // 2
            target_height = self.output_height
        else:
            target_width = None
            target_height = None

        if target_width is not None and target_height is not None:  
            frame1 = self._resize_for_split_screen(frame1, target_width, target_height)
            frame2 = self._resize_for_split_screen(frame2, target_width, target_height)

        combined = np.hstack((frame1, frame2))
        return combined

    def stop(self):
        self.cam1.stop()
        self.cam2.stop()
