import pyzed.sl as sl
import cv2
import logging
import threading
import time
import numpy as np
import abc
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from _and_.rtc2kng.video_manager import VideoManager

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class VRZEDCameraManager(VideoManager):
    def __init__(self, source=0, capture_width=3840, capture_height=1080, fps=30,
                 output_width=1280, output_height=360, **kwargs):
        super().__init__(**kwargs)

        self.source = source
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.fps = fps
        self.output_width = output_width
        self.output_height = output_height

        self.zed_width = self.capture_width // 2
        self.zed_height = self.capture_height

        self.zoom_level = 1.0
        self.max_zoom_level = 1.0

        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()

        if isinstance(self.source, str):
            self.init_params.set_from_svo_file(self.source)
        else:
            self.init_params.set_from_camera_id(self.source)

        self.init_params.camera_resolution = self._get_sl_resolution(self.zed_width, self.zed_height)
        self.init_params.camera_fps = self.fps
        self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_minimum_distance = 0.2

        self.runtime_params = sl.RuntimeParameters()
        self.depth_map = sl.Mat()
        self.point_cloud = sl.Mat()

        self.running = False
        self.thread = None

    def _get_sl_resolution(self, w, h):
        if w == 1920 and h == 1080: return sl.RESOLUTION.HD1080
        if w == 1280 and h == 720: return sl.RESOLUTION.HD720
        return sl.RESOLUTION.HD1080

    def start(self):
        if self.running: return
        logger.info("VRZEDCameraManager 시작 중...")

        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            logger.error(f"ZED 카메라 열기 실패: {err}")
            return

        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.info("ZED 카메라 스트림 시작됨.")

    def _capture_loop(self):
        left, right = sl.Mat(), sl.Mat()
        is_first_frame = True

        while self.running:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # 컬러 이미지와 뎁스 맵은 매번 가져옴 (화면 표시에 필요)
                self.zed.retrieve_image(left, sl.VIEW.LEFT)
                self.zed.retrieve_image(right, sl.VIEW.RIGHT)
                self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)

                # ✨ 포인트 클라우드 계산(retrieve_measure)은 여기서 더 이상 하지 않음

                left_np = left.get_data()[:, :, :3]
                right_np = right.get_data()[:, :, :3]

                self.last_frame = np.concatenate((left_np, right_np), axis=1)

                if is_first_frame:
                    self._calculate_max_zoom()
                    self.print_feature()
                    is_first_frame = False
            else:
                time.sleep(0.01)

    def get_point_cloud_value(self, x, y):
        """(On-Demand) 3D 공간 좌표 (X, Y, Z)를 계산하여 반환합니다."""
        if not self.running or self.point_cloud is None:
            return None

        # ✨ 이 함수가 호출되는 시점에 포인트 클라우드 계산을 요청
        err_retrieve = self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ)

        if err_retrieve == sl.ERROR_CODE.SUCCESS:
            err, point_3d = self.point_cloud.get_value(x, y)
            if err == sl.ERROR_CODE.SUCCESS and np.isfinite(point_3d[2]):
                return point_3d[:3]

        return None

    def _calculate_max_zoom(self):
        if self.last_frame is None: return
        h, w, _ = self.last_frame.shape
        source_w_eye, source_h_eye = w // 2, h
        target_w_eye, target_h_eye = self.output_width // 2, self.output_height
        if target_w_eye > 0 and target_h_eye > 0:
            ratio_w = source_w_eye / target_w_eye
            ratio_h = source_h_eye / target_h_eye
            self.max_zoom_level = min(ratio_w, ratio_h)
        if self.max_zoom_level < 1.0: self.max_zoom_level = 1.0

    def get_processed_frame(self):
        sbs_frame = self.get_last_frame()
        if sbs_frame is None: return None
        left, right = sbs_frame[:, :sbs_frame.shape[1] // 2], sbs_frame[:, sbs_frame.shape[1] // 2:]
        processed_left = self._process_single_eye(left)
        processed_right = self._process_single_eye(right)
        return cv2.hconcat([processed_left, processed_right])

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

    def set_zoom(self, level):
        self.zoom_level = max(1.0, min(level, self.max_zoom_level))
        logger.info(f"🔎 줌 레벨 변경: {self.zoom_level:.2f}x (최대: {self.max_zoom_level:.2f}x)")

    def print_feature(self):
        print("\n" + "=" * 40)
        print("🎥 VR ZED 카메라 정보")
        print(f" - 원본 해상도 (SBS): {self.capture_width} x {self.capture_height}")
        print(f" - 출력 해상도 (SBS): {self.output_width} x {self.output_height}")
        print(f" - 무손실 최대 줌  : {self.max_zoom_level:.2f}x")
        print("=" * 40 + "\n")

    def stop(self):
        self.running = False
        if self.thread: self.thread.join()
        if self.zed.is_opened(): self.zed.close()
        logger.info("VRZEDCameraManager 중지됨.")


# --- 🧪 메인 테스트 실행부 ---
if __name__ == "__main__":

    cam = VRZEDCameraManager(
        source=0, capture_width=3840, capture_height=1080,
        output_width=1280, output_height=360, fps=30
    )
    cam.start()

    window_name = "VR ZED Camera Test - Click Left Image"
    cv2.namedWindow(window_name)

    # ✨ 1. 마우스 클릭 정보를 저장할 공유 딕셔너리
    mouse_data = {
        "cam_instance": cam,
        "clicked_point": None,  # (x, y) on output image
        "point_3d": None,  # [X, Y, Z] in meters
    }


    # ✨ 2. 마우스 이벤트를 처리할 콜백 함수 정의
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cam_obj = param["cam_instance"]
            output_w_eye = cam_obj.output_width // 2

            # 왼쪽 이미지 영역 내에서만 반응
            if x < output_w_eye:
                # 출력 이미지 좌표 (x, y)를 원본 캡처 이미지 좌표로 변환
                capture_w_eye = cam_obj.zed_width
                capture_h_eye = cam_obj.zed_height
                zoom = cam_obj.zoom_level

                if zoom <= 1.0:
                    # 줌이 없을 때: 단순 스케일링
                    scale_w = capture_w_eye / output_w_eye
                    scale_h = capture_h_eye / cam_obj.output_height
                    capture_x = int(x * scale_w)
                    capture_y = int(y * scale_h)
                else:
                    # 줌이 있을 때: 크롭 영역을 고려하여 좌표 계산
                    crop_w = capture_w_eye / zoom
                    crop_h = capture_h_eye / zoom

                    # 크롭 영역의 좌상단 좌표 (원본 기준)
                    crop_x1 = (capture_w_eye - crop_w) / 2
                    crop_y1 = (capture_h_eye - crop_h) / 2

                    # 클릭 지점의 좌표 계산
                    capture_x = int(crop_x1 + (x / output_w_eye) * crop_w)
                    capture_y = int(crop_y1 + (y / cam_obj.output_height) * crop_h)

                # 계산된 원본 좌표로 3D 포인트 요청
                point_3d = cam_obj.get_point_cloud_value(capture_x, capture_y)

                # 결과 저장
                param["clicked_point"] = (x, y)
                param["point_3d"] = point_3d


    # ✨ 3. 윈도우에 마우스 콜백 함수 등록
    cv2.setMouseCallback(window_name, mouse_callback, mouse_data)

    print("\n--- 조작법 ---")
    print("  마우스 왼쪽 클릭 : (왼쪽 이미지) 해당 지점의 3D 좌표 확인")
    print("  [+] / [=] : 줌 인 (Zoom In)")
    print("  [-] / [_] : 줌 아웃 (Zoom Out)")
    print("  [r] : 줌 리셋 (Zoom Reset)")
    print("  [q] : 종료 (Quit)")
    print("-----------------\n")

    try:
        while True:
            frame = cam.get_processed_frame()
            if frame is None:
                print("카메라 시작 대기 중...")
                time.sleep(0.5)
                continue

            # ✨ 4. 저장된 마우스 클릭 정보로 화면에 텍스트와 마커 표시
            point_3d = mouse_data["point_3d"]
            if point_3d is not None:
                depth_text = f"Clicked Depth (Z): {point_3d[2]:.2f} m"
                xyz_text = f"Clicked XYZ: [{point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f}]"
                # 클릭한 위치에 마커 그리기
                cv2.drawMarker(frame, mouse_data["clicked_point"], (0, 0, 255),
                               markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
            else:
                depth_text = "Click Left Image to get 3D point"
                xyz_text = ""

            cv2.putText(frame, depth_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, xyz_text, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key in [ord('+'), ord('=')]:
                cam.set_zoom(cam.zoom_level + 0.2)
            elif key in [ord('-'), ord('_')]:
                cam.set_zoom(cam.zoom_level - 0.2)
            elif key == ord('r'):
                cam.set_zoom(1.0)
                mouse_data["clicked_point"] = None  # 줌 리셋 시 마커도 초기화
                mouse_data["point_3d"] = None
    finally:
        print("프로그램을 종료합니다...")
        cam.stop()
        cv2.destroyAllWindows()