import time
import cv2
import logging
import threading
import platform
import numpy as np

import os, sys
# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
# from _and_.rtc2kng.video_manager import VideoManager

logger = logging.getLogger(__name__)

# VideoManager 클래스가 없으므로 임시로 생성합니다.
class VideoManager:
    def __init__(self, **kwargs):
        self.width = kwargs.get('width', 1920)
        self.height = kwargs.get('height', 1080)
        self.fps = kwargs.get('fps', 30)
        self.last_frame = None
        self.last_frame_time = 0
        self.debug = kwargs.get('debug', False)

class CameraManager(VideoManager):
    def __init__(self, index=0, name='cam', **kwargs):
        # 캡처할 원본 해상도와 최종 출력 해상도를 구분합니다.
        # capture_width/height: 카메라에서 가져올 최대 해상도
        # output_width/height: 화면에 표시하거나 스트리밍할 최종 해상도
        self.capture_width = kwargs.get('capture_width', 3840)
        self.capture_height = kwargs.get('capture_height', 1080)
        self.output_width = kwargs.get('output_width', 1280)
        self.output_height = kwargs.get('output_height', 360)
        
        # VideoManager의 width, height를 출력 해상도로 설정합니다.
        kwargs['width'] = self.output_width
        kwargs['height'] = self.output_height
        
        super().__init__(**kwargs)
        self.index = index
        self.name = name

        self.running = False
        self.high_res_frame = None # 원본 고해상도 프레임을 저장할 변수

        # 줌 관련 상태 변수
        self.zoom_level = 1.0 # 1.0 = 줌 없음
        self.zoom_center = None # (x, y) 줌 중심 좌표 (원본 프레임 기준)
        
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
        self.cap = self._open_capture(self.index)
        
        # 카메라에 최대 해상도 설정을 '요청'합니다.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # 실제 적용된 해상도를 다시 확인하여 저장합니다. (카메라가 지원하지 않을 수 있음)
        self.actual_capture_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_capture_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        self.print_feature()

    def _capture_loop(self):
        while self.running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    raise RuntimeError("❌ 프레임 읽기 실패")
                # 읽어온 프레임을 고해상도 원본 프레임 변수에 저장
                self.high_res_frame = frame 
                self.last_frame_time = time.time()
                time.sleep(1 / self.fps) # FPS에 맞춰 대기 시간 조절
            except Exception as e:
                logger.error(f"⚠️ 캡처 오류 발생: {e}")
                self.restart()

    # ✨✨✨ 새로운 핵심 기능: 줌 설정 메소드 ✨✨✨
    def set_zoom(self, level=1.0, center=None):
        """
        줌 레벨과 중심점을 설정합니다.
        level: 줌 배율 (1.0 이상)
        center: 줌 중심 좌표 (x, y). None이면 프레임 중앙.
        """
        self.zoom_level = max(1.0, level) # 줌 레벨은 1.0 이상이어야 함
        if center:
            # 입력된 좌표를 실제 원본 해상도 내의 좌표로 변환
            self.zoom_center = (
                np.clip(center[0], 0, self.actual_capture_width),
                np.clip(center[1], 0, self.actual_capture_height)
            )
        else:
            self.zoom_center = (self.actual_capture_width / 2, self.actual_capture_height / 2)
        print(f"🔎 줌 설정: Level={self.zoom_level:.2f}, Center={self.zoom_center}")

    # ✨✨✨ 새로운 핵심 기능: 처리된 프레임 반환 메소드 ✨✨✨
    def get_processed_frame(self):
        """
        줌 레벨에 따라 처리된 최종 출력 프레임을 반환합니다.
        """
        if self.high_res_frame is None:
            return None

        if self.zoom_level <= 1.0:
            # 줌이 없는 경우: 고해상도 원본을 출력 크기로 다운스케일
            return cv2.resize(self.high_res_frame, (self.output_width, self.output_height), interpolation=cv2.INTER_AREA)
        else:
            # 줌이 있는 경우: 무손실 디지털 줌 처리
            # 1. 원본에서 잘라낼 영역(ROI)의 크기 계산
            crop_w = int(self.actual_capture_width / self.zoom_level)
            crop_h = int(self.actual_capture_height / self.zoom_level)

            # 2. 잘라낼 영역의 좌상단(x1, y1) 좌표 계산
            center_x, center_y = self.zoom_center
            x1 = int(center_x - crop_w / 2)
            y1 = int(center_y - crop_h / 2)

            # 3. 좌표가 프레임 밖으로 나가지 않도록 보정
            x1 = np.clip(x1, 0, self.actual_capture_width - crop_w)
            y1 = np.clip(y1, 0, self.actual_capture_height - crop_h)
            
            x2 = x1 + crop_w
            y2 = y1 + crop_h
            
            # 4. 원본에서 해당 영역을 잘라내기(Crop)
            cropped_frame = self.high_res_frame[y1:y2, x1:x2]

            # 5. 잘라낸 영역을 최종 출력 크기로 리사이즈
            return cv2.resize(cropped_frame, (self.output_width, self.output_height), interpolation=cv2.INTER_LINEAR)


    def print_feature(self):
        print("🎥 카메라 속성 정보")
        print(f" - Capture Resolution (Actual): {self.actual_capture_width} x {self.actual_capture_height} px")
        print(f" - Output Resolution: {self.output_width} x {self.output_height} px")
        print(f" - FPS: {self.cap.get(cv2.CAP_PROP_FPS)}")
        fourcc_int = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4)])
        print(f" - FourCC: {fourcc_str}")
        # ... (기존 코드와 동일)

    def restart(self):
        self.stop()
        time.sleep(1)
        self.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        logger.info("🛑 카메라 중지됨")


# --- 사용 예제 ---
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    
    # 카메라 초기화
    # capture_width/height: 카메라에서 가져올 원본 해상도 (카메라 성능에 맞게 조절)
    # output_width/height: 우리가 최종적으로 보고자 하는 화면 해상도
    cam = CameraManager(
        index=0, 
        capture_width=1920, 
        capture_height=1080, 
        output_width=1280,
        output_height=720,
        fps=30
    )
    
    window_name = "Lossless Digital Zoom"
    cv2.namedWindow(window_name)

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 마우스 클릭 위치를 줌 중심으로 설정 (출력 화면 기준 좌표 -> 원본 영상 기준 좌표로 변환)
            cam_instance = param['cam']
            current_zoom = cam_instance.zoom_level

            # 현재 줌 상태를 고려하여 원본 프레임에서의 클릭 위치를 계산
            # 이 부분은 get_processed_frame의 역연산과 유사합니다.
            if current_zoom <= 1.0:
                 center_x = int(x * (cam_instance.actual_capture_width / cam_instance.output_width))
                 center_y = int(y * (cam_instance.actual_capture_height / cam_instance.output_height))
            else:
                # 줌 상태일때는 현재 보이는 화면의 좌상단 좌표를 먼저 계산해야 함
                crop_w = int(cam_instance.actual_capture_width / current_zoom)
                crop_h = int(cam_instance.actual_capture_height / current_zoom)
                
                center_x_on_capture, center_y_on_capture = cam_instance.zoom_center
                
                x1 = int(center_x_on_capture - crop_w / 2)
                y1 = int(center_y_on_capture - crop_h / 2)
                
                # 현재 보이는 화면(크롭된 영역) 내에서의 클릭 좌표 계산
                center_x = x1 + int(x * (crop_w / cam_instance.output_width))
                center_y = y1 + int(y * (crop_h / cam_instance.output_height))

            # 새로운 줌 레벨과 계산된 중심으로 줌 설정
            new_zoom = current_zoom + 0.5
            cam_instance.set_zoom(level=new_zoom, center=(center_x, center_y))
            
    cv2.setMouseCallback(window_name, mouse_callback, param={'cam': cam})
    
    print("\n--- 조작법 ---")
    print("마우스 좌클릭: 해당 위치를 중심으로 0.5배씩 줌 인")
    print("키보드 +/= : 줌 인")
    print("키보드 -/_ : 줌 아웃")
    print("키보드 r : 줌 리셋")
    print("키보드 q : 종료")
    
    try:
        while True:
            # 직접 high_res_frame에 접근하는 대신, get_processed_frame()을 호출
            frame = cam.get_processed_frame()
            if frame is None:
                time.sleep(0.1)
                continue
            
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('+') or key == ord('='):
                cam.set_zoom(level=cam.zoom_level + 0.5) # 중심은 현재 중심으로 유지
            elif key == ord('-') or key == ord('_'):
                cam.set_zoom(level=cam.zoom_level - 0.5)
            elif key == ord('r'):
                cam.set_zoom(level=1.0) # 줌 리셋
                
    finally:
        cam.stop()
        cv2.destroyAllWindows()
        print("프로그램 종료.")