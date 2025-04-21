import cv2
import numpy as np

class StereoVROption:
    def __init__(self,
                 interocular_adjust=0,
                 shift_left=None,
                 shift_right=None,
                 zoom_factor=1.0,
                 resize_width_px=None,
                 resize_height_px=None,
                 resize_scale_ratio=None,
                 target_aspect_ratio=None,
                 aspect_mode="resize"):
        self.interocular_adjust = interocular_adjust
        self.shift_left = shift_left
        self.shift_right = shift_right
        self.zoom_factor = zoom_factor
        self.resize_width_px = resize_width_px
        self.resize_height_px = resize_height_px
        self.resize_scale_ratio = resize_scale_ratio
        self.target_aspect_ratio = target_aspect_ratio
        self.aspect_mode = aspect_mode

        
    def get_shift_values(self):
        if self.shift_left is not None and self.shift_right is not None:
            return self.shift_left, self.shift_right
        else:
            return -self.interocular_adjust, self.interocular_adjust


class StereoVRProcessor:
    def __init__(self, source, option: StereoVROption = StereoVROption()):
        self.source = source
        self.option = option

    def get_frame(self):
        if hasattr(self.source, "get_frame") and callable(self.source.get_frame):
            frame = self.source.get_frame()
        elif hasattr(self.source, "last_frame"):
            frame = self.source.last_frame
        else:
            raise ValueError("Source must have .get_frame() or .last_frame")

        if frame is None:
            return None

        return self.apply(frame)

    def apply(self, stereo_frame: np.ndarray) -> np.ndarray:
        h, w = stereo_frame.shape[:2]
        w_half = w // 2
        left = stereo_frame[:, :w_half]
        right = stereo_frame[:, w_half:]

        left = self._resize(left)
        right = self._resize(right)

        left, right = self._adjust_interocular_distance(left, right)

        shift_l, shift_r = self.option.get_shift_values()
        left = self._shift(left, shift_l)
        right = self._shift(right, shift_r)

        left = self._zoom(left, self.option.zoom_factor)
        right = self._zoom(right, self.option.zoom_factor)

        combined = np.hstack((left, right))

        combined = self._apply_aspect_ratio(combined)

        return combined

    def _apply_aspect_ratio(self, frame):
        if self.option.aspect_mode == "resize":
            return self._adjust_aspect_ratio_resize(frame)
        elif self.option.aspect_mode == "crop":
            return self._adjust_aspect_ratio_crop(frame)
        else:
            return frame


    def _adjust_aspect_ratio_crop(self, frame):
        opt = self.option
        if not opt.target_aspect_ratio:
            return frame
        h, w = frame.shape[:2]
        try:
            aw, ah = map(int, opt.target_aspect_ratio.split(":"))
            target_ratio = aw / ah
            current_ratio = w / h

            if current_ratio > target_ratio:
                new_w = int(h * target_ratio)
                x1 = (w - new_w) // 2
                return frame[:, x1:x1 + new_w]
            elif current_ratio < target_ratio:
                new_h = int(w / target_ratio)
                y1 = (h - new_h) // 2
                return frame[y1:y1 + new_h, :]
            else:
                return frame
        except:
            return frame

    def _resize(self, frame):
        opt = self.option
        h, w = frame.shape[:2]

        if opt.resize_width_px and opt.resize_height_px:
            return cv2.resize(frame, (opt.resize_width_px, opt.resize_height_px))
        elif opt.resize_scale_ratio:
            new_w = int(w * opt.resize_scale_ratio)
            new_h = int(h * opt.resize_scale_ratio)
            return cv2.resize(frame, (new_w, new_h))
        else:
            return frame

    def _adjust_interocular_distance(self, left, right):
        dx = self.option.interocular_adjust

        def crop_or_pad(frame, dx, side):
            h, w = frame.shape[:2]
            if dx > 0:
                pad = np.zeros((h, dx, 3), dtype=np.uint8)
                return np.hstack((pad, frame)) if side == 'left' else np.hstack((frame, pad))
            elif dx < 0:
                crop = abs(dx)
                if w <= crop:
                    return frame
                cropped = frame[:, crop:] if side == 'left' else frame[:, :-crop]
                # 👉 crop 후 다시 원래 width로 리사이즈 (비율 유지)
                return cv2.resize(cropped, (w, h))
            else:
                return frame

        return crop_or_pad(left, dx, 'left'), crop_or_pad(right, dx, 'right')
    
    def _adjust_aspect_ratio_resize(self, frame):
        opt = self.option
        if not opt.target_aspect_ratio:
            return frame

        h, w = frame.shape[:2]
        try:
            aw, ah = map(int, opt.target_aspect_ratio.split(":"))
            target_ratio = aw / ah
            current_ratio = w / h

            if abs(current_ratio - target_ratio) < 0.01:
                return frame

            # 새 크기 계산
            if current_ratio > target_ratio:
                # 너무 넓다 → 높이를 줄임
                new_h = int(w / target_ratio)
                return cv2.resize(frame, (w, new_h))
            else:
                # 너무 좁다 → 폭을 줄임
                new_w = int(h * target_ratio)
                return cv2.resize(frame, (new_w, h))

        except:
            return frame

    def _shift(self, frame: np.ndarray, dx: int) -> np.ndarray:
        if dx == 0:
            return frame
        h, w = frame.shape[:2]
        M = np.float32([[1, 0, dx], [0, 1, 0]])
        return cv2.warpAffine(frame, M, (w, h))

    def _zoom(self, frame: np.ndarray, factor: float) -> np.ndarray:
        if factor == 1.0:
            return frame

        h, w = frame.shape[:2]
        new_w = int(w / factor)
        new_h = int(h / factor)

        resized = cv2.resize(frame, (new_w, new_h))

        if factor > 1.0:
            # 확대: 중앙 crop (기존과 동일)
            x1 = (w - new_w) // 2
            y1 = (h - new_h) // 2
            cropped = frame[y1:y1 + new_h, x1:x1 + new_w]
            return cv2.resize(cropped, (w, h))
        else:
            # 축소: 중앙 배치 + 패딩
            pad_left = (w - new_w) // 2
            pad_right = w - new_w - pad_left
            pad_top = (h - new_h) // 2
            pad_bottom = h - new_h - pad_top
            return cv2.copyMakeBorder(
                resized, pad_top, pad_bottom, pad_left, pad_right,
                borderType=cv2.BORDER_CONSTANT, value=[0, 0, 0]
            )