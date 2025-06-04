import asyncio
import numpy as np
import cv2
from video_manager import VideoManager

class RealSenseCameraManager(VideoManager):
    def __init__(self, mode="color", width=640, height=480, fps=30, debug=False):
        super().__init__(width=width, height=height, fps=fps, debug=debug)
        self.mode = mode
        self.pipeline = None
        self.last_frame = None
        self.align = None

    async def start(self):
        import pyrealsense2 as rs
        self.pipeline = rs.pipeline()
        config = rs.config()

        if self.mode == "color":
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        elif self.mode == "depth":
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        elif self.mode == "infrared":
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(rs.stream.infrared, self.width, self.height, rs.format.y8, self.fps)
        elif self.mode == "depth-ir":
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.infrared, self.width, self.height, rs.format.y8, self.fps)
        else:
            raise ValueError(f"Unsupported mode: {self.mode}")

        try:
            self.pipeline.start(config)
            if self.mode in ["depth-ir"]:
                self.align = rs.align(rs.stream.color)
        except Exception as e:
            print(f"❌ RealSense 시작 실패: {e}")
            return

        asyncio.create_task(self._run_loop())

    async def _run_loop(self):
        import pyrealsense2 as rs
        print(f"✅ RealSense {self.mode} 루프 시작됨")

        try:
            while True:
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                except RuntimeError as e:
                    print(f"⚠️ 프레임 수신 실패: {e}")
                    await asyncio.sleep(0.1)
                    continue

                if self.mode == "depth-ir" and self.align:
                    aligned_frames = self.align.process(frames)
                    depth_frame = aligned_frames.get_depth_frame()
                    ir_frame = aligned_frames.get_infrared_frame()

                    if depth_frame and ir_frame:
                        depth_image = np.asanyarray(depth_frame.get_data())
                        ir_image = np.asanyarray(ir_frame.get_data())

                        ir_colored = cv2.applyColorMap(
                            cv2.convertScaleAbs(ir_image, alpha=0.03),
                            cv2.COLORMAP_BONE
                        )
                        depth_colormap = cv2.applyColorMap(
                            cv2.convertScaleAbs(depth_image, alpha=0.03),
                            cv2.COLORMAP_JET
                        )

                        self.last_frame = np.hstack((ir_colored, depth_colormap))

                elif self.mode == "color":
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        self.last_frame = np.asanyarray(color_frame.get_data())

                elif self.mode == "depth":
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        depth_image = np.asanyarray(depth_frame.get_data())
                        self.last_frame = cv2.applyColorMap(
                            cv2.convertScaleAbs(depth_image, alpha=0.03),
                            cv2.COLORMAP_JET
                        )

                elif self.mode == "infrared":
                    ir_frame = frames.get_infrared_frame()
                    if ir_frame:
                        ir_image = np.asanyarray(ir_frame.get_data())
                        self.last_frame = cv2.applyColorMap(
                            cv2.convertScaleAbs(ir_image, alpha=0.03),
                            cv2.COLORMAP_BONE
                        )

                if self.debug and self.last_frame is not None:
                    cv2.imshow(f"DEBUG - RealSense ({self.mode})", self.last_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                await asyncio.sleep(1 / self.fps)

        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

    def get_last_frame(self):
        return self.last_frame

    async def stop(self):
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None
            print(f"🛑 RealSense 정지됨")
