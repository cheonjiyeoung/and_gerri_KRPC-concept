import time

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

class AdaptiveNetworkDaemon:
    def __init__(self, robot_id, backend="ketirtc", command=None, video=None, audio=None):
        self.robot_id = robot_id
        self.backend = backend

        self.command_channels = {}
        self.video_channels = {}
        self.audio_channels = {}

        # backend별 채널 생성기 로드
        self.backend_module = self._load_backend_module()
        self._setup_channels(command, video, audio)

    def _load_backend_module(self):
        if self.backend == "ketirtc":
            from _and_.keti_rtc import webrtc_robot as backend
        elif self.backend == "zeromq":
            from _and_.zeromq import zeromq_robot as backend
        else:
            raise ValueError(f"Unsupported backend: {self.backend}")
        return backend

    def _setup_channels(self, command, video, audio):
        self.command_channels = self.backend_module.create_command_channels(self.robot_id, command)
        self.video_channels = self.backend_module.create_video_channels(self.robot_id, video)
        self.audio_channels = self.backend_module.create_audio_channels(self.robot_id, audio)

    def connect(self):
        for ch in self.command_channels.values(): ch.connect()
        for ch in self.video_channels.values(): ch.connect()
        for ch in self.audio_channels.values(): ch.connect()

    def run_forever(self):
        print("✅ AdaptiveNetworkDaemon is running...")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("🛑 Stopped by user.")


if __name__ == "__main__":
    from _and_.keti_rtc.cam_manager import CameraManager
    from _and_.keti_rtc.video_streamer import VideoStreamer

    from gerri.robot.gerri_config import ROBOT_ID

    # 🎥 카메라 구성
    cam = CameraManager(camera_name="front_cam", camera_index=0, width=1920, height=1080, fps=30)
    cam.start()

    streamer = VideoStreamer(cam)

    # 🚀 데몬 실행
    daemon = AdaptiveNetworkDaemon(
        robot_id=ROBOT_ID,
        backend="ketirtc",
        command="command",
        video=streamer,
        audio="audio"
    )

    daemon.connect()
    daemon.run_forever()