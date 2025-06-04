import time

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

class AdaptiveNetworkDaemon:
    def __init__(self, robot_info, network="ketirtc", command=None, video=None, audio=None):
        self.robot_info = robot_info
        self.network = network
        self.command = command
        self.video = video
        self.audio = audio

        self.channels = self._setup_channels()

    def _setup_channels(self):
        if self.network == "ketirtc":
            from _and_.keti_rtc import webrtc_robot as backend
        elif self.network == "rtc2kng":
            from _and_.rtc2kng import rtc2kng_robot as backend
        elif self.network == "zeromq":
            from _and_.zeromq import zeromq_robot as backend
        else:
            raise ValueError(f"Unsupported network backend: {self.network}")

        return backend.create_channels(
            robot_info=self.robot_info,
            command=self.command,
            video_info=self.video,
            audio_info=self.audio
        )

    def connect(self):
        for ch in self.channels.values():
            ch.connect()

    def run_forever(self):
        print("✅ AdaptiveNetworkDaemon is running...")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("🛑 Stopped by user.")



if __name__ == "__main__":
    from hello_universe_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO

    # 🚀 데몬 실행
    daemon = AdaptiveNetworkDaemon(
        robot_info=ROBOT_INFO,
        network='ketirtc',
        command="command",
        video=VIDEO_INFO,
        audio=AUDIO_INFO
    )

    daemon.connect()
    daemon.run_forever()