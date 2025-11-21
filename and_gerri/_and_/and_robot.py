import time

class AdaptiveNetworkDaemon:
    def __init__(self, robot_info, network="ketirtc", command=None, video_info=None, audio_info=None, server_info=None):
        self.robot_info = robot_info
        self.network = network
        self.command = command
        self.video_info = video_info
        self.audio_info = audio_info
        self.server_info = server_info

        self.channels = self._setup_channels()

    def _setup_channels(self):
        if self.network == "ketirtc":
            from _and_.keti_rtc.robot import webrtc_robot as backend
            return backend.create_channels(
                robot_info=self.robot_info,
                command=self.command,
                video_info=self.video_info,
                audio_info=self.audio_info
            )
        elif self.network == "rtc2kng":
            from _and_.rtc2kng import rtc2kng_robot as backend
            return backend.create_channels(
                robot_info=self.robot_info,
                server_info=self.server_info,
                video_info=self.video_info,
                audio_info=self.audio_info
            )
        else:
            raise ValueError(f"Unsupported network backend: {self.network}")


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
        video_info=VIDEO_INFO,
        audio_info=AUDIO_INFO
    )

    daemon.connect()
    daemon.run_forever()