import asyncio
import os, sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), '../..')))

from pubsub import pub

SERVER_HOST = '175.126.123.199'
SERVER_PORT = 9980

class AdaptiveNetworkDaemon:
    def __init__(self, operator_info, network='ketirtc', command=None, video=None, audio=None):
        self.operator_info = operator_info
        self.network = network
        self.command = command
        self.video = video
        self.audio = audio

        self.channels = self._setup_channels()

    def _setup_channels(self):
        if self.network == "ketirtc":
            from _and_.keti_rtc import TODO_webrtc_operator as backend

        else:
            return


        return backend.create_channels(
            operator_info=self.operator_info,
            command=self.command,
            video_info=self.video,
            audio_info=self.audio
        )

    def connect(self):
        for ch in self.channels.values():
            ch.connect()

    def run_forever(self):
        print('✅ AdaptiveNetworkDaemon is running...')
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print('🛑 Stopped by user.')


if __name__ == '__main__':
    from hello_universe_config import OPERATOR_INFO, VIDEO_INFO, AUDIO_INFO

    daemon = AdaptiveNetworkDaemon(
        operator_info=OPERATOR_INFO,
        network='ketirtc',
        command='command',
        # video=VIDEO_INFO,
        # audio=AUDIO_INFO
    )

    daemon.connect()
    daemon.run_forever()
