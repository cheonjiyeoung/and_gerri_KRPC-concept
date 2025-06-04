# 🎙 마이크 장치에서 음성 데이터를 읽는 로직
import pyaudio
import numpy as np
import asyncio

class AudioRecorder:
    def __init__(self, device_name=None):
        self.p = pyaudio.PyAudio()
        self.device_index = None

        # 🎯 마이크 정보 탐색
        self.mic_info = self.p.get_default_input_device_info()
        if device_name:
            for i in range(self.p.get_device_count()):
                info = self.p.get_device_info_by_index(i)
                if device_name.lower() in info["name"].lower():
                    self.mic_info = info
                    self.device_index = info["index"]
                    break
        else:
            self.device_index = self.mic_info["index"]

        self.sample_rate = int(self.mic_info["defaultSampleRate"])
        self.channels = min(2, self.mic_info["maxInputChannels"])
        self.frames_per_buffer = 960
        self.format = pyaudio.paInt16

        print(f"🎙 AudioRecorder Device: {self.mic_info['name']}")
        print(f"📊 sample_rate={self.sample_rate}, channels={self.channels}, frames_per_buffer={self.frames_per_buffer}")

        self.stream = self.p.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=self.frames_per_buffer
        )

    async def read(self):
        """오디오 데이터를 직접 읽어 비동기로 반환"""
        try:
            data = await asyncio.to_thread(self.stream.read, self.frames_per_buffer, exception_on_overflow=False)
            audio_data = np.frombuffer(data, dtype=np.int16).reshape(-1, self.channels)
            return audio_data
        except Exception as e:
            print(f"🎙️ 오디오 읽기 오류: {e}")
            return np.zeros((self.frames_per_buffer, self.channels), dtype=np.int16)

    def stop(self):
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.p:
            self.p.terminate()


# 🔊 스피커 장치로 오디오를 출력하는 재생기
class AudioPlayer:
    def __init__(self, device_name=None, volume=100, silence_threshold=50):
        self.p = pyaudio.PyAudio()
        self.device_index = None
        self.set_volume(volume)

        # 🔊 출력 장치 정보 탐색
        self.device_info = self.p.get_default_output_device_info()
        if device_name:
            for i in range(self.p.get_device_count()):
                info = self.p.get_device_info_by_index(i)
                if device_name.lower() in info["name"].lower():
                    self.device_info = info
                    self.device_index = info["index"]
                    break
        else:
            self.device_index = self.device_info["index"]

        self.sample_rate = int(self.device_info["defaultSampleRate"])
        self.channels = min(2, self.device_info["maxOutputChannels"])
        self.silence_threshold = silence_threshold  # 16-bit 음량 기준
        self.frames_per_buffer = 960
        self.format = pyaudio.paInt16

        print(f"🔊 AudioPlayer Device: {self.device_info['name']}")
        print(f"📊 sample_rate={self.sample_rate}, channels={self.channels}, frames_per_buffer={self.frames_per_buffer}")
        print(f"🔁 예상 output latency: {self.device_info.get('defaultHighOutputLatency', -1)*1000:.1f} ms")

        self.stream = self.p.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            output=True,
            output_device_index=self.device_index,
            frames_per_buffer=self.frames_per_buffer
        )

        self._task = None

    async def play(self, track):
        print("🎧 오디오 수신 + 실시간 조건부 재생 시작")

        async def loop():
            while True:
                try:
                    frame = await asyncio.wait_for(track.recv(), timeout=5.0)
                except asyncio.TimeoutError:
                    print("⚠️ recv 타임아웃")
                    continue
                except Exception as e:
                    print(f"❌ recv 오류: {e}")
                    break

                audio = frame.to_ndarray()

                # 음량 측정 후 조건부 재생
                peak = np.max(np.abs(audio))
                if peak < self.silence_threshold:
                    # print(f"🔇 무음/저음량 프레임 (peak={peak}) - 재생 삭제")
                    continue

                audio = (audio * self.volume).astype(np.int16)

                self.stream.write(audio.astype(np.int16).tobytes(), exception_on_underflow=False)

        self._task = asyncio.create_task(loop())

    def stop(self):
        if self._task:
            self._task.cancel()
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.p:
            self.p.terminate()


    def _normalize_volume(self, volume):
        """ 볼륨 값을 0.0 ~ 1.0 범위로 변환 """
        return max(0.0, min(1.0, volume / 100))

    def set_volume(self, volume):
        """ 볼륨을 설정 (0 ~ 100) """
        self.volume = self._normalize_volume(volume)
        print(f"🔊 Volume set to: {volume}%")