import fractions
from aiortc import AudioStreamTrack
from av import AudioFrame

class AudioSendTrack(AudioStreamTrack):
    def __init__(self, recorder):
        super().__init__()
        self.recorder = recorder  # AudioRecorder 인스턴스
        self._timestamp = 0
        # recorder.sample_rate가 int이므로 Fraction 생성 시 분모로 사용 가능
        self._time_base = fractions.Fraction(1, int(recorder.sample_rate))


    async def recv(self):
        audio_data = await self.recorder.read() # audio_data는 (frames_per_buffer, channels) 형태의 int16 NumPy 배열

        if self.recorder.channels == 1:
            layout = "mono"
        elif self.recorder.channels == 2:
            layout = "stereo"
        else:
            # AudioRecorder에서 channels는 최대 2이므로 이 경우는 발생하지 않음
            layout = f"channel{self.recorder.channels}"

        try:
            # audio_data.shape[0] (즉, frames_per_buffer)를 사용하는 것이 명확합니다.
            frame = AudioFrame(format="s16", layout=layout, samples=audio_data.shape[0])
            frame.sample_rate = self.recorder.sample_rate
            frame.pts = self._timestamp
            frame.time_base = self._time_base

            # 👇 ***** 중요: 오디오 데이터를 프레임에 채워 넣습니다. *****
            frame.planes[0].update(audio_data.tobytes())

            self._timestamp += audio_data.shape[0] # frames_per_buffer 대신 실제 프레임의 샘플 수 사용

            # 아래 print 문은 디버깅 시 유용하지만, 실제 운영 시에는 매우 많은 로그를 생성하므로 주석 처리하거나 제거하는 것이 좋습니다.
            # print(f"AudioSendTrack: sr={frame.sample_rate}, pts={frame.pts}, tb={frame.time_base}, layout={frame.layout}, samples={frame.samples}")
            return frame
        except Exception as e:
            print(f"오디오 프레임 생성 오류: {e}")
            # 오류 발생 시 빈 프레임 대신 None을 반환하거나,
            # aiortc가 기대하는 방식으로 오류를 처리해야 할 수 있습니다.
            # 빈 프레임을 반환해야 한다면, 오류 없는 빈 프레임을 생성해야 합니다.
            # 여기서는 일단 None으로 두겠습니다.
            return None