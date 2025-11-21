import ntplib
import time
from datetime import datetime, timezone, timedelta
from collections import deque

KST = timezone(timedelta(hours=9))  # 한국 시간대 (UTC+9)


class TimeSyncManager:
    def __init__(self, ntp_server="time.google.com", sample_count=5, latency_window_size=300):
        self.ntp_server = ntp_server
        self.sample_count = sample_count
        self.latency_window_size = latency_window_size
        self.latencies = deque(maxlen=latency_window_size)

        self.server_start_time, self.local_start_time, self.time_gap = self._get_corrected_server_time()

    def _get_corrected_server_time(self):
        try:
            client = ntplib.NTPClient()
            delays = []

            for _ in range(self.sample_count):
                t0 = time.monotonic()
                response = client.request(self.ntp_server, version=3)
                t1 = time.monotonic()

                delay = t1 - t0
                delays.append(delay)
                time.sleep(0.1)

            avg_delay = sum(delays) / len(delays)
            one_way_delay = avg_delay / 2

            server_utc = datetime.utcfromtimestamp(response.tx_time + one_way_delay).replace(tzinfo=timezone.utc)
            server_kst = server_utc.astimezone(KST)

            local_utc = datetime.utcnow().replace(tzinfo=timezone.utc)
            gap = server_kst - local_utc

            print_gap = f"-{abs(gap)}" if gap.total_seconds() < 0 else str(gap)

            print(f"🕒 NTP 보정 완료! 평균 왕복 지연: {avg_delay:.6f}s / 편도: {one_way_delay:.6f}s / 시간 오차: {print_gap}")
            return server_kst, local_utc, gap

        except Exception as e:
            print(f"[WARN] NTP 실패: {e}, 로컬 시간 사용")
            now = datetime.utcnow().replace(tzinfo=timezone.utc) + timedelta(hours=9)
            return now, now, timedelta(0)

    def get_scaled_server_time(self) -> datetime:
        elapsed = datetime.utcnow().replace(tzinfo=timezone.utc) - self.local_start_time
        return self.server_start_time + elapsed

    def record_latency(self, sent_time):
        """서버에서 받은 sent_time을 기반으로 지연 시간 기록"""
        if isinstance(sent_time, str):
            sent_time = datetime.fromisoformat(sent_time)
        elif not isinstance(sent_time, datetime):
            print(f"❌ 잘못된 시간 타입: {type(sent_time)}")
            return

        now = self.get_scaled_server_time()
        latency_ms = (now - sent_time).total_seconds() * 1000
        self.latencies.append(latency_ms)

    def get_latency_stats(self):
        """현재까지 측정된 평균 지연과 1% low 값을 반환 (ms)"""
        if not self.latencies:
            return None, None

        lat_list = list(self.latencies)
        avg = sum(lat_list) / len(lat_list)
        one_percent_index = max(1, int(len(lat_list) * 0.01))
        low_1p = sorted(lat_list)[-one_percent_index]  # 가장 느린 하위 1%
        return round(avg, 2), round(low_1p, 2)

    def timestamp(self, fmt: str = "%Y-%m-%d %H:%M:%S.%f", trim_micro=True) -> str:
        """서버 시간 기준의 포맷된 문자열 반환"""
        now = self.get_scaled_server_time()
        text = now.strftime(fmt)
        return text[:-3] if trim_micro else text



time_sync = TimeSyncManager()