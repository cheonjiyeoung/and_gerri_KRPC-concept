import threading
import ntplib
import time
import uuid
from pubsub import pub
from datetime import datetime, timezone, timedelta
from time_sync_manager import TimeSyncManager

TimeSync = TimeSyncManager(sample_count=5)

class StatusManager:
    def __init__(self, name, category, model, controller, interval=0.1):
        self.name = name
        self.category = category
        self.model = model
        self.controller = controller
        self.interval = interval
        self.status = vars(self.controller)

        # ✅ 서버 시간 & 로컬 기준 시간 저장 (보정 포함)
        self.robot_info = self.get_robot_status()

        self.start_status_loop()

    def get_robot_status(self):
        if hasattr(self.controller, 'update_status'):
            self.controller.update_status()
        robot_status = vars(self.controller)
        robot_info = {
            'robot_name': self.name,
            'robot_type': {'category': self.category, 'model': self.model}
        }
        # MOBILE
        if 'pose' in robot_status:
            robot_info['pose'] = robot_status['pose']
        if 'battery_state' in robot_status:
            robot_info['battery'] = robot_status['battery']
        if 'robot_shape' in robot_status:
            robot_info['robot_shape'] = robot_status['robot_shape']
        if 'wheel_state' in robot_status:
            robot_info['wheel_state'] = robot_status['wheel_state']

        # MANIPULATOR

        if 'joint_state' in robot_status:
            robot_info['joint_state'] = robot_status['joint_state']
        return robot_info


    def send_status(self):
        """📡 현재 로봇 상태 + KST 기준 보정된 현재 시간 포함"""
        robot_status = {
            'robot_info': self.robot_info,
            'metadata': {
                'timestamp': TimeSync.get_scaled_server_time(),  # KST 기준 보정된 현재 시간
                'uuid': str(uuid.uuid4()),  # 메시지 고유 식별자
            },
        }

        status_message = {
            'topic': 'robot_status',
            'value': robot_status
        }

        pub.sendMessage('send_message', message=status_message)

        return status_message

    def start_status_loop(self):
        """⏳ 일정 시간마다 status를 업데이트하여 전송하는 루프"""
        def loop():
            while True:
                self.robot_info = self.get_robot_status()  # 최신 상태 갱신
                # self.send_status()  # 상태 전송
                time.sleep(self.interval)  # 주어진 간격만큼 대기

        # 백그라운드에서 실행될 스레드 생성
        thread = threading.Thread(target=loop, daemon=True)
        thread.start()


# ✅ 테스트 실행 부분 (Main)
if __name__ == "__main__":
    class DummyController:
        """가짜 컨트롤러 객체 (테스트용)"""
        pose = [0.0, 0.0, 0.0]
        battery = 100
        robot_shape = "quadruped"
        wheel_state = "stopped"
        joint_state = [0.0] * 6


    # ✅ `StatusManager` 인스턴스 생성
    robot = StatusManager(name="Piper", category="quadruped", model="Piper-X", controller=DummyController())

    print("✅ 서버 기반 KST 시간 보정 테스트 시작!")
    for i in range(3):  # 3번 반복
        status = robot.send_status()
        print(status)
        print(f"🔹 [{i + 1}] 상태 메시지 전송: {status['value']['metadata']['timestamp']} (UUID: {status['value']['metadata']['uuid']})")
        time.sleep(2)  # 2초 대기
