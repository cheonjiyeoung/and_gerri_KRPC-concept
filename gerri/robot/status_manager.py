import threading
import ntplib
import time
import uuid
from pubsub import pub

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))

from utils.time_sync_manager import time_sync

print(time_sync.timestamp())

def timestamp():
    return time_sync.timestamp()

class StatusManager:
    def __init__(self, robot_info, controller, interval=1):
        self.id = robot_info['id']
        self.category = robot_info['category']
        self.model = robot_info['model']
        self.controller = controller
        self.interval = interval
        self.status = vars(self.controller)

        # ✅ 서버 시간 & 로컬 기준 시간 저장 (보정 포함)
        self.robot_info = self.get_robot_status()

        self.start_status_loop()

    def get_robot_status(self):
        robot_status:dict = vars(self.controller.status)
        robot_info = {
            'robot_id': self.id,
            'robot_type': {'category': self.category, 'model': self.model}
        }

        for key in robot_status.keys():
            if key in robot_status:
                robot_info[key] = robot_status[key]

        return robot_info


    def send_status(self):
        """📡 현재 로봇 상태 + KST 기준 보정된 현재 시간 포함"""
        robot_status = {
            'robot_info': self.robot_info,
            'metadata': {
                'timestamp': timestamp(),  # KST 기준 보정된 현재 시간
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
                self.send_status()  # 상태 전송
                time.sleep(self.interval)  # 주어진 간격만큼 대기

        # 백그라운드에서 실행될 스레드 생성
        thread = threading.Thread(target=loop, daemon=True)
        thread.start()

