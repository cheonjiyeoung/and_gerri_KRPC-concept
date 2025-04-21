import asyncio
import websockets
import random
import json
import logging
from typing import Dict, Any, Optional
from pubsub import pub
import os,sys
CURRENT_FILE = os.path.abspath(__file__)
VENV_DIR = os.path.dirname(sys.executable) 
PROJECT_ROOT = os.path.abspath(os.path.join(VENV_DIR, "../.."))
sys.path.insert(0, PROJECT_ROOT)
from avatar_darm.robot.network_tools.solar_robot.config import config

# 로거 설정
logger = logging.getLogger('kesiroma_robot_client')
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
logger.addHandler(handler)

class SOLAR_ROBOT_CLIENT:
    """SOLAR API에 연결하는 로봇 클라이언트 클래스"""
    
    def __init__(self, robot_id: Optional[str] = None, robot_type: str = "mobile"):
        """
        로봇 클라이언트 초기화
        
        Args:
            robot_id: 로봇 ID (기본값: None, 자동 생성)
            robot_type: 로봇 타입 (기본값: "mobile")
        """
        self.registered = False
        self.robot_id = robot_id or f"gyd_mobile_{random.randint(0, 10)}"
        self.robot_type = robot_type
        
        # 로봇 상태 초기화
        self.status = {
            "robot_id": self.robot_id,
            "type": robot_type,
            "pose": {"x": 0, "y": 0, "th": 0},
            "velo": {"vx": 0, "vth": 0},
            "status": "Initializing...",
            "battery": {"percent": 100, "now_charging": False}
        }
        
        # 상태 업데이트 구독
        pub.subscribe(self.update, "kesiroma")
        logger.info(f"로봇 클라이언트 초기화: ID={self.robot_id}, Type={robot_type}")

    async def main_loop(self) -> None:
        """메인 루프 - WebSocket 연결 및 메시지 처리"""
        uri = f"ws://{config.api.HOST}:{config.api.PORT}/robot"
        logger.info(f"서버에 연결 시도: {uri}")
        
        while True:
            try:
                async with websockets.connect(uri) as websocket:
                    logger.info("서버에 연결됨")
                    
                    # 등록 처리
                    if not self.registered:
                        await self._register(websocket)
                    
                    # 등록 성공 시 메시지 처리 시작
                    if self.registered:
                        await asyncio.gather(
                            self.received_handler(websocket),
                            self.status_send(websocket)
                        )
            except Exception as e:
                logger.error(f"연결 오류: {e}")
            
            # 재연결 전 대기
            await asyncio.sleep(1)
            logger.info("재연결 시도 중...")

    async def _register(self, websocket) -> None:
        """로봇 등록 처리"""
        try:
            register_msg = {"id": self.robot_id}
            await websocket.send(json.dumps(register_msg))
            response = await websocket.recv()
            
            if "register ok" in response:
                self.registered = True
                logger.info(f"로봇 등록 성공: {self.robot_id}")
            else:
                logger.error(f"로봇 등록 실패: {response}")
        except Exception as e:
            logger.error(f"등록 중 오류 발생: {e}")

    async def received_handler(self, websocket) -> None:
        """서버로부터 메시지 수신 처리"""
        while True:
            try:
                message = await websocket.recv()
                message_dict = json.loads(message)
                pub.sendMessage("receive_message", message=message_dict)
                logger.debug(f"메시지 수신: {message_dict}")
            except json.JSONDecodeError:
                # logger.warning(f"잘못된 JSON 형식: {message}")
                pass
            except Exception as e:
                logger.error(f"메시지 수신 중 오류: {e}")
                break

    async def status_send(self, websocket) -> None:
        """로봇 상태 전송"""
        while True:
            try:
                if self.registered:
                    message_dict = {
                        "topic": "status",
                        "value": self.status
                    }
                    await websocket.send(json.dumps(message_dict))
                    logger.debug(f"상태 전송: {self.status}")
                await asyncio.sleep(0.5)  # 상태 업데이트 간격
            except Exception as e:
                logger.error(f"상태 전송 중 오류: {e}")
                break

    def update(self, value: Dict[str, Any]) -> None:
        # print("Aaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        """
        로봇 상태 업데이트
        
        Args:
            value: 업데이트할 상태 정보
        """
        if "pose" in value:
            self.status["pose"] = value["pose"]
        elif "velo" in value:
            self.status["velo"] = value["velo"]
        elif "status" in value:
            self.status["status"] = value["status"]
        elif "battery" in value:
            self.status["battery"] = value["battery"]
        
        logger.debug(f"상태 업데이트: {value}")

async def main():
    """메인 함수"""
    client = KESIROMA_ROBOT_CLIENT()
    await client.main_loop()

if __name__ == "__main__":
    asyncio.run(main())
    