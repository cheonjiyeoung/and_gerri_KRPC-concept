import asyncio
import json
import websockets
from pubsub import pub
import os
import sys
import time
import requests

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.gyd_mobile.gyd_robot_config import SOLAR_INFO
from gerri.robot.examples.gyd_mobile.gyd_sub_controller import GydSubController

class SolarClient:
    def __init__(self, sub_controller):
        self.sub_controller: GydSubController = sub_controller
        self.websocket = None
        self.registered = False
        self.ccs_coords = SOLAR_INFO["ccs_coords"]

        self.server_host = SOLAR_INFO["server_host"]["addr"]
        self.server_port = SOLAR_INFO["server_host"]["port"]
        self.task_list = []
        pub.subscribe(self.send_message,"solar_send")
        

    async def main_loop(self) -> None:
        """메인 루프 - WebSocket 연결 및 메시지 처리"""
        url = f"ws://{self.server_host}:{self.server_port}/connect/robot"
        
        while True:
            try:
                async with websockets.connect(url, ping_timeout=1800) as websocket:       
                    # 등록 처리
                    if not self.registered:
                        payload = {
                            "robot_id": "gyd_mobile_01",
                            "robot_model": "gyd_mobile",
                            "css_coords" : self.ccs_coords
                        }
                        print(f"\n\n CCS_COORDS : {self.ccs_coords} \n\n")
                        await websocket.send(json.dumps(payload))
                        response = await websocket.recv()
                        response_dict = json.loads(response)
                        if response_dict.get("code") == "001":
                            self.registered = True
                        else:
                            continue
                    
                    # 등록 성공 시 메시지 처리 시작
                    if self.registered:
                        self.websocket = websocket
                        receive_task = asyncio.create_task(self.received_handler(websocket))
                        status_task = asyncio.create_task(self.status_send(websocket))
                        self.task_list.append(receive_task)
                        self.task_list.append(status_task)
                        await asyncio.gather(receive_task, status_task, return_exceptions=True)
            except Exception as e:
                print(f"Error in main_loop: {e}")
                await self.disconnect()
            
            # 재연결 전 대기
            await asyncio.sleep(1)

    async def received_handler(self, websocket) -> None:
        """서버로부터 메시지 수신 처리"""
        while True:
            try:
                message = await websocket.recv()
                print(f"received = {message}")
                message_dict = json.loads(message)
                uuid = message_dict.get("uuid")

                if uuid is not None:
                    callback_message = {
                        "topic":"received_callback",
                        "uuid":uuid
                    }
                    self.send_message(callback_message)
                    print(f"send = {callback_message}")

                pub.sendMessage("receive_message", message=message_dict)
                print(f"메시지 수신: {message_dict}")
            except json.JSONDecodeError:
                pass  # 잘못된 JSON 형식일 때 처리
            except Exception as e:
                print(f"Error in received_handler: {e}")
                await self.disconnect()
                break  # 연결 종료 시 loop 종료

    async def status_send(self, websocket) -> None:
        """로봇 상태 전송"""
        while True:
            try:
                # status 값이 None이 아닐 때만 전송
                if self.registered and self.sub_controller.status is not None:
                    message_dict = {
                        "topic": "robot_status",
                        "value": vars(self.sub_controller.status)
                    }
                    await websocket.send(json.dumps(message_dict))
                else:
                    print("No status to send")
                await asyncio.sleep(0.5)  # 상태 업데이트 간격
            except Exception as e:
                print(f"Error in status_send: {e}")
                await self.disconnect()
                break  # 연결 종료 시 loop 종료

    async def disconnect(self):
        for task in self.task_list:
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    print(f"Task {task.get_coro().__name__} cancelled")
        self.task_list = []
        self.registered = False

    def send_message(self,message):
        url = f"http://{self.server_host}:{self.server_port}/robot_message"
        for i in range (10):
            res = requests.post(url,json=message)
            if res.status_code == 200:
                return True
            else:
                time.sleep(2)
                continue
        return False
