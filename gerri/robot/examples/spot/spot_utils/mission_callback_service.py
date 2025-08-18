import logging
import asyncio
import signal
from bosdyn.api.mission import remote_pb2, remote_service_pb2_grpc
from bosdyn.client.server_util import GrpcServiceRunner, ResponseContext
from bosdyn.client.directory_registration import DirectoryRegistrationClient, DirectoryRegistrationKeepAlive
import bosdyn.client
import bosdyn.client.util
import time
import json
import socket
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(sys.executable), "../..")))
from gerri.robot.examples.spot.spot_config import SPOT_INFO

DIRECTORY_NAME = "custom_callback_service"
SERVICE_TYPE = "bosdyn.api.mission.RemoteMissionService" 
AUTHORITY = "remote-mission"

logging.basicConfig(level=logging.DEBUG)
_LOGGER = logging.getLogger(__name__)

class SpotCallbackServicer(remote_service_pb2_grpc.RemoteMissionServiceServicer):
    def __init__(self, robot_controller, ip="192.168.50.250", port=50051, robot=None):
        print("callback servicer run")
        self.controller = robot_controller
        try:
            self.robot = robot_controller.robot
        except:
            self.robot = robot
        self.ip = ip
        self.port = port

        self.callback_socket_addr = ('localhost',12345)

    def send_message(self,message):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            try:
                # 서버에 연결
                sock.connect(self.callback_socket_addr)
                print(f"Connected to {self.callback_socket_addr}")
                
                # 메시지 전송
                sock.sendall(message.encode())
                print(f"Message sent: {message}")
                
                # 서버로부터 응답 받기 (옵션)
                data = sock.recv(1024).decode()
                print(f"Received from server: {data}")
                
            except Exception as e:
                print(f"Error: {e}")


    def GetRemoteMissionServiceInfo(self, request, context):
        response = remote_pb2.GetRemoteMissionServiceInfoResponse()
        with ResponseContext(response, request):
            pass
        return response

    # callback method
    def Tick(self, request, context):
        response = remote_pb2.TickResponse()
        with ResponseContext(response, request):
            
            topic = request.inputs[0].key
            value = request.inputs[0].value.constant.string_value
            
            message_dict = {"topic":topic,"value":value,"target":"all"}
            message_json = json.dumps(message_dict)
            
            # 데이터 전송 시 예외 처리 추가
            try:
                self.send_message(message=message_json)
            except Exception as e:
                print(f"err in send_message : {e}")
               

            response.status = remote_pb2.TickResponse.STATUS_SUCCESS
        return response

    def EstablishSession(self, request, context):
        response = remote_pb2.EstablishSessionResponse()
        with ResponseContext(response, request):
            response.session_id = "session_1"
            response.status = remote_pb2.EstablishSessionResponse.STATUS_OK
        return response

    def Stop(self, request, context):
        response = remote_pb2.StopResponse()
        with ResponseContext(response, request):
            response.status = remote_pb2.StopResponse.STATUS_OK
        return response

    def TeardownSession(self, request, context):
        response = remote_pb2.TeardownSessionResponse()
        with ResponseContext(response, request):
            response.status = remote_pb2.TeardownSessionResponse.STATUS_OK
        return response

    async def run_service_async(self):
        try:
            add_fn = remote_service_pb2_grpc.add_RemoteMissionServiceServicer_to_server
            service_runner = GrpcServiceRunner(self, add_fn, self.port, logger=_LOGGER)

            # Directory에 서비스 등록
            dir_client = self.robot.ensure_client(DirectoryRegistrationClient.default_service_name)
            keep_alive = DirectoryRegistrationKeepAlive(dir_client, logger=_LOGGER)
            keep_alive.start(DIRECTORY_NAME, SERVICE_TYPE, AUTHORITY, self.ip, service_runner.port)

            # 백그라운드에서 서비스 실행
            with keep_alive:
                await service_runner.run_until_interrupt()
        except Exception as e:
            print(f"Error: {e}")

    def stop(self):
        print("Stopping the service...")

async def main():
    # 메인 스레드에서 시그널 처리
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 서비스 실행
    sdk = bosdyn.client.create_standard_sdk("HelloCallbackService")
    robot = sdk.create_robot("192.168.50.3")
    robot.authenticate(SPOT_INFO["spot_id"],SPOT_INFO["spot_pwd"])
    bosdyn.client.util.authenticate(robot)

    # Callback 서비스 실행
    callback_handler = SpotCallbackServicer(robot=robot, robot_controller=None)
    await callback_handler.run_service_async()

def signal_handler(sig, frame):
    print(f"Received signal: {sig}")
    # 종료 처리
    exit(0)

if __name__ == "__main__":
    asyncio.run(main())
