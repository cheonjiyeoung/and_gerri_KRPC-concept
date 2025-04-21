import zmq
import threading
import time
from pubsub import pub  # PyPubSub 라이브러리

class ZeroMQBridge:
    def __init__(self, target_ip='*', pub_port=8100, sub_port=8200):
        """
        ZeroMQ 기반 PUB-SUB 통신 매니저
        :param target_ip: 상대 노드의 IP (ORIN NX 또는 Raspberry Pi 5)
        :param pub_port: PUB 소켓 포트
        :param sub_port: SUB 소켓 포트
        """

        self.context = zmq.Context()

        # PUB 소켓 (메시지 송신)
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{pub_port}")

        # SUB 소켓 (메시지 수신)
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{target_ip}:{sub_port}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 모든 메시지 구독

        pub.subscribe(self.send_message, 'send_message_bridge')


    def connect(self):
        """ SUB 메시지를 별도 스레드에서 처리 """
        thread = threading.Thread(target=self.receive_messages, daemon=True)
        thread.start()

    def receive_messages(self):
        """ SUB 소켓을 통해 메시지를 지속적으로 수신 """
        while True:
            message = self.sub_socket.recv_json()
            print(f"📥ZeroMQ 수신된 메시지:", message)
            pub.sendMessage("receive_message_bridge", message=message)  # 내부 이벤트로 데이터 전달

    def send_message(self, message):
        """ PUB 소켓을 통해 메시지 전송 """
        print(f"📤ZeroMQ 메시지 전송:", message)
        self.pub_socket.send_json(message)
