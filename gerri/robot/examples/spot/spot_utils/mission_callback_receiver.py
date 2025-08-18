import socket
from pubsub import pub
import json
class MissionCallbackReceiver:
    def __init__(self, server_address=('localhost', 12345)):
        self.server_address = server_address
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)

    def open_socket(self):
        self.socket.listen(1)
        print("Waiting for connection...")
        try:
            while True:
                # 클라이언트 연결 수립 대기
                conn, addr = self.socket.accept()
                print(f"Connection established: {addr}")
                
                try:
                    conn.sendall("connect success".encode())  # 클라이언트에게 데이터 전송
                    data = conn.recv(1024).decode()  # 클라이언트로부터 데이터 수신\
                    try:
                        print(f"data:{data} / data type : {type(data)}")
                        message = json.loads(data)
                        pub.sendMessage("receive_message",message=message)
                    except Exception as e:
                        print(f"err in prosess socket data : {e}")
                    print(f"Received: {data}")
                except Exception as e:
                    print(f"Error while receiving data: {e}")
                finally:
                    # 연결 종료
                    conn.close()
                    print(f"Connection with {addr} closed.")
        except Exception as e:
            print(f"Error while waiting for connections: {e}")
        finally:
            self.socket.close()

# 사용 예시
if __name__ == "__main__":
    receiver = MissionCallbackReceiver()
    receiver.open_socket()
