import socket

# 서버 소켓과 동일한 포트로 연결
host = 'localhost'
port = 12345  # 서버에서 할당한 포트와 동일해야 함

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 서버에 연결
client_socket.connect((host, port))
print(f"Connected to server at {host}:{port}")

def receive_loop():
    while True:
        # 서버로부터 응답 받기
        try:
            response = client_socket.recv(1024)  # 최대 1024 바이트까지 수신
            if not response:
                # 서버가 연결을 끊으면 종료
                print("Server closed the connection")
                break
            print(f"Received from server: {response.decode()}")
        except KeyboardInterrupt:
            print("Closing connection.")
            break
        except Exception as e:
            print(f"Error: {e}")
            break
        print(1)
    client_socket.close()

# 메시지 수신 루프 시작
receive_loop()
