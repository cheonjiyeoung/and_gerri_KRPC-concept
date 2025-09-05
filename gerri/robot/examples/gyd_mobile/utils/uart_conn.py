import queue,serial
import threading

class UART_Connector:
    def __init__(self,port="/dev/ttyUSB0",baudrate=115200,timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.buf = []
        self.q_data = queue.Queue(-1)

        self.th_receiver = threading.Thread(target=self.uart_recieve,daemon=True)
        self.th_receiver.start()

    # 시리얼 받는놈
    # 수신 -> 버퍼저장 -> 패킷단위 분리 -> 파싱/처리
    def uart_recieve(self):
        signiture = [] # 패킷 시작 시그니쳐 \xaaT
        buf = [] # 버퍼
        try:
            while True:
                raw_data = self.ser.read() # 1바이트 수신

                # 시작 시그니쳐 첫번째 바이트 발견
                if raw_data == b'\xaa':
                    # 버퍼에 데이터가있는경우 : 버퍼에 있는 데이터 -> 데이터큐에 저장 후 초기화
                    if buf:
                        self.q_data.put(''.join(buf[1:-1]))
                        buf = []
                    signiture = []
                    signiture.append(raw_data)

                # 시작 시그니쳐 두번째 바이트 발견
                elif raw_data == b'T':
                    signiture.append(raw_data)
                    buf.append(raw_data.decode("utf-8"))

                # 시작 시그니쳐 두바이트 전부 받았을때 : 버퍼에 데이터 쌓기 시작
                elif signiture == [b'\xaa',b'T']:
                    try:
                        buf.append(raw_data.decode("utf-8"))
                    except Exception as e:
                        pass

        except KeyboardInterrupt:
            self.ser.close()
            print("Serial connection closed.")
        except:
            pass
        
    # 데이터 전송
    def uart_send(self,res: str):
        bytes_data = res.encode('ascii')  # 문자열을 ASCII 바이트로 변환
        packet_length = len(bytes_data)   # 데이터 길이

        byte1 = bytearray(4 + packet_length)  # 전체 패킷 크기 (헤더 + 길이 + 데이터 + 체크섬)
        byte1[0] = 0xAA  # Frame Header 첫 번째 바이트
        byte1[1] = 0x54  # Frame Header 두 번째 바이트
        byte1[2] = packet_length  # 데이터 길이 (L)

        # 데이터 바이트 복사
        for i, b in enumerate(bytes_data):
            byte1[i + 3] = b  # 데이터 삽입

        # 체크섬 계산 (길이와 모든 데이터 XOR)
        checksum = packet_length
        for b in bytes_data:
            checksum ^= b
        
        byte1[-1] = checksum  # 마지막 바이트에 체크섬 저장
        packet = bytes(byte1)
        self.ser.write(packet) 

if __name__ == "__main__":
    import requests
    import time
    ROBOT_ADDR = "192.168.1.102"
    uart = UART_Connector()
    # packet = "nav_point[2F_CHARGINGPILE]"
    # packet = "nav_point[2F_TELEOP]"
    # packet = "nav_pause"
    packet = "nav_resume"
    # packet = "nav_cancel"
    uart.uart_send(packet)
