from pymodbus.client import ModbusTcpClient
import time
# -----------------------------
# 환경 설정
# -----------------------------
ROBOT_IP = "192.168.10.118"   # 두산 로봇 제어기 IP
ROBOT_PORT = 502            # 기본 포트
client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT)

# -----------------------------
# 연결
# -----------------------------
if not client.connect():
    print("❌ 연결 실패. IP/Port 확인하세요.")
    exit()
else:
    print("✅ 연결 성공")

# -----------------------------
# 단일 레지스터 쓰기
# -----------------------------
def write_reg(addr, val):
    client.write_register(addr, val)
    print(f"[WRITE] addr={addr}, val={val}")

# -----------------------------
# 여러 레지스터 쓰기 (연속 주소)
# -----------------------------
def write_regs(start_addr, values):
    client.write_registers(start_addr, values)
    print(f"[WRITE_MULTI] start={start_addr}, values={values}")

# -----------------------------
# 읽기 함수
# -----------------------------
def read_regs(start_addr, count=1):
    result = client.read_holding_registers(start_addr, count=count)
    if result.isError():
        print("❌ 읽기 실패")
        return None
    vals = result.registers
    print(f"[READ] start={start_addr}, vals={vals}")
    return vals

# -----------------------------
# 예시: 여러 개 쓰기
# -----------------------------
# 예) 130~135번 레지스터에 조인트 값 한꺼번에 쓰기
# 음수 대응을 위해 16bit 변환
def to_modbus_value(val):
    return val * 10 & 0xFFFF



# 모드 전환 (movej 실행)
write_reg(129, 20)

target_pose = [0, 0, 0, 0, 0, 0]

write_regs(155, [to_modbus_value(v) for v in target_pose])


# 상태 읽기 (예: 129~137)
read_regs(200, 8)

write_reg(129, 0)

while True:
    read_regs(129)
    read_regs(200, 8)
    time.sleep(1)

# -----------------------------
# 연결 종료
# -----------------------------
client.close()
print("✅ 연결 종료")
