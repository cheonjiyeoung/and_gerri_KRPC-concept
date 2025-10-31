# ============================================================
# Zimmer Gripper Modbus Register Map (출력/입력 데이터 워드)
# ============================================================
#
# Output data word 0 - 0x0801 (ControlWord)
#   - UINT16, Write
#   - Execute a command (only one bit at a time)
#     0      : No action
#     1      : DataTransfer       (0x0001) – apply process data / workpiece data
#     2      : WritePDU           (0x0002) – write current dataset to tool recipe
#     4      : ResetDirectionFlag (0x0004) – reset direction flag
#     8      : Teach              (0x0008) – save current pos as TeachPosition
#     256    : MoveToBase         (0x0100) – move gripper to BasePosition
#     512    : MoveToWork         (0x0200) – move gripper to WorkPosition
#     1024   : JogToWork          (0x0400) – jog toward WorkPosition (slow move)
#     2048   : JogToBase          (0x0800) – jog toward BasePosition (slow move)
#
# Output data word 1 - 0x0802 (DeviceMode, Workpiece No)
#   - UINT8 DeviceMode + UINT8 WorkpieceNo
#   - DeviceMode: defines motion profile / operation mode (0~255)
#   - WorkpieceNo: dataset index (0~32)
#
# Output data word 2 - 0x0803 (Reserved, PositionTolerance)
#   - PositionTolerance [0.01 mm] e.g. 50 → 0.5 mm tolerance
#
# Output data word 3 - 0x0804 (GripForce, DriveVelocity)
#   - High byte : GripForce (1~100%)
#   - Low  byte : DriveVelocity (1~100%)
#
# Output data word 4 - 0x0805 (BasePosition)
#   - External gripping reference position [0.01 mm]
#
# Output data word 5 - 0x0806 (ShiftPosition)
#   - Intermediate shift position [0.01 mm]
#
# Output data word 6 - 0x0807 (TeachPosition)
#   - Position saved by Teach command [0.01 mm]
#
# Output data word 7 - 0x0808 (WorkPosition)
#   - Internal gripping target position [0.01 mm]
#
# ------------------------------------------------------------
#
# Input data word 0 - 0x0002 (StatusWord)
#   - UINT16, Read
#   - Bit-coded status flags:
#     0x4000 : MoveWorkpositionFlag
#     0x2000 : MoveBasepositionFlag
#     0x1000 : DataTransferOK
#     0x0400 : AtWorkposition
#     0x0100 : AtBaseposition
#     0x0040 : PLCActive
#     0x0008 : MovementComplete
#     0x0004 : InMotion
#     0x0002 : MotorOn
#     0x0001 : HomingPositionOK
#
# Input data word 1 - 0x0003 (Diagnosis)
#   - Diagnostic information (see device manual)
#
# Input data word 2 - 0x0004 (ActualPosition)
#   - Current jaw position [0.01 mm]
#   - 0 ~ max jaw stroke (model-dependent)
#   - Resolution: 0.01 mm / Accuracy: ±0.1 mm
#
# ============================================================

import threading
import time

from pymodbus.client import ModbusTcpClient

# ANSI SGR (Select Graphic Rendition) parameters
# 표준 터미널 색상 코드 정의

RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
NC = '\033[0m'  # No Color (색상 초기화)


# ------------------------------ constants (상수) ------------------------------
# yapf: disable
class ControlWord:
    """ControlWord (0x0801) bits (명령 비트)"""
    DataTransfer = 0x0001  # apply process data (프로세스 데이터 적용)
    WritePDU = 0x0002  # write dataset (데이터셋 저장)
    ResetDirectionFlag = 0x0004  # reset direction flag (방향 플래그 리셋)
    Teach = 0x0008  # save current pos as TeachPosition (교시)
    MoveToBase = 0x0100  # move to BasePosition (베이스로 이동)
    MoveToWork = 0x0200  # move to WorkPosition (워크로 이동)
    JogToWork = 0x0400  # jog toward WorkPosition (워크 방향 조그)
    JogToBase = 0x0800  # jog toward BasePosition (베이스 방향 조그)


class DeviceMode:
    """DeviceMode high byte values (모드 값)"""
    Mode0 = 0
    Mode1 = 1
    Mode2 = 2
    Mode3 = 3
    # NOTE: vendor specific modes can be added (제조사 모드 추가 가능)
    # e.g., Mode85 = 85


class StatusFlags:
    """StatusWord (0x0002) flags (상태 비트)"""
    MoveWorkpositionFlag = 0x4000
    MoveBasepositionFlag = 0x2000
    DataTransferOK = 0x1000
    AtWorkposition = 0x0400
    AtBaseposition = 0x0100
    PLCActive = 0x0040
    MovementComplete = 0x0008
    InMotion = 0x0004
    MotorOn = 0x0002
    HomingPositionOK = 0x0001


# yapf: enable
# ------------------------------------------------------------------------------


class ZimmerController:
    # 2-Finger Gripper : GEH6060IL, GEH6040IL
    # 3-Finger Gripper : GED6060IL, GED6040IL
    def __init__(self, ip, port):
        """
        Initialize the Zimmer class
        """

        # Register address definition
        self.NUM_RECV_REG = 3
        self.NUM_SEND_REG = 8

        # Status word definition
        # yapf: disable
        self.MoveWorkpositionFlag = StatusFlags.MoveWorkpositionFlag
        self.MoveBasepositionFlag = StatusFlags.MoveBasepositionFlag
        self.DataTransferOK = StatusFlags.DataTransferOK
        self.AtWorkposition = StatusFlags.AtWorkposition
        self.AtBaseposition = StatusFlags.AtBaseposition
        self.PLCActive = StatusFlags.PLCActive
        self.MovementComplete = StatusFlags.MovementComplete
        self.InMotion = StatusFlags.InMotion
        self.MotorOn = StatusFlags.MotorOn
        self.HomingPositionOK = StatusFlags.HomingPositionOK
        # yapf: enable

        # Zimmer connection variables
        self.ip = ip
        self.port = port
        self.connected = False

        # Modbus client
        self.mb = ModbusTcpClient(host=self.ip, port=self.port)

        # Zimmer Gripper ------------------------------------------------------------
        self.ADDR_RECV = 0x0001  # 2F: 0x0001, 3F: 0x0031
        self.ADDR_SEND = 0x0801  # 2F: 0x0801, 3F: 0x0831

        self.reg_read = 0
        self.reg_write = [0, 0, 0, 0, 0, 0, 0, 0]

        self.gripper_thread_run = False
        self.gripper_thread = threading.Thread(target=self.communication_func)

        self.gripper_force = 50
        self.gripper_velocity = 50
        self.gripper_grip_distance = 0

        self.gripper_send_flag = False
        self.gripper_comm_step = 0
        self.gripper_init_flag = False
        self.gripper_grip_flag = False

        self.gripper_max_distance = 3100  # WorkPosition 최대값 (0.01mm)
        self.gripper_min_distance = 100  # BasePosition 최소값 (0.01mm)

        # 그리퍼가 실제로 움직일 수 있는 거리 (카운트)
        self.MaxMoveableDistance = self.gripper_max_distance - self.gripper_min_distance

        self.last_position = self.gripper_max_distance

    # ---------------- Connection ----------------
    def connect(self):
        """
        Connect to the Zimmer Gripper

        Parameters:
        - ip (str): IP address of the Zimmer Gripper
        - port (int): Port number of the Zimmer Gripper
        """
        if self.connected is True:
            return

        self.mb = ModbusTcpClient(host=self.ip, port=self.port)
        self.connected = self.mb.connect()

        if self.connected is True:
            print(f'{GREEN}[SUCCESS]{NC} Connected gripper')
            self.gripper_thread = threading.Thread(target=self.communication_func)
            self.gripper_thread.daemon = True
            self.gripper_thread.start()


            self.init()
            self.opt_velocity(100)
            self.opt_force(10)

            time.sleep(3)
            self.move_to_percentage(0)
            self.move_to_percentage(100)
            self.move_to_percentage(0)
        else:
            print(f'{RED}[ERROR]{NC} Not connected gripper')
            exit(1)

    def disconnect(self):
        """
        Disconnect from the Zimmer Gripper
        """
        if self.connected is False:
            return

        self.gripper_thread_run = False
        self.mb.close()
        self.connected = False
        print(f'{GREEN}[SUCCESS]{NC} Disconnected gripper')

    # ---------------- Communication ----------------
    def communication_func(self):
        """
        Gripper function
        """
        self.gripper_thread_run = True

        while self.gripper_thread_run is True and self.connected is True:
            # read input registers (입력 레지스터 읽기)
            self.reg_read = self.mb.read_input_registers(self.ADDR_RECV, count=self.NUM_RECV_REG)
            self.gripper_grip_distance = self.reg_read.registers[2]

            if self.gripper_send_flag is True:
                # 0. PLC active bit check
                if self.gripper_comm_step == 0:
                    if bool(self.reg_read.registers[0] & self.PLCActive) is True:
                        print(f"\n{YELLOW}[ ZIMMER ]{NC} 0. PLC active bit check complete")
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 1. Data transfer ok bit & motor on bit check
                elif self.gripper_comm_step == 1:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is True \
                            and bool(self.reg_read.registers[0] & self.MotorOn) is True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 1. Data transfer ok bit & motor on bit check complete")
                        self.reg_write[0] = 0
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 2. Handshake
                elif self.gripper_comm_step == 2:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is not True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 2. Handshake is done")
                        # re-arm data transfer with a specific device mode (특정 모드로 재전송)
                        self.reg_write[0] = ControlWord.DataTransfer
                        self.reg_write[1] = (85 << 8) | 0  # NOTE: vendor-specific mode 85 (제조사 모드 85 가정)
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 3. Data transfer ok bit check
                elif self.gripper_comm_step == 3:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 3. Data transfer ok bit check complete")
                        self.reg_write[0] = 0
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1
                        self.gripper_init_flag = True

                # 4. Grip move to workposition/baseposition
                elif self.gripper_comm_step == 4:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is not True:
                        if self.gripper_grip_flag is True:
                            print(f"{YELLOW}[ ZIMMER ]{NC} 4. Grip move to workposition")
                            self.reg_write[0] = ControlWord.MoveToWork
                        else:
                            if bool(self.reg_read.registers[0] & self.AtBaseposition) is not True:
                                print(f"{YELLOW}[ ZIMMER ]{NC} 4. Grip move to baseposition")
                                self.reg_write[0] = ControlWord.MoveToBase

                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 5. Grip move complete check
                elif self.gripper_comm_step == 5:
                    if bool(self.reg_read.registers[0] & self.InMotion) is True \
                            and bool(self.reg_read.registers[0] & self.MovementComplete) is not True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 5. Grip move complete check")
                        self.gripper_comm_step += 1

                # 6. Move complete check
                elif self.gripper_comm_step == 6:
                    if bool(self.reg_read.registers[0] & self.InMotion) is not True \
                            and bool(self.reg_read.registers[0] & self.MovementComplete) is True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 6. Move complete")
                        self.reg_write[0] = ControlWord.ResetDirectionFlag
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 7. Move workposition flag & move baseposition flag check
                elif self.gripper_comm_step == 7:
                    if bool(self.reg_read.registers[0] & self.MoveWorkpositionFlag) is not True \
                            and bool(self.reg_read.registers[0] & self.MoveBasepositionFlag) is not True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 7. Move workposition flag & move baseposition flag check")
                        self.gripper_comm_step = -1  # reset communication step
                        self.gripper_send_flag = False

            time.sleep(0.01)

        self.reg_read = 0

    # ------------------- Commands -------------------
    def init(self):
        """
        Zimmer Gripper initialization
        """
        # build command
        self.reg_write[0] = ControlWord.DataTransfer  # ControlWord
        self.reg_write[1] = (DeviceMode.Mode3 << 8) | 0  # DeviceMode(3), WorkpieceNo(0)
        self.reg_write[2] = 50  # PositionTolerance = 0.50 mm
        self.reg_write[3] = (self.gripper_force << 8) | self.gripper_velocity  # GripForce/DriveVelocity
        self.reg_write[4] = 100  # BasePosition = 1.00 mm
        self.reg_write[5] = 2000  # ShiftPosition = 20.00 mm
        self.reg_write[7] = self.gripper_max_distance  # WorkPosition = max stroke

        self.gripper_init_flag = False
        self.gripper_comm_step = 0
        self.gripper_send_flag = True

        while self.gripper_init_flag is False:
            time.sleep(0.001)
        print(f'{BLUE}[ ZIMMER ]{NC} Initialized')

    def grip(self, grip_distance=-1, sync=True):
        """
        Zimmer Gripper gripping (closing)

        Parameters:
        - grip_distance (int): Grip distance (75 ~ max distance) [mm]
        - sync (bool): Synchronization flag
        """

        if grip_distance == -1:
            target_position = self.gripper_max_distance
        else:
            # convert gap[mm] to WorkPosition counts [0.01mm] (갭→워크포지션 변환)
            # actual_position = (74 - grip_distance) / 2 * 100 + 100
            target_position = grip_distance

        print(f'\n{BLUE}[ ZIMMER ]{NC} grip_distance : {grip_distance}')
        print(f'{BLUE}[ ZIMMER ]{NC} target_position : {target_position}')

        if self.gripper_init_flag is True:
            self.reg_write[0] = ControlWord.DataTransfer
            self.reg_write[1] = (DeviceMode.Mode3 << 8) | 0
            self.reg_write[2] = 5  # PositionTolerance
            self.reg_write[3] = (self.gripper_force << 8) | self.gripper_velocity
            self.reg_write[4] = 100  # BasePosition
            self.reg_write[5] = int(target_position - 20)  # ShiftPosition
            self.reg_write[7] = int(target_position)  # WorkPosition

            self.gripper_comm_step = 0  # reset communication step
            self.gripper_send_flag = True
            self.gripper_grip_flag = True

            if sync is True:
                while self.gripper_send_flag is True:
                    time.sleep(0.001)

    def release(self, release_distance=-1, sync=True):
        """
        Zimmer Gripper releasing (opening)

        Parameters:
        - release_distance (int): Release distance (75 ~ max distance) [mm]
        - sync (bool): Synchronization flag
        """
        if release_distance == -1:
            target_position = 100
        else:
            # convert gap[mm] to WorkPosition counts [0.01mm] (갭→워크포지션 변환)
            # actual_position = (74 - release_distance) / 2 * 100 + 150
            target_position = release_distance

        print(f'\n{BLUE}[ ZIMMER ]{NC} release_distance : {release_distance}')
        print(f'{BLUE}[ ZIMMER ]{NC} actual_position : {target_position}')

        if self.gripper_init_flag is True:
            self.reg_write[0] = ControlWord.DataTransfer
            self.reg_write[1] = (DeviceMode.Mode3 << 8) | 0
            self.reg_write[2] = 5  # PositionTolerance
            self.reg_write[3] = (self.gripper_force << 8) | self.gripper_velocity
            self.reg_write[4] = int(target_position)  # BasePosition
            self.reg_write[5] = int(target_position + 20)  # ShiftPosition
            self.reg_write[7] = self.gripper_max_distance  # WorkPosition

            self.gripper_comm_step = 0  # reset communication step
            self.gripper_send_flag = True
            self.gripper_grip_flag = False

            if sync is True:
                while self.gripper_send_flag is True:
                    time.sleep(0.001)

    def opt_velocity(self, velocity=50):
        """
        Zimmer Gripper velocity option (1~100%)
        """
        self.gripper_velocity = velocity

    def opt_force(self, force=50):
        """
        Zimmer Gripper force option (1~100%)
        """
        self.gripper_force = force

    def get_position(self):
        """
        Zimmer Gripper position
        """
        return self.gripper_grip_distance

    def get_status(self):
        """
        Zimmer Gripper status

        Returns:
        - bool: True if gripper is closed, False if gripper is open
        """

        if self.gripper_grip_flag:
            return True
        else:
            return False

    def convert_width_to_position(self, width_mm):
        """
        [mm] 단위의 그리퍼 간격을 WorkPosition/BasePosition 카운트 [0.01 mm]로 변환합니다.

        주의:
        - Zimmer 매뉴얼에 따라 (74mm - gap) / 2 * 100 + 100와 같은 수식 사용을 가정합니다.
        - 이는 모델(2F/3F, 스트로크)에 따라 다를 수 있습니다.
        """
        # width_mm: 조 팁 간의 거리 (mm)
        # 100 카운트는 BasePosition의 최소값 (1.00 mm)으로 추정
        # Zimmer 모델별 최대 스트로크(예: 40.75mm)와 jaw 오버행을 고려한 수식

        # 실제 그리퍼 조작 위치 (0.01mm 카운트)
        # BasePosition(100)이 완전 개방 위치(예: 74mm 간격)와 관련 있다고 가정하고 변환
        # max_distance (4075) = 40.75 mm 스트로크 + 75mm (가정)

        # 간격(mm)을 위치 카운트(0.01mm)로 변환하는 일반적인 역산:
        # P = K1 * (MaxGap - width_mm) + K2
        # 여기서는 기존 코드의 로직을 따라 변환합니다.

        # 기존 코드 (grip): actual_position = (74 - grip_distance) / 2 * 100 + 100
        actual_position = (74.0 - width_mm) / 2.0 * 100.0 + 100.0

        # 최대/최소 범위 제한 (모델 의존적)
        actual_position = max(100, min(actual_position, self.gripper_max_distance))

        return int(actual_position)

    def move_to_width(self, width_mm, sync=True):
        """
        Zimmer Gripper를 목표 간격(width_mm)으로 이동시킵니다. (그립/릴리즈 통합)

        Parameters:
        - width_mm (float): 목표 그리퍼 간격 [mm] (예: 10.0 ~ 74.0)
        - sync (bool): 동기화 플래그
        """
        target_position = self.convert_width_to_position(width_mm)

        # 현재 위치 (0.01mm 카운트)
        current_position = self.gripper_grip_distance

        # 간격(mm) 정보 출력
        print(f'\n{BLUE}[ ZIMMER ]{NC} move_to_width : {width_mm} mm')
        print(f'{BLUE}[ ZIMMER ]{NC} Target Position (0.01mm counts) : {target_position}')

        if self.gripper_init_flag is False:
            print(f'{RED}[ERROR]{NC} Gripper is not initialized.')
            return

        self.reg_write[0] = ControlWord.DataTransfer
        self.reg_write[1] = (DeviceMode.Mode3 << 8) | 0
        self.reg_write[2] = 50  # PositionTolerance
        self.reg_write[3] = (self.gripper_force << 8) | self.gripper_velocity

        # 목표 위치가 현재 위치보다 크면 닫는 동작 (그립, WorkPosition으로 이동)
        if target_position > current_position and target_position <= self.gripper_max_distance:
            self.grip(target_position)

        # 목표 위치가 현재 위치보다 작으면 여는 동작 (릴리즈, BasePosition으로 이동)
        elif target_position < current_position and target_position >= 100:
            self.release(target_position)

        # 목표 위치가 이미 현재 위치와 같거나, 유효 범위를 벗어나면 동작 안 함
        else:
            print(f'{YELLOW}[ ZIMMER ]{NC} Target width is the same as current position or out of range. No movement.')
            return

        self.gripper_comm_step = 0  # reset communication step
        self.gripper_send_flag = True

        if sync is True:
            while self.gripper_send_flag is True:
                time.sleep(0.001)

    def move_to_percentage(self, pos_perc, velocity_perc=None, force_perc=None, sync=True):
        """
        Zimmer Gripper를 목표 간격(width_mm)으로 이동시킵니다. (그립/릴리즈 통합)

        Parameters:
        - width_mm (float): 목표 그리퍼 간격 [mm] (예: 10.0 ~ 74.0)
        - sync (bool): 동기화 플래그
        """
        # 1. 위치 변환 및 현재 위치 가져오기 (map_value 사용)
        # pos_perc (0~100)을 BasePosition(100) ~ WorkPosition(4075) 카운트로 매핑
        # *최대 개방(0%) = BasePosition (100)
        # *최대 닫힘(100%) = WorkPosition (4075)
        target_position_float = self.map_value(
            pos_perc,
            0, 100,
            self.gripper_min_distance, self.gripper_max_distance
        )
        target_position = int(round(target_position_float))  # 정수로 변환
        current_position = self.gripper_grip_distance
        print(f'\n{BLUE}[ ZIMMER ]{NC} move_to_percentage : {pos_perc}% ({target_position} counts)')
        print(f'\n{BLUE}[ ZIMMER ]{NC} current_position : {current_position}')
        if self.gripper_init_flag is False:
            print(f'{RED}[ERROR]{NC} Gripper is not initialized.')
            return

        if target_position == self.last_position:
            return

        # 목표 위치가 현재 위치보다 크면 닫는 동작 (그립, WorkPosition으로 이동)
        elif target_position > current_position and target_position <= self.gripper_max_distance:
            self.grip(target_position, sync=sync)

        # 목표 위치가 현재 위치보다 작으면 여는 동작 (릴리즈, BasePosition으로 이동)
        elif target_position < current_position and target_position >= 100:
            self.release(target_position, sync=sync)
        # 목표 위치가 이미 현재 위치와 같거나, 유효 범위를 벗어나면 동작 안 함
        else:
            print(f'{YELLOW}[ ZIMMER ]{NC} Target width is the same as current position or out of range. No movement.')
            return
        
        self.last_position = target_position


    @staticmethod
    def clamp(value, min_value, max_value, absolute_limit=None):
        if absolute_limit:
            min_value = max(min_value, absolute_limit[0])
            max_value = min(max_value, absolute_limit[1])
        return max(min_value, min(value, max_value))

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """
        특정 값을 주어진 범위 내에서 다른 범위로 매핑하는 함수.

        :param value: 매핑할 값
        :param in_min: 매핑 전 최소값
        :param in_max: 매핑 전 최대값
        :param out_min: 매핑 후 최소값
        :param out_max: 매핑 후 최대값
        :return: 매핑된 값
        """
        map_value = self.clamp((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max)

        return map_value


if __name__ == "__main__":
    # ❗ 그리퍼 IP 주소를 실제 환경에 맞게 변경하세요.
    GRIPPER_IP = '192.168.10.120'
    GRIPPER_PORT = 502
    gripper = ZimmerController(ip=GRIPPER_IP, port=GRIPPER_PORT)

    print(f"\n{YELLOW}=== Zimmer Gripper 통합 기능 테스트 시작 ==={NC}")

    gripper.connect()

    if not gripper.connected:
        print(f"{RED}연결 실패. IP 주소 및 전원을 확인하세요.{NC}")
    else:
        try:
            # 1. 초기화
            gripper.init()
            gripper.opt_velocity(100)
            gripper.opt_force(10)
            # gripper.grip()

            for i in range(0,101,5):
                # gripper.move_to_width(i, sync=False)
                gripper.move_to_percentage(i, sync=False)
                time.sleep(0.5)

            # --- 실시간성 테스트: 0%에서 100%까지 100단계로 연속 이동 ---
            print(f"\n{BLUE}--- 연속 100단계 제어 시작 (V:100%, F:10%) ---{NC}")

            # 먼저 0% (최대 개방)으로 이동
            # gripper.move_to_percentage(0, sync=False)
            # time.sleep(3)

            # # 0% -> 100% (닫기) 연속 이동 (sync=False로 비동기 실행)
            # for perc in range(1, 101, 1): # 1%씩 증가
            #     gripper.move_to_percentage(perc, sync=False)
            #     time.sleep(0.1) # 50ms 간격으로 명령 전송

            # time.sleep(1)

            # 100% -> 0% (열기) 연속 이동 (sync=False)
            for perc in range(100, -1, -5):  # 1%씩 감소
                gripper.move_to_percentage(perc, sync=False)
                time.sleep(0.5)

            time.sleep(2)
            print(f"{GREEN}연속 제어 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")

            # -------------------------------------------------------------

            # 2. 완전 개방 (0%)
            print(f"\n{BLUE}--- 1. 완전 개방 (0%) ---{NC}")
            # gripper.move_to_percentage(0, velocity_perc=50, force_perc=50, sync=True)
            # print(f"{GREEN}Move 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")
            time.sleep(1)

            # # 3. 중간 닫힘 (50%)
            # print(f"\n{BLUE}--- 2. 중간 닫힘 (50%) ---{NC}")
            # gripper.move_to_percentage(50, velocity_perc=20, force_perc=80, sync=True)
            # print(f"{GREEN}Move 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")
            # time.sleep(1)

            # # 2. 완전 개방 (릴리즈 동작)
            # OPEN_WIDTH = 70.0
            # print(f"\n{BLUE}--- 1. 완전 개방 (Width: {OPEN_WIDTH}mm) ---{NC}")
            # gripper.move_to_width(OPEN_WIDTH, sync=True)
            # print(f"{GREEN}Move 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")
            # time.sleep(1)

            # # 3. 목표 간격으로 닫기 (그립 동작)
            # GRIP_WIDTH = 25.0
            # print(f"\n{BLUE}--- 2. 목표 간격으로 닫기 (Width: {GRIP_WIDTH}mm) ---{NC}")
            # gripper.move_to_width(GRIP_WIDTH, sync=True)
            # print(f"{GREEN}Move 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")
            # time.sleep(1)

            # # 4. 더 좁은 간격으로 이동 (그립 동작 연속)
            # NARROW_WIDTH = 15.0
            # print(f"\n{BLUE}--- 3. 더 좁은 간격으로 이동 (Width: {NARROW_WIDTH}mm) ---{NC}")
            # gripper.move_to_width(NARROW_WIDTH, sync=True)
            # print(f"{GREEN}Move 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")
            # time.sleep(1)

            # # 5. 다시 넓게 열기 (릴리즈 동작 연속)
            # WIDE_WIDTH = 50.0
            # print(f"\n{BLUE}--- 4. 다시 넓게 열기 (Width: {WIDE_WIDTH}mm) ---{NC}")
            # gripper.move_to_width(WIDE_WIDTH, sync=True)
            # print(f"{GREEN}Move 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")
            # time.sleep(1)

        except Exception as e:
            print(f"{RED}테스트 중 오류 발생: {e}{NC}")

        finally:
            print(f"\n{YELLOW}=== Zimmer Gripper 통합 기능 테스트 종료 ==={NC}")
            gripper.disconnect()

# if __name__ == "__main__":
#     # Zimmer Gripper 객체 생성
#     gripper = Zimmer()

#     # ❗ 그리퍼 IP 주소를 실제 환경에 맞게 변경하세요.
#     GRIPPER_IP = '192.168.3.113'
#     GRIPPER_PORT = 502

#     print(f"\n{YELLOW}=== Zimmer Gripper 직접 실행 테스트 시작 ==={NC}")

#     # 1. 연결 시도
#     gripper.connect(ip=GRIPPER_IP, port=GRIPPER_PORT)

#     if not gripper.connected:
#         print(f"{RED}연결 실패. IP 주소 및 전원을 확인하세요.{NC}")
#         # 연결 실패 시 종료
#     else:
#         try:
#             # 2. 초기화 (Initialization)
#             print(f"\n{BLUE}--- 1. Gripper 초기화 (Homing & Data Transfer) ---{NC}")
#             gripper.init()
#             time.sleep(1)

#             # 3. 옵션 설정: 속도 50%, 힘 70%
#             TEST_VELOCITY = 50
#             TEST_FORCE = 70
#             print(f"\n{BLUE}--- 2. 속도/힘 설정 (Velocity={TEST_VELOCITY}%, Force={TEST_FORCE}%) ---{NC}")
#             gripper.opt_velocity(TEST_VELOCITY)
#             gripper.opt_force(TEST_FORCE)

#             # 4. 그립 테스트 1: 간격 40mm로 이동
#             GRIP_DISTANCE_MM = 40
#             print(f"\n{BLUE}--- 3. Gripping (간격 {GRIP_DISTANCE_MM}mm) ---{NC}")
#             gripper.grip(grip_distance=GRIP_DISTANCE_MM, sync=True)
#             print(f"{GREEN}GRIP 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")
#             time.sleep(2)

#             # 5. 속도 변경 후 릴리즈 (느리게 열기)
#             TEST_VELOCITY_SLOW = 10
#             gripper.opt_velocity(TEST_VELOCITY_SLOW)
#             RELEASE_DISTANCE_MM = 65
#             print(f"\n{BLUE}--- 4. 속도 {TEST_VELOCITY_SLOW}%로 Release (간격 {RELEASE_DISTANCE_MM}mm) ---{NC}")
#             gripper.release(release_distance=RELEASE_DISTANCE_MM, sync=True)
#             print(f"{GREEN}RELEASE 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")
#             time.sleep(2)

#             # 6. 완전 개방 후, 힘 100%로 다시 닫기 (최대 힘 테스트)
#             gripper.opt_force(100)
#             gripper.opt_velocity(TEST_VELOCITY) # 속도 복원
#             print(f"\n{BLUE}--- 5. 완전 개방 후, 힘 100%로 닫기 (간격 20mm) ---{NC}")
#             gripper.release(release_distance=-1, sync=True) # 완전 개방
#             time.sleep(1)
#             gripper.grip(grip_distance=20, sync=True)
#             print(f"{GREEN}최대 힘 GRIP 완료.{NC} 현재 위치 (0.01mm) : {gripper.get_position()}")
#             time.sleep(2)

#         except Exception as e:
#             print(f"{RED}테스트 중 오류 발생: {e}{NC}")

#         finally:
#             # 7. 연결 해제
#             print(f"\n{YELLOW}=== Zimmer Gripper 직접 실행 테스트 종료 ==={NC}")
#             gripper.disconnect()