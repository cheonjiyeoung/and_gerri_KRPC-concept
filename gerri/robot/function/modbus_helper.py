class ModbusHelper:
    """
    Modbus 인코딩/디코딩 헬퍼 클래스
    리스트와 단일 값을 모두 처리할 수 있습니다.
    """

    def __init__(self):
        pass

    # --- 내부 헬퍼 함수 (단일 값 처리 로직) ---

    def _encode_single(self, value, scale):
        """단일 값을 Modbus 16비트 unsigned int로 인코딩합니다."""
        scaled_value = int(round(value * scale))
        # 16비트 unsigned 값으로 마스킹 (음수는 2의 보수)
        return scaled_value & 0xFFFF

    def _decode_single(self, value, scale):
        """Modbus 16비트 값을 2의 보수로 해석하여 float으로 디코딩합니다."""
        # 16비트 2의 보수(signed int)로 변환
        signed_value = value if value < 0x8000 else value - 0x10000
        return float(signed_value) / scale

    # --- 공개 함수 (타입 체크 및 분기) ---

    def modbus_encode(self, value, scale=10):
        """
        입력값이 리스트이면 리스트를, 단일 값이면 단일 값을 인코딩하여 반환합니다.
        """
        if isinstance(value, (list, tuple)):
            # 입력이 리스트/튜플이면, 리스트로 반환
            return [self._encode_single(v, scale) for v in value]
        else:
            # 입력이 단일 값이면, 단일 값으로 반환
            return self._encode_single(value, scale)

    def modbus_decode(self, value, scale=10):
        """
        입력값이 리스트이면 리스트를, 단일 값이면 단일 값을 디코딩하여 반환합니다.
        """
        if isinstance(value, (list, tuple)):
            # 입력이 리스트/튜플이면, 리스트로 반환
            return [self._decode_single(v, scale) for v in value]
        else:
            # 입력이 단일 값이면, 단일 값으로 반환
            return self._decode_single(value, scale)


# --- 사용 예시 ---
if __name__ == "__main__":
    converter = ModbusHelper()

    print("--- 인코딩 테스트 ---")
    # 1. 단일 값 인코딩
    single_float = 12.3
    encoded_val = converter.modbus_encode(single_float)
    print(f"단일 값: {single_float}  ->  {encoded_val}")

    # 2. 리스트 인코딩
    list_float = [10.1, -5.2, 0.0, 3276.8] # 3276.8은 오버플로우 테스트
    encoded_list = converter.modbus_encode(list_float)
    print(f"리스트: {list_float}  ->  {encoded_list}")


    print("\n--- 디코딩 테스트 ---")
    # 3. 단일 값 디코딩
    single_int = 65484  # (-5.2 * 10) & 0xFFFF
    decoded_val = converter.modbus_decode(single_int)
    print(f"단일 값: {single_int}  ->  {decoded_val}")

    # 4. 리스트 디코딩
    list_int = [101, 65484, 0, 32768] # 32768 = -3276.8
    decoded_list = converter.modbus_decode(list_int)
    print(f"리스트: {list_int}  ->  {decoded_list}")