# hello_universe_config.py

### ROBOT
# 로봇의 ID, 모델, 분류 및 서버 접속 정보 등을 정의합니다.
ROBOT_INFO = {
    "id": "hello_universe",  # 로봇의 고유 ID
    "model": "gerri",  # 로봇 모델명
    "category": "sample",  # 로봇 카테고리
    "room_id": "test_room",  # 접속할 WebRTC 방 ID
    # "server_ip": "localhost",  # 접속할 서버 IP 주소
    # "server_ip": "172.20.1.250",  # 접속할 서버 IP 주소
    "server_ip": "125.131.105.165",  # 접속할 서버 IP 주소
    "server_port": 25000,  # 접속할 서버 포트 번호
}

### VIDEO
# 카메라 장치 설정: 카메라 장치 인덱스 및 해상도 등을 정의합니다.
# 키 값(예: "front_cam")은 카메라의 레이블로 사용됩니다.
VIDEO_INFO = {
    "front_cam": {
        "index": 2,  # 카메라 장치 인덱스 (일반적으로 0부터 시작)
        "width": 2000,  # 비디오 해상도 너비
        "height": 1000,  # 비디오 해상도 높이
        "debug": False  # CameraManager의 디버그 모드 활성화 여부
    },
    # 추가 카메라가 있다면 여기에 정의할 수 있습니다.
    # "rear_cam": {"index": 1, "width": 1280, "height": 720, "debug": False},
}

### AUDIO
# 오디오 입출력 장치 및 관련 설정을 정의합니다.
# 오디오 기능을 사용하려면 'audio' 키 아래 'input'과 'output' 장치 이름이 유효해야 합니다.
AUDIO_INFO = {
    "audio": {
        "input": "default",  # 입력 오디오 장치 이름 (예: "마이크(Realtek High Definition Audio)") 또는 "default"
        "output": "default",  # 출력 오디오 장치 이름 (예: "스피커(Realtek High Definition Audio)") 또는 "default"

        # 아래는 AudioPlayer 및 AudioRecorder에 전달될 수 있는 추가적인 선택적 파라미터입니다.
        "player_volume": 100,  # 수신 오디오 재생 볼륨 (0.0 ~ 1.0)
        "silence_threshold": 50,  # 오디오 플레이어의 묵음 감지 임계값
    }
    # 만약 오디오 기능을 사용하지 않으려면 아래와 같이 'audio' 키를 비워두거나,
    # 'input' 또는 'output' 값을 None 또는 빈 문자열로 설정할 수 있습니다.
    # "audio": {}
    # 또는
    # "audio": {"input": None, "output": None}
}
