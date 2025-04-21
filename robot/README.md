## **AVATAR ROBOT**

### **개요**  
AVATAR의 Robot 소프트웨어는 로봇에 설치되어 관제사가 다음 작업을 수행할 수 있도록 지원합니다:  
- 로봇의 카메라를 통해 원격으로 시각 정보를 제공  
- 로봇의 마이크와 스피커를 통해 양방향 오디오 통신 지원  
- 관제사의 입력을 통해 로봇의 원격 조작 가능  

WebRTC 기반의 데이터 전송 방식을 사용하며, 모든 기능은 모듈화되어 필요에 따라 선택적으로 사용할 수 있습니다.

---

### **Demo 실행 방법**

#### **환경 설정**  
가상환경이 설정되어 있지 않은 경우:  
```bash
pip install -r requirements.txt
```

#### **기능별 실행**  
- **카메라 스트리밍**  
  ```bash
  python -m webrtc_camera_stream.py
  ```
- **오디오 스트리밍 (스피커/마이크)**  
  ```bash
  python -m webrtc.audio_stream.py
  ```
- **로봇 조작**  
  ```bash
  python -m webrtc_command_channel.py
  ```

---

### **로봇 연동 및 커스터마이징**

#### **WebRTC Bridge 수정**  
`webrtc_robot_bridge.py` 파일에서 아래 함수를 수정하여 로봇과의 연동을 커스터마이징할 수 있습니다:  
- `remote_message_received`: Operator에서 명령을 수신  
- `send_message`: Operator로 메시지 전송  

#### **로봇 컨트롤러 수정**  
`robot_controller.py` 파일을 수정하여 로봇의 동작과 명령 전달 방식을 맞춤 설정합니다:  
- `remote_message_received`에서 `robot_controller`를 호출하거나 **pubsub 방식**을 활용하여 로봇에 명령 전달  
- 로봇의 상태를 Operator로 전송 가능  

---