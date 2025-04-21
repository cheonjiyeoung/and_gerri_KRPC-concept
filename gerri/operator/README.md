## **AVATAR OPERATOR**

### **개요**  
AVATAR의 Operator 소프트웨어는 관제사의 PC에서 실행되며 다음 작업을 지원합니다:  
- 로봇 카메라 스트리밍 
- 로봇 원격 조작  

웹 기반 서비스인 **Rubberneck**과 비교했을 때 주요 차별점은 다음과 같습니다:  
- 다양한 컨트롤러 지원  
- 많은 단축키 제공 및 유연한 조작 가능  
- 로봇에 집중할 수 있는 사용자 환경 구축  
- 특정 로봇에 최적화된 데이터 시각화  

---

### **Demo 실행 방법**

#### **환경 설정**  
가상환경에 의존성 라이브러리가 설정되어 있지 않은 경우:  
```bash
pip install -r requirements.txt
```

#### **실행**  
Operator 소프트웨어 실행:  
```bash
python -m gerri_operator.py
```

---

### **기능 커스터마이징**

#### **키보드/마우스 제어**  
`keyboard_mouse_controller.py`에서 입력값을 처리하여 **pubsub**을 통해 `[로봇명]_operator.py`로 전달합니다.  
- 조작 커맨드 추가: `operator_tools/[로봇명]_operaotr.py` 파일의 key_mouse_control 함수에 키 추가

#### **Polling Rate 조정**  
기본 `100ms` 단위로 로봇으로 데이터가 전송됩니다.  
- 더 빠르거나 느린 조작이 필요하다면 `webrtc_operator_bridge.py`의 `POLLING_RATE_MS` 값을 수정하여 조정할 수 있습니다.  

---