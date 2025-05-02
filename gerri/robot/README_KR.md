# 🤖 Hello Universe 로봇 프로젝트

GERRI 플랫폼과 AdaptiveNetworkDaemon을 기반으로 한 모듈형 로봇 제어 프레임워크입니다.  
본 저장소는 매니퓰레이터 및 모바일 베이스를 포함한 로봇과 오퍼레이터 구성의 샘플 셋업을 제공합니다.

---

## 🗂 프로젝트 구조 개요 (KR)

```
and_gerri/
├── gerri/                       # GERRI(Global Extended Robot Remote Interface)
│   ├── operator/                # 오퍼레이터(사용자 제어 측) 모듈
│   │   ├── commander/           # 명령 처리 로직 (베이스 + 로봇별)
│   │   ├── interface/           # 입력 장치 (키보드, VR, 마스터암 등)
│   │   ├── examples/            # 로봇별 예시 구현체 (예: Piper)
│   │   └── ...
│   ├── robot/                   # 로봇 측 모듈
│   │   ├── controller/          # 로봇 동작 실행 로직 (베이스 + 로봇별)
│   │   ├── interface/           # 센서, 비상정지 버튼 등 하드웨어 IO
│   │   ├── examples/            # 로봇별 예시 구현체 (예: Piper, Gyd)
│   │   └── ...
├── _and_/                       # Adaptive Network Daemon (통신)
├── utils/                       # 유틸리티 스크립트
├── hello_universe_*             # 로봇/오퍼레이터 실행 진입점
├── requirements.txt             # Python 의존성 목록
├── install.sh                   # 환경 설치 스크립트
└── README.md                    # 이 문서
```

---

## 🚀 빠른 시작

### 1. 저장소 클론

```bash
git clone https://github.com/your-org/and_gerri.git
cd and_gerri
```

### 2. 환경 구축

Ubuntu 기준 설치 스크립트 실행:

```bash
bash install.sh
```

이 작업은 다음을 수행합니다:

- Python 3.10 및 시스템 패키지 설치
- 가상환경 생성 및 활성화
- Python 패키지 설치

가상환경 활성화:

```bash
source venv/bin/activate
```

---

## 🤖 로봇 실행

```bash
python hello_universe_robot.py
```

이 스크립트는:

- AdaptiveNetworkDaemon 실행
- 로봇 컨트롤러 연결
- 로봇 루프 실행

설정 파일은 `hello_universe_config.py`를 수정하세요.

---

## 🧠 시스템 구성

시스템은 크게 두 파트로 나뉩니다:

### 1. 오퍼레이터 측 (`gerri/operator/`)

사용자의 입력과 명령 구성 담당

#### ▸ `interface/`
- 키보드/마우스
- VR 트래커
- 마스터 암, 조이스틱 등

#### ▸ `commander/`
- **베이스 커맨더**: 메시지 구성 및 전송
- **서브 커맨더**: 로봇별 인터페이스 처리

예:
- `ManipulatorCommander` → 매니퓰레이터용 베이스 커맨더
- `PiperCommander` → Piper 로봇용 서브 커맨더

---

### 2. 로봇 측 (`gerri/robot/`)

로봇 동작 처리 담당

#### ▸ `interface/`
- 비상정지 버튼
- 초음파, 라이다, 인코더 등

#### ▸ `controller/`
- **베이스 컨트롤러**: 명령 파싱 및 라우팅
- **서브 컨트롤러**: 실제 하드웨어 제어 로직

예:
- `ManipulatorController` → 매니퓰레이터용 베이스 컨트롤러
- `PiperController` → Piper 하드웨어 제어

---

### 🔄 전체 흐름 요약

```text
[ 인터페이스 입력 ]
    ↓ (예: 키보드, 마스터암)
[ 서브 커맨더 ]
    ↓
[ 베이스 커맨더 ]
    ↓  명령 포맷 구성
────────────────────────────→ (네트워크 통해 전송)
                             ↓
                      [ 베이스 컨트롤러 ]
                             ↓
                      [ 서브 컨트롤러 ]
                             ↓
                  실제 로봇 동작 실행
```

---

## 🧩 새 로봇 통합 방법

### 방법 1: 베이스 컨트롤러 내부에 등록

`_initialize_robot()` 함수에 모델 추가:

```python
if robot_model == 'my_robot':
    from gerri.robot.examples.my_robot.my_robot_controller import MyRobotController
    return MyRobotController(port="your-port", ...)
```

### 방법 2: 직접 주입 방식

로봇 실행 스크립트에서 명시적으로 주입:

```python
from gerri.robot.examples.sample_robot.sample_base_controller import SampleBaseController
from gerri.robot.examples.sample_robot.sample_sub_controller import SampleSubController

robot = SampleBaseController(ROBOT_INFO, controller=SampleSubController)
robot.connect()
```

---

## 📚 의존성 설치

설치 스크립트에서 `requirements.txt` 기반으로 자동 설치됩니다.  
새 패키지를 추가하면 다음 명령으로 갱신하세요:

```bash
pip freeze > requirements.txt
```

---

## 📝 라이선스

MIT 라이선스 또는 조직 내부 정책에 따릅니다.

---

## 🙋 도움이 필요하신가요?

문의: `your.name@yourdomain.com`  
또는 GitHub 이슈로 남겨주세요.