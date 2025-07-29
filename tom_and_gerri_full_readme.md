# 🤖 (TOM) AND GERRI

모듈형 로봇 원격 제어 프레임워크 TOM AND GERRI는 다음 세 가지 핵심 구성요소로 이루어져 있습니다:

- TOM (Tele Operation Module): 로봇에 물리적으로 장착되는 장치로, GERRI와 연결되어 로봇 상태 및 사용자 입력 정보를 실시간으로 주고받습니다.

- GERRI (Global Extended Robot Remote Interface): 오퍼레이터와 로봇으로 구성된 분산형 로봇 제어 소프트웨어입니다.
  - Operator: 키보드, VR, 마스터 암 등 다양한 입력 인터페이스로부터 명령을 받아 AND를 통해 로봇에 전달합니다.
  - Robot: 수신된 명령을 해석하고 실제 하드웨어 동작으로 수행합니다.

- AND (Adaptive Network Daemon): 오퍼레이터와 로봇 간 메시지를 주고받는 범용 통신 모듈로, WebRTC 등 다양한 프로토콜을 지원하며 고정된 방식에 의존하지 않습니다.

이 구조는 인터페이스, 네트워크, 실행 계층 간의 역할을 명확히 분리하여, 유연하고 확장 가능한 원격 로봇 제어를 가능하게 합니다.

## 🗂️ 프로젝트 구조 (Project Structure)

```
and_gerri/
├── gerri/                  # GERRI 프레임워크 코어
│   ├── operator/           # Operator 측 모듈
│   │   ├── function/       # BaseCommander가 상속받는 기본 메시지 포맷
│   │   ├── interface/      # 키보드, VR 등 입력 장치 인터페이스
│   │   ├── examples/       # 로봇별 Operator 구현 예시
│   │   │   └── [someoperator]/ # 특정 로봇의 Operator 구현부 (예: piper_operator)
│   │   │       ├── base_commander.py   # 메시지 포맷팅 및 전송 (BaseCommander)
│   │   │       ├── sub_commander.py    # 입력 값 기반 로직 처리 (SubCommander)
│   │   │       └── robot_config.py     # 로봇 관련 설정 파일
│   │   └── ...
│   ├── robot/              # Robot 측 모듈
│   │   ├── function/       # BaseController가 상속받는 기본 메시지 포맷
│   │   ├── interface/      # 로봇 측 센서, E-Stop 등 I/O
│   │   ├── examples/       # 로봇별 Controller 구현 예시
│   │   │   └── [somerobot]/      # 특정 로봇의 Robot 구현부 (예: doosan_robot)
│   │   │       ├── base_controller.py  # 메시지 라우팅 (BaseController)
│   │   │       ├── sub_controller.py   # 값 변환, 함수 매핑 등 어댑터 (SubController)
│   │   │       ├── robot_controller.py # 로봇 SDK 또는 직접 제어 로직 (RobotController)
│   │   │       └── robot_config.py     # 로봇 관련 설정 파일
│   │   └── ...
│   └── ...
├── _and_/                  # Adaptive Network Daemon (AND) 코어
├── utils/                  # 유틸리티 스크립트
├── [robot_name]_*          # 로봇/오퍼레이터 실행 스크립트
├── requirements.txt        # Python 의존성 파일
├── install.sh              # 설치 스크립트
└── README.md               # README 파일
```

## 🛠️ TOM 초기 구성 (First-Time TOM Setup)

TOM 모듈을 처음 사용하는 경우 아래 장비와 설정이 필요합니다.

필요 장비
- reComputer J4012 (NVIDIA® Jetson™ Orin™ NX 16GB)
- 점퍼케이블 F/F
- Ubuntu 22.04가 설치된 PC

Jetson Flash 안내
- reComputer J4012의 출고 버전은 Jetson 5.1이므로 Jetson 6로 플래시 과정이 필요합니다.
- 자세한 설명은 Seeed Studio Wiki에서 확인하세요: https://wiki.seeedstudio.com/reComputer_J4012_Flash_Jetpack/

## 📦 설치 (Installation)

1. 저장소 클론

```bash
mkdir -p ~/dev
cd ~/dev
git clone https://github.com/keti-ai/and_gerri.git
cd and_gerri
```

Note: Private 저장소 접근 시 아래와 같이 Username과 Token을 입력해야 합니다.
- username: 2kng
- password(Token): github_pat_...

2. 환경 설정 (Ubuntu)

```bash
./install.sh
```

설치 내용:
- Python 3.10 및 필수 시스템 패키지
- Python 가상환경(venv) 생성 및 의존성 패키지 설치

가상환경 활성화:

```bash
source venv/bin/activate
```

## 🧠 시스템 구성 및 명령 흐름

```
   오퍼레이터 측                      로봇 측
┌──────────────┐             ┌─────────────────┐
│  Interface   │             │ BaseController  │
└──────┬───────┘             └────────┬──────────┘
       │ raw 입력 (e.g. 'W' key)      │ topic 메시지
       ↓                              ↓
┌──────────────┐             ┌─────────────────┐
│ SubCommander │             │ SubController   │
└──────┬───────┘             └────────┬──────────┘
       │ 로직 처리 (e.g. 전진 명령)   │ 로봇 맞춤 변환
       ↓                              ↓
┌──────────────┐             ┌─────────────────┐
│ BaseCommander│──── AND ────>│ RobotController │
└──────────────┘ (네트워크)   └─────────────────┘
```

## 🚀 예제 실행법 (Hello Universe)

항상 로봇 측 프로그램을 먼저 실행해야 오퍼레이터가 접속할 수 있습니다.

### 1. 로봇 등록 및 API 키 발급

1. 러버넥 페이지(https://rubberneck.kr)로 이동하여 로그인 후 My Pages > Robots 메뉴로 이동합니다.
2. + New Robot 버튼을 눌러 로봇을 등록합니다.
3. 등록된 로봇의 수정(펜 모양) 버튼을 클릭하여 API Key를 확인하고 복사합니다.

### 2. 로봇 설정 및 실행 (on TOM / Jetson)

hello_universe_robot_config.py 파일에서 다음 항목 수정:

```python
ROBOT_INFO = {
 "id": "등록한 로봇 ID",
 "model": "gerri",
 "category": "sample",
 "api_key": "발급받은 API Key"
}
```

실행:

```bash
source venv/bin/activate
python hello_universe_robot.py
```

### 3. 오퍼레이터 설정 및 실행 (on PC)

hello_universe_operator_config.py 파일에서 다음 항목 수정:

```python
OPERATOR_INFO = {
 "id": "본인 아이디",
 "password": "비밀번호"
}
```

실행:

```bash
source venv/bin/activate
python hello_universe_operator.py
```

## 🧩 커스텀 로봇 통합 방법

### Controller 통합 (로봇 측)

방법 1: 모델 이름으로 분기 처리

```python
# in BaseController
if robot_model == 'my_robot':
    from gerri.robot.examples.my_robot.my_sub_controller import MySubController
    return MySubController()
```

방법 2: Controller 직접 주입

```python
from my_robot.my_sub_controller import MySubController

sub_controller = MySubController()
robot = MyBaseController(ROBOT_INFO, controller=sub_controller)
robot.connect()
```

### Commander 통합 (오퍼레이터 측)

```python
from my_operator.my_sub_commander import MySubCommander

sub = MySubCommander()
base = MyBaseCommander(ROBOT_INFO, commander=sub)
base.connect()
```
