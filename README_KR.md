# 🤖 (TOM) AND GERRI

모듈형 로봇 원격 제어 프레임워크 **TOM AND GERRI**는 다음 세 가지 핵심 구성요소로 이루어져 있습니다:

- **TOM (Tele Operation Module)**: 로봇에 물리적으로 장착되는 장치로, GERRI와 연결되어 로봇 상태 및 사용자 입력 정보를 실시간으로 주고받습니다.

- **GERRI (Global Extended Robot Remote Interface)**: 오퍼레이터와 로봇으로 구성된 분산형 로봇 제어 소프트웨어입니다.
  - **Operator**: 키보드, VR, 마스터 암 등 다양한 입력 인터페이스로부터 명령을 받아 AND를 통해 로봇에 전달합니다.
  - **Robot**: 수신된 명령을 해석하고 실제 하드웨어 동작으로 수행합니다.

- **AND (Adaptive Network Daemon)**: 오퍼레이터와 로봇 간 메시지를 주고받는 범용 통신 모듈로, WebRTC 등 다양한 프로토콜을 지원하며 고정된 방식에 의존하지 않습니다.

이 구조는 인터페이스, 네트워크, 실행 계층 간의 역할을 명확히 분리하여, 유연하고 확장 가능한 원격 로봇 제어를 가능하게 합니다.

---

## 🧭 System Overview

Below is a high-level architecture of the Hello Universe robot framework:

![System Architecture](./and_gerri_diagram.svg)

## 🗂 Project Structure Overview

```
and_gerri/
├── gerri/                       # Core GERRI Framework
│   ├── operator/                # Operator-side modules
│   │   ├── commander/           # Command generation logic (base/sub)
│   │   ├── interface/           # Input devices (keyboard, VR, etc.)
│   │   ├── examples/            # Per-robot operator implementations
│   │   └── ...                  
│   ├── robot/                   # Robot-side modules
│   │   ├── controller/          # Base/sub controller logic
│   │   ├── interface/           # Robot-side I/O (sensors, estop)
│   │   ├── examples/            # Per-robot controller implementations
│   │   └── ...
├── _and_/                       # Adaptive Network Daemon core
├── utils/                       # Utility scripts
├── hello_universe_*             # Entry scripts for robot/operator
├── requirements.txt             # Python dependencies
├── install.sh                   # Setup script
└── README.md                    # This file
```

## 📦 Quick Start

### 1. 저장소 클론

```bash
git clone https://github.com/keti-ai/and_gerri.git
cd and_gerri
```

### 2. 환경 설정 (Ubuntu)

```bash
sudo chmod 777 install.sh
bash install.sh
```

설치 내용:
- Python 3.10 및 필수 시스템 패키지 설치
- 가상환경 생성 및 패키지 설치

📌 새로운 터미널을 열었을 때 가상환경을 다시 활성화하려면:

```bash
source venv/bin/activate
```

---

## 🧠 시스템 구성

### 오퍼레이터 측 (`gerri/operator/`)

| 디렉터리 | 설명 |
|----------|------|
| `commander/` | **BaseCommander** (예: `SampleBaseCommander`)와 **SubCommander** (예: `SampleSubCommander`)로 구성. Sub에서 입력 해석 및 명령 생성, Base가 메시지 포맷 전송. |
| `interface/` | 키보드, VR, 마스터 암 등 다양한 입력 장치 제어 |
| `examples/` | 로봇별 오퍼레이터 구현체 |

---

### 로봇 측 (`gerri/robot/`)

| 디렉터리 | 설명 |
|----------|------|
| `controller/` | **BaseController**는 메시지 라우팅, **SubController**는 실제 하드웨어 제어 로직 |
| `interface/` | 센서, 비상정지 스위치 등 물리적 장치 관리 |
| `examples/` | 로봇별 제어 로직 구현체 |

---

## 🔄 명령 흐름

```
[입력 장치 예: 키보드]
     ↓
[SubCommander]
     ↓
[BaseCommander]
     ↓
────────────────────→ 네트워크 전송
                     ↓
              [BaseController]
                     ↓
              [SubController]
                     ↓
              [로봇 동작 수행]
```

---

## 🧩 로봇 통합 방법

### 방법 1: base controller에 모델 등록

```python
if robot_model == 'gerri':
  from gerri.robot.examples.sample_robot.sample_sub_controller import SampleSubController

  return SampleSubController()
```

### 방법 2: controller 직접 주입

```python
robot = SampleBaseController(ROBOT_INFO, controller=SampleSubController())
robot.connect()
```

---

## 🧩 커맨더 통합 방법

지연 연결(후 주입) 패턴 지원:

```python
sub = SampleSubCommander()
base = SampleBaseCommander(ROBOT_INFO, commander=sub)
base.connect()
```

`set_base_commander()` 호출을 통해 순환 참조 없이 안전하게 연결합니다.

---