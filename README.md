# 🤖 (TOM) AND GERRI

A modular robot teleoperation framework composed of three key components:

- **TOM (Tele Operation Module)**: A physical interface device directly connected to the robot. It acts as the bridge between the robot and GERRI, enabling real-time exchange of robot status and user commands.

- **GERRI (Global Extended Robot Remote Interface)**: A distributed software system split into two parts:
  - **Operator**: Receives user input from various interfaces (e.g. keyboard, VR, master arm), processes commands, and sends them to the robot.
  - **Robot**: Interprets the received commands and executes appropriate hardware-level actions.

- **AND (Adaptive Network Daemon)**: A protocol-agnostic communication module responsible for transporting messages between the operator and the robot. While WebRTC is commonly used, the underlying protocol is not fixed.

Together, TOM, AND, and GERRI provide a scalable and flexible architecture for remote robot control with clear separation of concerns between interface, network, and execution layers.

---


## 🧭 System Overview

Below is a high-level architecture of the Hello Universe robot framework:

![System Architecture](./tom_and_gerri_2025_diagram.svg)

## 🗂 Project Structure Overview
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


## 📦 Quick Start

### 1. Clone the repository

```bash
mkdir ~/dev
cd ~/dev
git clone https://github.com/keti-ai/and_gerri.git # NEED ID and TOKEN
cd and_gerri
```

### 2. Set up the environment (Ubuntu)

```bash
sudo chmod 777 install.sh
bash install.sh
```

This will:
- Install Python 3.10 and system dependencies
- Create a Python virtual environment
- Install dependencies from `requirements.txt`

To activate the environment in a new terminal session:

```bash
source venv/bin/activate
```

---

## 🧠 Architecture Overview

### Operator Side (`gerri/operator/`)

| Directory | Role |
|----------|------|
| `commander/` | Divided into **base** (e.g. `SampleBaseCommander`) and **sub** (e.g. `SampleSubCommander`). SubCommanders define input mappings and send logic to BaseCommanders. |
| `interface/` | Manages input devices like keyboard, VR tracker, master arm. |
| `examples/` | Robot-specific commander logic. |

---

### Robot Side (`gerri/robot/`)

| Directory | Role |
|----------|------|
| `controller/` | Divided into **base** (e.g. `SampleBaseController`) and **sub** (e.g. `SampleSubController`). SubControllers define hardware logic; BaseControllers manage message routing. |
| `interface/` | Deals with sensors, emergency buttons, etc. |
| `examples/` | Robot-specific control logic. |

---

## 🔄 Command Flow

```
[Interface Input (e.g. keyboard)]
     ↓
[SubCommander]
     ↓
[BaseCommander]
     ↓
────────────────────→ via network
                     ↓
              [BaseController]
                     ↓
              [SubController]
                     ↓
           [Robot executes command]
```

---

## 🧩 Custom Robot Integration

### Option 1: Register model in base controller

```python
if robot_model == 'gerri':
  from gerri.robot.examples.sample_robot.sample_sub_controller import SampleSubController

  return SampleSubController()
```

### Option 2: Inject sub-controller manually

```python
from gerri.robot.examples.sample_robot.sample_base_controller import SampleBaseController
from gerri.robot.examples.sample_robot.sample_sub_controller import SampleSubController

robot = SampleBaseController(ROBOT_INFO, controller=SampleSubController())
robot.connect()
```

---

## 🧩 Custom Commander Integration

Supports dependency injection and delayed binding:

```python
sub = SampleSubCommander()
base = SampleBaseCommander(ROBOT_INFO, commander=sub)
base.connect()
```

The `set_base_commander()` method binds both sides after instantiation to avoid circular reference.

---