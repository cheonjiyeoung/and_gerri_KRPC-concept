
# 🤖 Hello Universe Robot Project

A modular and extensible framework for robot control using the GERRI platform and `AdaptiveNetworkDaemon`.  
This repository provides a sample setup for robot and operator components, supporting manipulators and mobile bases.

---

## 🗂 Project Structure Overview

```
and_gerri/
├── gerri/                       # GERRI(Global Extended Robot Remote Interface)
│   ├── operator/                # Operator-side modules
│   │   ├── commander/           # Command logic (base + per-robot)
│   │   ├── interface/           # Input devices (keyboard, VR, master arm, etc.)
│   │   ├── examples/            # Robot-specific implementations (e.g., Piper)
│   │   └── ...
│   ├── robot/                   # Robot-side modules
│   │   ├── controller/          # Execution logic (base + per-robot)
│   │   ├── interface/           # Onboard sensors, emergency buttons, etc.
│   │   ├── examples/            # Robot-specific implementations (e.g., Piper, Gyd)
│   │   └── ...
├── _and_/                       # Adaptive Network Daemon core
├── utils/                       # Utility scripts
├── hello_universe_*             # Entry scripts for robot/operator
├── requirements.txt             # Python dependencies
├── install.sh                   # Setup script
└── README.md                    # This file
```

---

## 🚀 Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/your-org/and_gerri.git
cd and_gerri
```

### 2. Set up the environment

Run the setup script (Tested on Ubuntu):

```bash
bash install.sh
```

This will:
- Install Python 3.10 and system packages
- Create a virtual environment
- Install all Python dependencies

Activate the virtual environment:

```bash
source venv/bin/activate
```

---

## 🤖 Running the Robot

```bash
python hello_universe_robot.py
```

This script:
- Starts `AdaptiveNetworkDaemon`
- Connects to a robot controller based on `ROBOT_INFO`
- Runs the robot indefinitely

You can modify the default configuration in:
```python
hello_universe_config.py
```

---

## 🧠 How It Works

### Core Components

| File | Description |
|------|-------------|
| `hello_universe_robot.py` | Main entry point for running a robot node |
| `sample_base_controller.py` | Base class that handles message routing and robot connection |
| `sample_sub_controller.py` | Hardware abstraction layer — you define your robot behavior here |
| `hello_universe_config.py` | Central configuration (robot ID, model, camera/audio settings, etc.) |

### Controller Chain

```
SampleBaseController
      └── SampleSubController
               └── Your actual robot code
```

---

## 🛠 Customization

### Add your robot behavior
Edit `sample_sub_controller.py` and define methods like:

```python
def joint_ctrl(self, joint_angles: list):
    print(f"Move joints: {joint_angles}")
```

### Add a new robot model
Edit the `_initialize_robot()` method in `sample_base_controller.py`:

```python
if robot_model == 'my_robot':
    from gerri.robot.examples.my_robot.my_robot_controller import MyRobotController
    return MyRobotController(...)
```

---

## 🔐 Operator Setup

To implement an operator for teleoperation:

```bash
python hello_universe_operator.py
```

*Coming soon: Joystick, VR Tracker, Master Arm control examples*

---


## 🧠 System Architecture Overview

The system consists of two major sides:

### 1. Operator Side (`gerri/operator/`)
Responsible for user input and control logic.

#### ▸ `interface/`
Manages **input devices** like:
- Keyboard/Mouse
- VR Tracker
- Joysticks
- Master Arms

#### ▸ `commander/`
Divided into:
- **Base Commander**: Handles message formatting and dispatch.
- **Sub Commander**: Implements control logic for specific robots (e.g., Piper).

Example:
- `ManipulatorCommander` → base commander for manipulators
- `PiperCommander` → sub commander for Piper robot

Flow:
1. Operator input (keyboard, master arm, etc.)
2. → Sub Commander: interprets command
3. → Base Commander: formats message
4. → Sends to remote Controller

---

### 2. Robot Side (`gerri/robot/`)
Responsible for robot-side execution.

#### ▸ `interface/`
Handles **robot-side sensors and emergency devices**, such as:
- Emergency stop buttons
- Ultrasonic sensors
- LIDAR
- Encoders

#### ▸ `controller/`
Divided into:
- **Base Controller**: Handles message parsing and dispatch.
- **Sub Controller**: Implements hardware logic for a specific robot.

Example:
- `ManipulatorController` → base controller
- `PiperController` → sub controller for Piper robot

Flow:
1. Base Controller receives command
2. → Sub Controller executes control (e.g., joint angles, gripper, etc.)

---

### 🔄 Command Flow Summary

```text
[ Interface ]
    ↓ (e.g. key press, master arm motion)
[ Sub Commander ]
    ↓
[ Base Commander ]
    ↓  send formatted command
────────────────────────────→ (Remote)
                             ↓
                      [ Base Controller ]
                             ↓
                      [ Sub Controller ]
                             ↓
                  Executes robot motion
```

This architecture enables **modular** extension and hardware abstraction between operator and robot logic.


---

## 🧩 Integrating a New Robot

There are **two ways** to add a new robot controller:

### Option 1: Register inside Base Controller

Edit the `_initialize_robot()` method in your base controller:

```python
if robot_model == 'my_robot':
    from gerri.robot.examples.my_robot.my_robot_controller import MyRobotController
    return MyRobotController(port="your-port", ...)
```

This lets your base controller automatically instantiate the correct sub-controller based on `robot_model`.

---

### Option 2: Direct Controller Injection

In your robot entry script (e.g., `hello_universe_robot.py`), instantiate your robot directly:

```python
from gerri.robot.examples.sample_robot.sample_base_controller import SampleBaseController
from gerri.robot.examples.sample_robot.sample_sub_controller import SampleSubController

robot = SampleBaseController(ROBOT_INFO, controller=SampleSubController)
robot.connect()
```

This gives you **explicit control** over which sub-controller to use.

Use this method for rapid prototyping or when bypassing automatic robot model detection.

