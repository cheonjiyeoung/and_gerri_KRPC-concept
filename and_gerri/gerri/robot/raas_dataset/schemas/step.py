from pydantic import BaseModel, Field
from typing import Optional, Dict, List, Tuple


# ===========================================================
# Robot Type / Robot Info
# ===========================================================

class RobotSize(BaseModel):
    width: Optional[float]
    length: Optional[float]
    height: Optional[float]


class RobotType(BaseModel):
    category: str
    model: str
    footprint: Optional[Dict] = None
    size: RobotSize


# ===========================================================
# Pose
# ===========================================================

class Position3D(BaseModel):
    x: Optional[float]
    y: Optional[float]
    z: Optional[float]


class Orientation(BaseModel):
    yaw: Optional[float]
    pitch: Optional[float]
    roll: Optional[float]


class Pose2D(BaseModel):
    x: Optional[float]
    y: Optional[float]
    th: Optional[float]


class Pose(BaseModel):
    position: Position3D
    orientation: Orientation
    pose_2d: Optional[Pose2D] = Field(None, alias="2d")


# ===========================================================
# Battery
# ===========================================================

class BatteryState(BaseModel):
    voltage: Optional[float]
    current: Optional[float]
    capacity: Optional[float]
    percentage: Optional[float]
    temperature: Optional[float]
    now_charging: Optional[bool]


# ===========================================================
# Velocity
# ===========================================================

class LinearVelocity(BaseModel):
    x: Optional[float]
    y: Optional[float]
    z: Optional[float]


class AngularVelocity(BaseModel):
    x: Optional[float]
    y: Optional[float]
    z: Optional[float]


class Velocity2D(BaseModel):
    vx: Optional[float]
    vy: Optional[float]
    vth: Optional[float]


class Velocity(BaseModel):
    linear: LinearVelocity
    angular: AngularVelocity
    vel_2d: Optional[Velocity2D] = Field(None, alias="2d")


# ===========================================================
# Sensor Info
# ===========================================================

class Sensor(BaseModel):
    ultrasonic: Optional[List]
    laser: Optional[List[List[float]]]


# ===========================================================
# Map
# ===========================================================

class MapInfo(BaseModel):
    name: Optional[str]
    origin: Optional[List[float]]
    size: Optional[List[float]]
    resolution: Optional[float]


# ===========================================================
# Path
# ===========================================================

class PathPlan(BaseModel):
    global_: Optional[List[List[float]]] = None  # JSON key "global"
    local: Optional[List] = None


# ===========================================================
# Joint State
# ===========================================================

class JointState(BaseModel):
    name: Optional[List]
    position: Optional[List]
    velocity: Optional[List]
    effort: Optional[List]

# ===========================================================
# Observation Root
# ===========================================================

class RobotState(BaseModel):
    mode: Optional[str]
    operating_state: Optional[List[str]]


class Observation(BaseModel):
    robot_id: str
    robot_type: RobotType
    robot_state: Optional[RobotState]
    pose: Pose
    battery_state: BatteryState
    velocity: Velocity
    sensor: Sensor
    map: MapInfo
    path_plan: PathPlan
    joint_state: JointState


# ===========================================================
# Metadata
# ===========================================================

class StepMetadata(BaseModel):
    timestamp: str
    uuid: str


# ===========================================================
# Step
# ===========================================================

class Step(BaseModel):
    step: int
    observation: Observation
    action: Optional[dict]     # 현재 JSON에서는 null
    reward: Optional[float]
    discount: Optional[float]
    metadata: StepMetadata

if __name__ == "__main__":
    import asyncio
    import json

    class RobotStatus:
        def __init__(self, robot_id=None, model=None, category=None):
            self.robot_id = robot_id
            self.robot_type = {
                "category": category,
                "model": model,
                "footprint": None,
                "size": {"width": 360, "length": 540, "height": 840}
            }

            self.robot_state = {"mode": "AUTO", "operating_state": ["IDLE"]}

            self.pose = {
                "position": {"x": 1.0, "y": 2.0, "z": 0.1},
                "orientation": {"yaw": 0.1, "pitch": 0.0, "roll": 0.0},
                "2d": {"x": 1.0, "y": 2.0, "th": 0.1}
            }

            self.velocity = {
                "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.03},
                "2d": {"vx": 0.1, "vy": None, "vth": 0.03}
            }

            self.battery_state = {
                "voltage": 24.5,
                "current": -1.2,
                "capacity": 2200,
                "percentage": 85,
                "temperature": 30.5,
                "now_charging": False
            }

            self.joint_state = {"name": None, "position": None, "velocity": None, "effort": None}

            self.sensor = {
                "ultrasonic": None,
                "laser": [[1.5, 0.3], [1.7, 0.2]],
                "imu": {"a": None, "g": None}
            }

            self.path_plan = {"global": None, "local": None}

            self.map = {
                "name": "KETI-SUSEO2F-3F",
                "origin": (36, 377),
                "size": (1477, 654),
                "resolution": 0.05
            }

            self.nav_state = {"goal": None, "result": None, "info": None, "dist": None, "mileage": None}

            self.current_floor = None
            self.nav_mode = None
            self.nav_start_event = asyncio.Event()
            self.nav_end_event = asyncio.Event()


    # ===========================================================
    # 스키마 기반 자동 매핑 함수
    # ===========================================================

    def extract_by_schema(schema: type[BaseModel], source: dict):
        result = {}

        for field_name, field_info in schema.model_fields.items():
            json_key = field_info.alias or field_name  # JSON에서 들어오는 키

            if json_key not in source:
                continue

            value = source[json_key]

            # nested model?
            if hasattr(field_info.annotation, "model_fields"):
                result[field_name] = extract_by_schema(field_info.annotation, value)
            else:
                result[field_name] = value

        return result


    def robot_status_to_observation(status: RobotStatus) -> Observation:
        status_dict = {
            "robot_id": status.robot_id,
            "robot_type": status.robot_type,
            "robot_state": status.robot_state,
            "pose": status.pose,
            "battery_state": status.battery_state,
            "velocity": status.velocity,
            "sensor": status.sensor,
            "map": status.map,
            "path_plan": status.path_plan,
            "joint_state": status.joint_state,
        }

        filtered = extract_by_schema(Observation, status_dict)
        return Observation.model_validate(filtered)


    # ===========================================================
    # 테스트 실행 (__main__)
    # ===========================================================

    # RobotStatus 생성
    rs = RobotStatus(robot_id="gyd_mobile_01", model="gyd", category="mobile")

    # 자동 변환 수행
    obs = robot_status_to_observation(rs)
    action_value = rs.velocity["2d"]
    # Step 생성
    step = Step(
        step=1,
        observation=obs,
        action=action_value,
        reward=1.0,
        discount=1.0,
        metadata=StepMetadata(timestamp="20250101_120000_000", uuid="abcd-1234")
    )

    # JSON 출력
    print(json.dumps(step.model_dump(by_alias=True,exclude_none=True), indent=4, ensure_ascii=False))