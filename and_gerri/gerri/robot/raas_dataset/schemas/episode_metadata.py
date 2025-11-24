from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import json


# ================================================================
# Episode Metadata
# ================================================================
class EpisodeInfo(BaseModel):
    recording_folderpath: str
    file_path: str
    mission_info: Optional[str] = None
    prev_episode: Optional[str] = None
    failure_trajectory: Optional[str] = None
    recovered_trajectory: Optional[str] = None


# ================================================================
# Observation
# ================================================================
class Observation(BaseModel):
    front_camera_video_path: str
    rear_camera_video_path: str


# ================================================================
# Action
# ================================================================
class Action(BaseModel):
    vx: float
    vy: Optional[float] = None
    vth: float


# ================================================================
# Step Info
# ================================================================
class StepInfo(BaseModel):
    is_first: bool
    is_last: bool
    is_terminal: bool
    instructions: Optional[List[str]] = None
    observation: Observation
    reward: Optional[float] = None
    discount: Optional[float] = None
    action: Action


# ================================================================
# Episode Metadata + Step Collection
# ================================================================
class EpisodeMetaData(BaseModel):
    episode_info: EpisodeInfo
    steps: List[StepInfo]


# ================================================================
# __main__
# ================================================================
# ================================================================
# TF TypeSpec (dtype + shape)
# ================================================================
class TFType(BaseModel):
    dtype: str
    shape: Optional[List[int]] = None


# ================================================================
# RLDS EpisodeMetaData Schema Definition
# (TYPE SPEC ONLY — 실제 데이터 없음)
# ================================================================
class EpisodeMetaData(BaseModel):
    episode_metadata: Dict[str, TFType]
    steps: Dict[str, Any]


# ================================================================
# 스키마 정의 (네가 보여준 스크린샷 그대로)
# ================================================================
def build_episode_schema() -> EpisodeMetaData:

    schema = EpisodeMetaData(
        episode_metadata={
            "recording_folderpath": TFType(dtype="tf.Text"),
            "file_path": TFType(dtype="tf.Text"),
        },
        steps={
            "is_first": TFType(dtype="tf.bool"),
            "is_last": TFType(dtype="tf.bool"),
            "is_terminal": TFType(dtype="tf.bool"),

            "language_instruction": TFType(dtype="tf.Text"),
            "language_instruction_2": TFType(dtype="tf.Text"),
            "language_instruction_3": TFType(dtype="tf.Text"),

            "observation": {
                "front_camera_video_path": TFType(dtype="tf.Text"),
                "rear_camera_video_path": TFType(dtype="tf.Text"),
            },

            "action_dict": {
                "vx": TFType(dtype="tf.float32"),
                "vy": TFType(dtype="tf.float32"),
                "vth": TFType(dtype="tf.float32"),
            },

            "discount": TFType(dtype="tf.float32"),
            "reward": TFType(dtype="tf.float32"),

            # raw action vector (e.g. 3-dim velocity)
            "action": TFType(dtype="tf.float64", shape=[3]),
        }
    )

    return schema


# ================================================================
# JSON 저장 함수
# ================================================================
def save_schema(schema: EpisodeMetaData, file_path: str):
    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(schema.model_dump(exclude_none=True), f, indent=4, ensure_ascii=False)

    print(f"[OK] RLDS schema saved to {file_path}")


# ================================================================
# TEST (__main__)
# ================================================================
if __name__ == "__main__":

    schema = build_episode_schema()

    print(json.dumps(schema.model_dump(exclude_none=True), indent=4, ensure_ascii=False))

    # save_schema(schema, "/home/keti/RaaS_dataset/schema.json")