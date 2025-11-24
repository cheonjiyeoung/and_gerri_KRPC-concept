import h5py
import numpy as np
from typing import Any, Dict, List

def save_dict_to_h5(group, data: Dict[str, Any]):
    """dict → h5 그룹에 재귀적으로 저장"""
    for key, value in data.items():
        if value is None:
            continue

        key = str(key)

        if isinstance(value, dict):
            subgroup = group.create_group(key)
            save_dict_to_h5(subgroup, value)

        elif isinstance(value, list):
            # 숫자 리스트는 dataset
            if all(isinstance(x, (int, float)) for x in value):
                group.create_dataset(key, data=np.array(value))
            else:
                # 리스트 안에 dict 등
                subgroup = group.create_group(key)
                for idx, item in enumerate(value):
                    if isinstance(item, dict):
                        sub = subgroup.create_group(str(idx))
                        save_dict_to_h5(sub, item)
                    else:
                        subgroup.create_dataset(str(idx), data=str(item).encode("utf-8"))

        elif isinstance(value, str):
            group.create_dataset(key, data=value.encode("utf-8"))

        elif isinstance(value, (int, float, np.number)):
            group.create_dataset(key, data=value)

        elif isinstance(value, np.ndarray):
            group.create_dataset(key, data=value)

        else:
            # fallback→string
            group.create_dataset(key, data=str(value).encode("utf-8"))

def save_episode_to_h5(episode: EpisodeMetaData, save_path: str):
    """
    EpisodeMetaData(메타 + 전체 스텝)를 HDF5 1개 파일에 저장
    RLDS/DROID 구조와 동일한 형식
    """
    ep_dict = episode.model_dump(by_alias=True, exclude_none=True)
    steps: List[StepInfo] = episode.steps
    N = len(steps)

    with h5py.File(save_path, "w") as f:

        # -----------------------------
        # 1) Episode Metadata 저장
        # -----------------------------
        meta_group = f.create_group("episode_metadata")
        save_dict_to_h5(meta_group, ep_dict["episode_info"])

        # 에피소드 길이 저장
        f["episode_length"] = N

        # -----------------------------
        # 2) Steps (벡터화하여 저장)
        # -----------------------------
        steps_group = f.create_group("steps")

        # 기본 필드 (N 길이 벡터)
        is_first     = np.array([s.is_first     for s in steps], dtype=np.bool_)
        is_last      = np.array([s.is_last      for s in steps], dtype=np.bool_)
        is_terminal  = np.array([s.is_terminal  for s in steps], dtype=np.bool_)
        reward       = np.array([s.reward       for s in steps], dtype=np.float32)
        discount     = np.array([s.discount     for s in steps], dtype=np.float32)

        steps_group.create_dataset("is_first",    data=is_first)
        steps_group.create_dataset("is_last",     data=is_last)
        steps_group.create_dataset("is_terminal", data=is_terminal)
        steps_group.create_dataset("reward",      data=reward)
        steps_group.create_dataset("discount",    data=discount)

        # -----------------------------
        # 2-1) Action 저장 (shape: N x 3)
        # -----------------------------
        action_array = np.array(
            [[s.action["vx"], s.action.get("vy", 0.0), s.action["vth"]]
             for s in ep_dict["steps"]], dtype=np.float64
        )
        steps_group.create_dataset("action", data=action_array)

        # -----------------------------
        # 2-2) observation / metadata (계층 구조)
        # -----------------------------
        obs_group = steps_group.create_group("observation")
        meta2_group = steps_group.create_group("metadata")

        # step 단위로 observation 저장
        for idx, s in enumerate(ep_dict["steps"]):
            # observation/<idx> group
            og = obs_group.create_group(str(idx))
            save_dict_to_h5(og, s["observation"])

            # metadata/<idx>
            mg = meta2_group.create_group(str(idx))
            save_dict_to_h5(mg, s["metadata"])

    print(f"[OK] Episode saved to HDF5: {save_path}")

if __name__ == "__main__":
    import json

    rs = RobotStatus(robot_id="gyd_mobile_01", model="gyd", category="mobile")

    # observation 변환
    obs = robot_status_to_observation(rs)
    action_value = rs.velocity["2d"]

    steps = []
    for i in range(5):
        step = Step(
            step=i,
            observation=obs,
            action=action_value,
            reward=1.0,
            discount=1.0,
            metadata=StepMetadata(timestamp="20250101_120000_000", uuid=f"uuid-{i}")
        )
        steps.append(step)

    episode_info = EpisodeInfo(
        recording_folderpath="/home/keti/RaaS_dataset/test_episode",
        file_path="/home/keti/RaaS_dataset/test_episode/episode.h5",
        mission_info="test mission"
    )

    episode = EpisodeMetaData(
        episode_info=episode_info,
        steps=steps
    )

    save_episode_to_h5(episode, episode_info.file_path)

