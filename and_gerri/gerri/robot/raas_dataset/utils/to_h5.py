import h5py
import numpy as np

def save_dict_to_h5(group, data):
    """
    dict → h5 파일에 재귀적으로 저장
    group: h5py Group
    data: dict/list/scalar
    """
    for key, value in data.items():

        if value is None:
            continue

        # 1) dict → subgroup
        if isinstance(value, dict):
            subgroup = group.create_group(str(key))
            save_dict_to_h5(subgroup, value)

        # 2) 리스트 처리
        elif isinstance(value, list):
            # 숫자 리스트 → dataset
            if all(isinstance(x, (int, float)) for x in value):
                group.create_dataset(str(key), data=np.array(value))
            else:
                # 리스트 안에 dict 등 복잡한 구조
                subgroup = group.create_group(str(key))
                for idx, item in enumerate(value):
                    if isinstance(item, dict):
                        item_group = subgroup.create_group(str(idx))
                        save_dict_to_h5(item_group, item)
                    else:
                        subgroup.create_dataset(str(idx), data=str(item).encode('utf-8'))

        # 3) 문자열
        elif isinstance(value, str):
            group.create_dataset(str(key), data=value.encode('utf-8'))

        # 4) 숫자
        elif isinstance(value, (int, float)):
            group.create_dataset(str(key), data=value)

        # 5) numpy array
        elif isinstance(value, np.ndarray):
            group.create_dataset(str(key), data=value)

        # 6) boolean
        elif isinstance(value, bool):
            group.create_dataset(str(key), data=int(value))

        else:
            # fallback → string 저장
            group.create_dataset(str(key), data=str(value).encode('utf-8'))


def save_step_to_h5(step_dict: dict, save_path: str):
    """
    Step(dict) → HDF5 파일 저장
    """
    with h5py.File(save_path, "w") as f:
        save_dict_to_h5(f, step_dict)

    print(f"[OK] Step saved to HDF5: {save_path}")
