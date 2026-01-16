from __future__ import annotations

from typing import Dict, List, Any

import yaml


def load_controller_joint_names(controller_yaml_path: str) -> List[str]:
    """Parse gz_position_controller.yaml.

    Expected structure (as provided):
      joint_group_position_controller:
        ros__parameters:
          joints: [ ... ]
    """
    with open(controller_yaml_path, "r") as f:
        data: Any = yaml.safe_load(f)

    try:
        joints = data["joint_group_position_controller"]["ros__parameters"]["joints"]
        if not isinstance(joints, list) or not all(isinstance(x, str) for x in joints):
            raise KeyError
        return joints
    except Exception:
        raise RuntimeError(
            "controller yaml does not contain joint_group_position_controller.ros__parameters.joints"
        )


def build_index_map_by_name(src_names: List[str], dst_names: List[str], what: str) -> List[int]:
    dst_index: Dict[str, int] = {n: i for i, n in enumerate(dst_names)}
    mapping: List[int] = []
    missing: List[str] = []
    for n in src_names:
        if n not in dst_index:
            missing.append(n)
        else:
            mapping.append(dst_index[n])

    if missing:
        raise RuntimeError(
            f"Missing joints while building mapping '{what}'. Not found in destination list: {missing}"
        )

    return mapping
