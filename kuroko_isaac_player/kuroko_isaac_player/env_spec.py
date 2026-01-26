# kuroko_isaac_player/env_spec.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List

import yaml


class _SafeLoaderWithIsaacTags(yaml.SafeLoader):
    """SafeLoader extension that accepts a minimal set of Hydra/OmegaConf tags."""


def _construct_python_tuple(loader: yaml.Loader, node: yaml.Node):
    # OmegaConf/Hydra may emit python/tuple tags; convert to list for simplicity.
    return list(loader.construct_sequence(node))


def _construct_builtins_slice(loader: yaml.Loader, node: yaml.Node):
    # Some Isaac Lab configs use a builtins.slice tag.
    if isinstance(node, yaml.SequenceNode):
        seq = loader.construct_sequence(node)
        start = stop = step = None
        if len(seq) == 1:
            stop = seq[0]
        elif len(seq) == 2:
            start, stop = seq
        elif len(seq) >= 3:
            start, stop, step = seq[0], seq[1], seq[2]
        return {"start": start, "stop": stop, "step": step}

    if isinstance(node, yaml.MappingNode):
        mp = loader.construct_mapping(node)
        return {"start": mp.get("start", None), "stop": mp.get("stop", None), "step": mp.get("step", None)}

    return {"start": None, "stop": None, "step": None}


_SafeLoaderWithIsaacTags.add_constructor("tag:yaml.org,2002:python/tuple", _construct_python_tuple)
_SafeLoaderWithIsaacTags.add_constructor(
    "tag:yaml.org,2002:python/object/apply:builtins.slice", _construct_builtins_slice
)


@dataclass(frozen=True)
class EnvSpec:
    raw: Dict[str, Any]
    policy_joint_names: List[str]
    default_joint_pos: List[float]
    observation_policy_tokens: List[str]
    active_observation_terms: List[str]

    @property
    def observation_terms_in_order(self) -> List[str]:
        return self.active_observation_terms


def _get(d: Any, path: List[str]) -> Any:
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            raise KeyError("Missing key path: " + ".".join(path))
        cur = cur[p]
    return cur


def extract_policy_joint_names(env: Dict[str, Any]) -> List[str]:
    names = _get(env, ["actions", "joint_pos", "joint_names"])
    if not isinstance(names, list) or not all(isinstance(x, str) for x in names):
        raise TypeError("actions.joint_pos.joint_names must be list[str]")
    return names


def extract_default_joint_pos(env: Dict[str, Any], policy_joint_names: List[str]) -> List[float]:
    """
    Must follow the real env.yaml:
      scene.robot.init_state.joint_pos: { joint_name: value, ... }

    Return list aligned with policy_joint_names.
    """
    jp = _get(env, ["scene", "robot", "init_state", "joint_pos"])
    if not isinstance(jp, dict):
        raise TypeError("scene.robot.init_state.joint_pos must be a dict")

    out: List[float] = []
    missing: List[str] = []
    for jn in policy_joint_names:
        if jn not in jp:
            missing.append(jn)
            out.append(0.0)
        else:
            out.append(float(jp[jn]))

    # Missing joints are not fatal here; the caller may log/handle it.
    return out


def extract_observations_policy_map(env: Dict[str, Any]) -> Dict[str, Any]:
    policy_map = _get(env, ["observations", "policy"])
    if not isinstance(policy_map, dict):
        raise TypeError(f"observations.policy must be a dict, got: {type(policy_map)}")
    return policy_map


def extract_observation_policy_tokens(env: Dict[str, Any]) -> List[str]:
    policy_map = extract_observations_policy_map(env)
    return [str(k) for k in policy_map.keys()]


def extract_active_observation_terms(env: Dict[str, Any]) -> List[str]:
    """
    Extract enabled observation terms from observations.policy.

    Isaac Lab env.yaml sometimes contains "meta" scalar keys inside observations.policy,
    e.g. concatenate_dim: 1. These are not observation terms and must not hard-fail.

    Rules:
      - dict  -> enabled if enable!=False (default True)
      - list  -> enabled
      - None/bool/int/float/str/... -> treated as meta/config; ignored
    """
    policy_map = extract_observations_policy_map(env)

    active: List[str] = []
    for k, v in policy_map.items():
        term = str(k)

        if isinstance(v, dict):
            if v.get("enable", True):
                active.append(term)
            continue

        if isinstance(v, list):
            active.append(term)
            continue

        # Skip meta/scalar fields (concatenate_dim, noise params, etc.)
        continue

    return active


def load_env_spec(path: str) -> EnvSpec:
    with open(path, "r") as f:
        data = yaml.load(f, Loader=_SafeLoaderWithIsaacTags)

    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise TypeError(f"env.yaml root must be dict, got: {type(data)}")

    policy_joint_names = extract_policy_joint_names(data)
    default_joint_pos = extract_default_joint_pos(data, policy_joint_names)

    obs_tokens = extract_observation_policy_tokens(data)
    active_terms = extract_active_observation_terms(data)

    return EnvSpec(
        raw=data,
        policy_joint_names=policy_joint_names,
        default_joint_pos=default_joint_pos,
        observation_policy_tokens=obs_tokens,
        active_observation_terms=active_terms,
    )
