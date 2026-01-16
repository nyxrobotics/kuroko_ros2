# kuroko_isaac_player/env_spec.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List

import yaml


# -----------------------------
# YAML loader for Isaac Lab / Hydra exports
# - !!python/tuple
# - !!python/object/apply:builtins.slice
# -----------------------------
class _SafeLoaderWithIsaacTags(yaml.SafeLoader):
    """SafeLoader extension that accepts a minimal set of Hydra/OmegaConf tags."""


def _construct_python_tuple(loader: yaml.Loader, node: yaml.Node):
    return list(loader.construct_sequence(node))


def _construct_builtins_slice(loader: yaml.Loader, node: yaml.Node):
    # Keep slice as a simple dict representation (we don't need it for playback)
    if isinstance(node, yaml.SequenceNode):
        seq = loader.construct_sequence(node)
        start = None
        stop = None
        step = None
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


# -----------------------------
# Public dataclass
# -----------------------------
@dataclass(frozen=True)
class EnvSpec:
    raw: Dict[str, Any]
    policy_joint_names: List[str]
    default_joint_pos: List[float]

    # tokens (keys) in observations.policy (includes config-ish keys)
    observation_policy_tokens: List[str]

    # actual vector terms (only dict entries in observations.policy; null/bool dropped)
    active_observation_terms: List[str]

    @property
    def observation_terms_in_order(self) -> List[str]:
        # what policy_player_node.py expects
        return self.active_observation_terms


# -----------------------------
# Helpers
# -----------------------------
def _get(d: Any, path: List[str]) -> Any:
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            raise KeyError("Missing key path: " + ".".join(path))
        cur = cur[p]
    return cur


def _get_opt(d: Any, path: List[str], default: Any = None) -> Any:
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur


# -----------------------------
# Extractors
# -----------------------------
def extract_policy_joint_names(env: Dict[str, Any]) -> List[str]:
    names = _get(env, ["actions", "joint_pos", "joint_names"])
    if not isinstance(names, list) or not all(isinstance(x, str) for x in names):
        raise TypeError("actions.joint_pos.joint_names must be list[str]")
    return names


def extract_default_joint_pos(env: Dict[str, Any], joint_count: int) -> List[float]:
    candidates = [
        ["actions", "joint_pos", "default_joint_pos"],
        ["actions", "joint_pos", "default"],
        ["defaults", "joint_pos"],
        ["robot", "default_joint_pos"],
    ]
    for c in candidates:
        v = _get_opt(env, c, None)
        if v is None:
            continue
        if isinstance(v, list) and all(isinstance(x, (int, float)) for x in v):
            if len(v) != joint_count:
                raise ValueError(f"{'.'.join(c)} length {len(v)} != joint_count {joint_count}")
            return [float(x) for x in v]
    return [0.0] * joint_count


def extract_observations_policy_map(env: Dict[str, Any]) -> Dict[str, Any]:
    """
    In your env.yaml, observations.policy is an ordered dict:
      observations:
        policy:
          concatenate_terms: true
          base_ang_vel: { ... }
          projected_gravity: null
          ...

    We keep it as-is (dict preserves insertion order in Python 3.7+).
    """
    policy_map = _get(env, ["observations", "policy"])
    if not isinstance(policy_map, dict):
        raise TypeError(f"observations.policy must be a dict in this env.yaml, got: {type(policy_map)}")
    return policy_map


def extract_observation_policy_tokens(env: Dict[str, Any]) -> List[str]:
    policy_map = extract_observations_policy_map(env)
    return [str(k) for k in policy_map.keys()]


def extract_active_observation_terms(env: Dict[str, Any]) -> List[str]:
    """
    Filter by actual contents in observations.policy:

      - value is None   -> drop
      - value is bool   -> drop
      - value is dict   -> keep (this term produces vector data)
    """
    policy_map = extract_observations_policy_map(env)

    active: List[str] = []
    for k, v in policy_map.items():
        term = str(k)
        if v is None:
            continue
        if isinstance(v, bool):
            continue
        if isinstance(v, dict):
            active.append(term)
            continue
        raise TypeError(f"Unsupported observations.policy.{term} type: {type(v)}")
    return active


# -----------------------------
# Loader entrypoint
# -----------------------------
def load_env_spec(path: str) -> EnvSpec:
    with open(path, "r") as f:
        data = yaml.load(f, Loader=_SafeLoaderWithIsaacTags)
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise TypeError(f"env.yaml root must be a mapping/dict, got: {type(data)}")

    policy_joint_names = extract_policy_joint_names(data)
    default_joint_pos = extract_default_joint_pos(data, joint_count=len(policy_joint_names))

    obs_tokens = extract_observation_policy_tokens(data)
    active_terms = extract_active_observation_terms(data)

    return EnvSpec(
        raw=data,
        policy_joint_names=policy_joint_names,
        default_joint_pos=default_joint_pos,
        observation_policy_tokens=obs_tokens,
        active_observation_terms=active_terms,
    )
