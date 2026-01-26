# kuroko_isaac_player/policy_loader.py

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

try:
    import torch  # type: ignore
    import torch.nn as nn  # type: ignore
except Exception as exc:  # noqa: BLE001
    raise RuntimeError(
        "PyTorch is required to run the policy. Install it in your environment (pip/conda)."
    ) from exc


def resolve_policy_path(
    policy_dir: str,
    *,
    policy_path: Optional[str] = None,
    prefer_exported: bool = True,
    logger: Any = None,
) -> str:
    """
    Resolve the policy file path.

    Behavior:
      - If policy_path is provided and exists, return it as-is.
      - Otherwise search inside policy_dir:
          1) exported/policy.pt (preferred)
          2) exported/*.pt
          3) policy_dir/policy.pt
          4) policy_dir/*.pt
        and pick the "best" candidate.

    Notes:
      - We do NOT attempt to validate TorchScript here. The actual loader will try torch.jit.load first.
      - This is designed to avoid missing Isaac Lab's common output: <run>/exported/policy.pt
    """
    base = Path(policy_dir)

    if policy_path:
        p = Path(policy_path)
        if p.is_file():
            return str(p)
        # If user gave a relative path, try relative to policy_dir as well.
        p2 = base / policy_path
        if p2.is_file():
            return str(p2)
        raise FileNotFoundError(f"policy_path does not exist: {policy_path} (also tried {p2})")

    def _mtime(path: Path) -> float:
        try:
            return path.stat().st_mtime
        except Exception:
            return 0.0

    candidates: List[Path] = []

    exported = base / "exported"
    if prefer_exported and exported.is_dir():
        # Most common Isaac Lab output
        p = exported / "policy.pt"
        if p.is_file():
            candidates.append(p)

        # Any other .pt inside exported/
        candidates.extend(sorted(exported.glob("*.pt")))

    # Also consider top-level
    p = base / "policy.pt"
    if p.is_file():
        candidates.append(p)
    candidates.extend(sorted(base.glob("*.pt")))

    # De-duplicate while keeping order
    seen: set[Path] = set()
    uniq: List[Path] = []
    for c in candidates:
        c_abs = c.resolve()
        if c_abs in seen:
            continue
        seen.add(c_abs)
        uniq.append(c)

    # Filter obvious non-policy artifacts if they exist
    # (keep this conservative; only remove patterns that are very unlikely to be a deployable policy)
    filtered: List[Path] = []
    for c in uniq:
        name = c.name.lower()
        if name.endswith(".pt") and any(x in name for x in ["optimizer", "replay", "buffer"]):
            continue
        filtered.append(c)

    if not filtered:
        raise FileNotFoundError(
            f"No .pt policy file found under: {base} (searched exported/ and top-level)."
        )

    # Prefer exact "policy.pt" names first (exported/policy.pt already first if prefer_exported=True).
    exact_policy = [c for c in filtered if c.name == "policy.pt"]
    if exact_policy:
        # If multiple exist, pick the newest by mtime.
        best = max(exact_policy, key=_mtime)
        if logger:
            logger.info(f"Resolved policy path: {best} (picked newest policy.pt)")
        return str(best)

    # Otherwise pick newest .pt by mtime.
    best = max(filtered, key=_mtime)
    if logger:
        logger.info(f"Resolved policy path: {best} (picked newest .pt)")
    return str(best)


def _to_numpy(x: Any) -> np.ndarray:
    if isinstance(x, (tuple, list)):
        x = x[0]
    if not isinstance(x, torch.Tensor):
        raise TypeError(f"Policy output must be torch.Tensor (or tuple/list of it), got: {type(x)}")
    return x.detach().cpu().numpy().astype(np.float32)


def _looks_like_state_dict(d: Any) -> bool:
    if not isinstance(d, dict):
        return False
    # Heuristic: keys like "...weight"/"...bias" and tensor values
    has_weight = any(isinstance(k, str) and k.endswith(".weight") for k in d.keys())
    has_tensor = any(isinstance(v, torch.Tensor) for v in d.values())
    return has_weight and has_tensor


def _extract_state_dict(ckpt: Any) -> Dict[str, torch.Tensor]:
    """
    Extract a state_dict from a checkpoint-like object.

    Supports:
      - raw state_dict dict
      - dict wrappers: state_dict / model_state_dict / policy_state_dict / etc.
      - nested dicts
    """
    if _looks_like_state_dict(ckpt):
        return ckpt  # type: ignore

    if isinstance(ckpt, dict):
        candidates = [
            "state_dict",
            "model_state_dict",
            "policy_state_dict",
            "actor_state_dict",
            "ac_state_dict",
            "net_state_dict",
            "model",
            "policy",
            "module",
            "agent",
        ]
        for k in candidates:
            v = ckpt.get(k, None)
            if _looks_like_state_dict(v):
                return v  # type: ignore

        # Sometimes it's nested deeper; scan one level
        for _, v in ckpt.items():
            if _looks_like_state_dict(v):
                return v  # type: ignore

    raise RuntimeError("Could not find a state_dict inside the .pt checkpoint.")


def _strip_prefix(state_dict: Dict[str, torch.Tensor], prefix: str) -> Dict[str, torch.Tensor]:
    if not prefix:
        return dict(state_dict)
    plen = len(prefix)
    out: Dict[str, torch.Tensor] = {}
    for k, v in state_dict.items():
        if isinstance(k, str) and k.startswith(prefix):
            out[k[plen:]] = v
    return out


def _extract_actor_state_dict(state_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
    """
    Try to extract the actor/policy network parameters from a bigger state_dict.

    We attempt a set of common prefixes used by Isaac Lab stacks (rl-games/skrl/rsl-rl/etc).
    We pick the prefix that yields the most parameters.
    """
    prefixes = [
        "actor.",
        "policy.actor.",
        "a2c_network.actor.",
        "a2c_network.actor_mlp.",
        "a2c_network.actor_net.",
        "model.actor.",
        "ac.actor.",
        "actor_mlp.",
        "actor_net.",
        "pi.",
        "policy.",
        "module.actor.",
        "module.policy.actor.",
    ]

    best_sd: Optional[Dict[str, torch.Tensor]] = None
    best_count = 0

    for p in prefixes:
        sub = _strip_prefix(state_dict, p)
        if len(sub) > best_count and any(isinstance(k, str) and k.endswith(".weight") for k in sub.keys()):
            best_sd = sub
            best_count = len(sub)

    # If no prefix matched, assume state_dict itself is already actor
    actor_sd = best_sd if best_sd is not None else dict(state_dict)

    # Also remove common wrapper prefixes like "net." or "model."
    # Do this only if it increases "linear weight" matches.
    def score(sd: Dict[str, torch.Tensor]) -> int:
        return sum(1 for k in sd.keys() if isinstance(k, str) and k.endswith(".weight"))

    for p in ["net.", "model.", "mlp.", "actor."]:
        stripped = _strip_prefix(actor_sd, p)
        if score(stripped) > score(actor_sd):
            actor_sd = stripped

    return actor_sd


def _infer_mlp_io(actor_sd: Dict[str, torch.Tensor]) -> Tuple[int, int]:
    """
    Infer input/output dimensions from linear layer weights in an MLP-like actor state_dict.

    We sort weight keys by the first numeric segment (Sequential index) if present.
    - in_dim  = first weight's in_features  (shape[1])
    - out_dim = last  weight's out_features (shape[0])
    """
    weights: List[Tuple[str, torch.Tensor]] = []
    for k, v in actor_sd.items():
        if (
            isinstance(k, str)
            and k.endswith(".weight")
            and isinstance(v, torch.Tensor)
            and v.ndim == 2
        ):
            weights.append((k, v))

    if not weights:
        raise RuntimeError("No 2D weight tensors found in actor state_dict; cannot infer MLP IO dims.")

    def order(name: str) -> Tuple[int, str]:
        # Find first integer token (Sequential index)
        for part in name.split("."):
            if part.isdigit():
                return (int(part), name)
        return (10**9, name)

    weights.sort(key=lambda kv: order(kv[0]))

    in_dim = int(weights[0][1].shape[1])
    out_dim = int(weights[-1][1].shape[0])
    return in_dim, out_dim


def load_policy_callable(
    policy_path: str,
    *,
    device: str = "cpu",
    hidden: Tuple[int, int, int] = (512, 256, 128),
    activation: str = "elu",
    logger: Any = None,
) -> Callable[[np.ndarray], np.ndarray]:
    """
    Returns a callable: obs_np -> action_np

    Loading order:
      1) TorchScript archive via torch.jit.load
      2) torch.save(nn.Module) via torch.load
      3) checkpoint/state_dict via torch.load -> extract actor state_dict -> build MLP (512/256/128 default)

    Notes:
      - This assumes the actor is a deterministic MLP outputting joint targets directly.
      - If your policy is Gaussian (mean/log_std) or uses RNNs, TorchScript export is recommended.
    """
    dev = torch.device(device)

    # 1) TorchScript
    try:
        policy = torch.jit.load(str(policy_path), map_location=dev)
        policy.eval()
        if logger:
            logger.info("Loaded policy as TorchScript (torch.jit.load).")

        def _call(obs_np: np.ndarray) -> np.ndarray:
            obs = torch.from_numpy(obs_np).to(dev)
            with torch.no_grad():
                out = policy(obs)
            return _to_numpy(out)

        return _call
    except Exception as exc:  # noqa: BLE001
        if logger:
            logger.warn(
                f"TorchScript load failed ({type(exc).__name__}: {exc}). Falling back to torch.load checkpoint."
            )

    # 2/3) torch.load
    ckpt = torch.load(str(policy_path), map_location=dev)

    # 2) nn.Module
    if isinstance(ckpt, nn.Module):
        ckpt.eval()
        if logger:
            logger.info("Loaded policy as torch.nn.Module (torch.load).")

        def _call(obs_np: np.ndarray) -> np.ndarray:
            obs = torch.from_numpy(obs_np).to(dev)
            with torch.no_grad():
                out = ckpt(obs)
            return _to_numpy(out)

        return _call

    # 3) checkpoint/state_dict
    state_dict = _extract_state_dict(ckpt)
    actor_sd = _extract_actor_state_dict(state_dict)

    in_dim, out_dim = _infer_mlp_io(actor_sd)

    act_cls = {
        "relu": nn.ReLU,
        "tanh": nn.Tanh,
        "elu": nn.ELU,
        "leaky_relu": nn.LeakyReLU,
        "gelu": nn.GELU,
    }.get(activation.lower(), nn.ELU)

    h1, h2, h3 = hidden
    model = nn.Sequential(
        nn.Linear(in_dim, h1),
        act_cls(),
        nn.Linear(h1, h2),
        act_cls(),
        nn.Linear(h2, h3),
        act_cls(),
        nn.Linear(h3, out_dim),
    ).to(dev)

    missing, unexpected = model.load_state_dict(actor_sd, strict=False)
    model.eval()

    if logger:
        if missing:
            logger.warn(f"Missing keys when loading actor state_dict: {missing}")
        if unexpected:
            logger.warn(f"Unexpected keys when loading actor state_dict: {unexpected}")
        logger.info(
            f"Loaded policy from checkpoint as MLP: obs_dim={in_dim}, act_dim={out_dim}, hidden={list(hidden)}"
        )

    def _call(obs_np: np.ndarray) -> np.ndarray:
        obs = torch.from_numpy(obs_np).to(dev)
        with torch.no_grad():
            out = model(obs)
        return _to_numpy(out)

    return _call
