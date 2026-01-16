# kuroko_isaac_player/observation_builder.py

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


@dataclass
class ObservationBuilder:
    """
    Build observation vector that matches Isaac Lab env.yaml (observations.policy.*).

    This builder currently supports the active terms found in your env.yaml:
      - base_ang_vel           (Imu.angular_velocity xyz)
      - velocity_commands      (Twist linear.x, linear.y, angular.z)
      - joint_pos              (joint_pos_rel = q - default_q)
      - joint_vel              (joint_vel_rel = dq)
      - actions                (last_action)
      - base_lin_acc_sens      (Imu.linear_acceleration xyz)

    Ordering MUST follow terms_in_order from env.yaml (filtered to dict terms).
    """

    policy_joint_names: List[str]
    terms_in_order: List[str]
    default_joint_pos: List[float]

    _n: int = field(init=False)
    _q_default: np.ndarray = field(init=False)
    last_action: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self._n = len(self.policy_joint_names)

        qd = np.asarray(self.default_joint_pos, dtype=np.float32).reshape(-1)
        if qd.shape[0] != self._n:
            raise ValueError(
                f"default_joint_pos length mismatch: expected {self._n}, got {qd.shape[0]}"
            )
        self._q_default = qd.copy()

        # last_action initialized to zeros so the first tick can run
        self.last_action = np.zeros((self._n,), dtype=np.float32)

    # -------------------------
    # Public helpers
    # -------------------------
    @property
    def obs_dim(self) -> int:
        return int(sum(self._term_dim(t) for t in self.terms_in_order))

    def reset_last_action(self) -> None:
        self.last_action[:] = 0.0

    def set_last_action(self, action: np.ndarray) -> None:
        """
        Update last_action buffer.

        action: shape (N,) or (1, N)
        """
        a = np.asarray(action, dtype=np.float32)
        if a.ndim == 2:
            if a.shape[0] != 1:
                raise ValueError(f"action batch must be (1, N), got {a.shape}")
            a = a.reshape(-1)
        if a.shape != (self._n,):
            raise ValueError(f"action shape must be ({self._n},), got {a.shape}")
        self.last_action[:] = a

    def debug_dump_observation_layout(self, logger) -> None:
        logger.info("==== Observation Layout Dump ====")

        idx = 0
        for term in self.terms_in_order:
            dim = self._term_dim(term)

            if term == "base_ang_vel":
                src = "/kuroko/sensors/imu/data.angular_velocity (x,y,z)"
            elif term == "velocity_commands":
                src = "/cmd_vel (linear.x, linear.y, angular.z)"
            elif term == "joint_pos":
                src = f"/joint_states.position -> joint_pos_rel (q - default_q), N={self._n}"
            elif term == "joint_vel":
                src = f"/joint_states.velocity or estimated dq -> joint_vel_rel, N={self._n}"
            elif term == "actions":
                src = f"last_action (policy output), N={self._n}"
            elif term == "base_lin_acc_sens":
                src = "/kuroko/sensors/imu/data.linear_acceleration (x,y,z)"
            else:
                src = "UNKNOWN"

            logger.info(f"[{idx:3d}:{idx+dim:3d}] {term:20s} dim={dim:2d} <- {src}")
            idx += dim

        logger.info(f"Total observation dim = {idx}")
        logger.info("=================================")

    # -------------------------
    # Core build
    # -------------------------
    def build(self, cmd: Twist, q: np.ndarray, dq: np.ndarray, imu: Imu) -> np.ndarray:
        """
        Build observation vector (float32) in the exact order of terms_in_order.

        q, dq must already be aligned to policy_joint_names order.
        """
        q = np.asarray(q, dtype=np.float32).reshape(-1)
        dq = np.asarray(dq, dtype=np.float32).reshape(-1)

        if q.shape != (self._n,):
            raise ValueError(f"q shape must be ({self._n},), got {q.shape}")
        if dq.shape != (self._n,):
            raise ValueError(f"dq shape must be ({self._n},), got {dq.shape}")

        parts: list[np.ndarray] = []

        for term in self.terms_in_order:
            if term == "base_ang_vel":
                parts.append(
                    np.array(
                        [
                            float(imu.angular_velocity.x),
                            float(imu.angular_velocity.y),
                            float(imu.angular_velocity.z),
                        ],
                        dtype=np.float32,
                    )
                )

            elif term == "velocity_commands":
                parts.append(
                    np.array(
                        [
                            float(cmd.linear.x),
                            float(cmd.linear.y),
                            float(cmd.angular.z),
                        ],
                        dtype=np.float32,
                    )
                )

            elif term == "joint_pos":
                # Isaac Lab: joint_pos_rel
                parts.append((q - self._q_default).astype(np.float32))

            elif term == "joint_vel":
                # Isaac Lab: joint_vel_rel (default vel is 0)
                parts.append(dq.astype(np.float32))

            elif term == "actions":
                parts.append(self.last_action.astype(np.float32))

            elif term == "base_lin_acc_sens":
                parts.append(
                    np.array(
                        [
                            float(imu.linear_acceleration.x),
                            float(imu.linear_acceleration.y),
                            float(imu.linear_acceleration.z),
                        ],
                        dtype=np.float32,
                    )
                )

            else:
                raise KeyError(f"Unsupported observation term: {term}")

        if not parts:
            raise ValueError("No observation parts built (terms_in_order is empty).")

        obs = np.concatenate(parts, axis=0).astype(np.float32)
        return obs

    # -------------------------
    # Term dim
    # -------------------------
    def _term_dim(self, term: str) -> int:
        if term in ("base_ang_vel", "velocity_commands", "base_lin_acc_sens"):
            return 3
        if term in ("joint_pos", "joint_vel", "actions"):
            return self._n
        raise KeyError(f"Unknown term for dim: {term}")
