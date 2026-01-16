# kuroko_isaac_player/observation_builder.py

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


def _quat_to_rotmat_xyzw(q: Tuple[float, float, float, float]) -> np.ndarray:
    # q = (x, y, z, w)
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


@dataclass
class ObservationBuilder:
    """
    Build observation vector following env.yaml active observation terms order.

    terms_in_order must already be filtered using env.yaml observations specs:
      - null terms removed
      - pure boolean terms removed

    Supported terms (for now):
      - base_ang_vel (3)          -> IMU angular_velocity
      - projected_gravity (3)     -> computed from IMU orientation
      - velocity_commands (3)     -> cmd_vel (vx, vy, wz)
      - joint_pos (N)             -> q - q_default
      - joint_vel (N)             -> dq
      - actions (N)               -> last_action
      - base_lin_acc_sens (3)     -> IMU linear_acceleration

    If your env.yaml marks additional terms as active (dict) we should implement them here.
    """

    policy_joint_names: List[str]
    terms_in_order: List[str]
    default_joint_pos: Sequence[float]

    last_action: np.ndarray = field(default_factory=lambda: np.zeros((0,), dtype=np.float32))

    def __post_init__(self):
        self._n = len(self.policy_joint_names)

        if self.default_joint_pos is None:
            self._q_default = np.zeros((self._n,), dtype=np.float32)
        else:
            if len(self.default_joint_pos) != self._n:
                raise ValueError(
                    f"default_joint_pos length {len(self.default_joint_pos)} != number of policy joints {self._n}"
                )
            self._q_default = np.array([float(x) for x in self.default_joint_pos], dtype=np.float32)

        if self.last_action.size == 0:
            self.last_action = np.zeros((self._n,), dtype=np.float32)
        elif self.last_action.shape != (self._n,):
            raise ValueError(f"last_action shape must be ({self._n},), got {self.last_action.shape}")

        self._obs_dim = 0
        for t in self.terms_in_order:
            self._obs_dim += self._term_dim(t)

    @property
    def obs_dim(self) -> int:
        return self._obs_dim

    @property
    def filtered_terms(self) -> List[str]:
        # Kept for compatibility with existing node logs
        return list(self.terms_in_order)
        
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

    def reset_last_action(self):
        self.last_action[:] = 0.0

    def _term_dim(self, term: str) -> int:
        if term in ("base_ang_vel", "projected_gravity", "velocity_commands", "base_lin_acc_sens"):
            return 3
        if term in ("joint_pos", "joint_vel", "actions"):
            return self._n
        raise ValueError(f"Unsupported observation term: {term}")

    def build(self, cmd: Twist, q: np.ndarray, dq: np.ndarray, imu: Imu) -> np.ndarray:
        parts: List[np.ndarray] = []

        # projected_gravity (in base frame)
        ox, oy, oz, ow = (
            float(imu.orientation.x),
            float(imu.orientation.y),
            float(imu.orientation.z),
            float(imu.orientation.w),
        )
        R = _quat_to_rotmat_xyzw((ox, oy, oz, ow))
        g_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        proj_g = R.T @ g_world  # (3,)

        for term in self.terms_in_order:
            if term == "base_ang_vel":
                w = imu.angular_velocity
                parts.append(np.array([w.x, w.y, w.z], dtype=np.float32))

            elif term == "projected_gravity":
                parts.append(proj_g.astype(np.float32))

            elif term == "velocity_commands":
                parts.append(np.array([cmd.linear.x, cmd.linear.y, cmd.angular.z], dtype=np.float32))

            elif term == "joint_pos":
                parts.append(q.astype(np.float32) - self._q_default)

            elif term == "joint_vel":
                parts.append(dq.astype(np.float32))

            elif term == "actions":
                parts.append(self.last_action.astype(np.float32))

            elif term == "base_lin_acc_sens":
                a = imu.linear_acceleration
                parts.append(np.array([a.x, a.y, a.z], dtype=np.float32))

            else:
                raise ValueError(f"Unsupported observation term: {term}")
        if not parts:
            return np.zeros((1, 0), dtype=np.float32)
        obs = np.concatenate(parts, axis=0).astype(np.float32)
        return obs.reshape(1, -1)
