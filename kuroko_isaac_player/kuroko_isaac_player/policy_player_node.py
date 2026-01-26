from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from .env_spec import load_env_spec
from .observation_builder import ObservationBuilder
from .policy_loader import load_policy_callable, resolve_policy_path


def _load_controller_joint_names(controller_yaml_path: Path) -> list[str]:
    import yaml

    with open(controller_yaml_path, "r") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise TypeError(f"controller yaml root must be dict: {controller_yaml_path}")

    # Layout B: top-level joint_group_position_controller
    jgp = data.get("joint_group_position_controller", None)
    if isinstance(jgp, dict):
        ros_params = jgp.get("ros__parameters", None)
        if isinstance(ros_params, dict):
            joints = ros_params.get("joints", None)
            if isinstance(joints, list) and all(isinstance(x, str) for x in joints):
                return joints

    # Layout A: controller_manager subtree
    cur: Any = data
    for k in ["controller_manager", "ros__parameters", "joint_group_position_controller", "ros__parameters"]:
        if isinstance(cur, dict) and k in cur:
            cur = cur[k]
        else:
            cur = None
            break
    if isinstance(cur, dict):
        joints = cur.get("joints", None)
        if isinstance(joints, list) and all(isinstance(x, str) for x in joints):
            return joints

    # Fallback recursive search
    best: list[str] = []

    def walk(obj: Any) -> None:
        nonlocal best
        if isinstance(obj, dict):
            j = obj.get("joints", None)
            if isinstance(j, list) and all(isinstance(x, str) for x in j):
                if len(j) > len(best):
                    best = list(j)
            for v in obj.values():
                walk(v)
        elif isinstance(obj, list):
            for v in obj:
                walk(v)

    walk(data)
    if best:
        return best

    raise KeyError(f"Could not find joints list in controller yaml: {controller_yaml_path}")


class _DQEstimator:
    """Estimate dq from q differences if /joint_states.velocity is missing/invalid."""

    def __init__(self, n: int, cutoff_hz: float = 30.0) -> None:
        self._n = n
        self._prev_q: Optional[np.ndarray] = None
        self._prev_t: Optional[float] = None
        self._dq_filt = np.zeros((n,), dtype=np.float32)
        self._cutoff_hz = float(cutoff_hz)

    def reset(self) -> None:
        self._prev_q = None
        self._prev_t = None
        self._dq_filt[:] = 0.0

    def update(self, q: np.ndarray, t: float) -> np.ndarray:
        q = np.asarray(q, dtype=np.float32).reshape(-1)
        if q.shape[0] != self._n:
            raise ValueError("dq estimator: q length mismatch")

        if self._prev_q is None or self._prev_t is None:
            self._prev_q = q.copy()
            self._prev_t = float(t)
            return self._dq_filt.copy()

        dt = float(t - self._prev_t)
        if dt <= 1e-6:
            return self._dq_filt.copy()

        dq_raw = (q - self._prev_q) / dt

        if self._cutoff_hz <= 0.0:
            self._dq_filt = dq_raw
        else:
            rc = 1.0 / (2.0 * math.pi * self._cutoff_hz)
            alpha = dt / (dt + rc)
            self._dq_filt = (1.0 - alpha) * self._dq_filt + alpha * dq_raw

        self._prev_q = q.copy()
        self._prev_t = float(self._prev_t + dt)
        return self._dq_filt.copy()


def _zero_twist() -> Twist:
    msg = Twist()
    return msg


class IsaacPolicyPlayer(Node):
    def __init__(self) -> None:
        super().__init__("isaac_policy_player")

        # ---- Parameters ----
        self.declare_parameter("policy_name", "kuroko_walk")
        self.declare_parameter("policy_dir", "")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("model_path", "")

        self.declare_parameter("env_yaml_path", "")
        self.declare_parameter("controller_yaml_path", "")

        self.declare_parameter("publish_hz", 200.0)
        self.declare_parameter("use_inference_thread", True)

        self.declare_parameter("startup_ramp_sec", 2.0)
        self.declare_parameter("action_clip_abs", 1.0)
        self.declare_parameter("target_rate_limit", 6.0)
        self.declare_parameter("dq_cutoff_hz", 30.0)

        self.declare_parameter("cmd_vel_timeout_sec", 1.0)
        self.declare_parameter("joint_velocity_source", "auto")

        self.declare_parameter("startup_move_to_default", False)
        self.declare_parameter("startup_move_duration_sec", 1.0)
        self.declare_parameter("init_last_action_from_joint_states", True)

        self.declare_parameter("startup_print", True)
        self.declare_parameter("debug_print", True)
        self.declare_parameter("debug_print_every", 50)
        self.declare_parameter("debug_print_joint_values", True)

        pkg_share = Path(get_package_share_directory("kuroko_isaac_player"))
        desc_share = Path(get_package_share_directory("kuroko_description"))

        policy_name = self.get_parameter("policy_name").value
        default_policy_dir = pkg_share / "trained_data" / policy_name

        policy_dir_param = self.get_parameter("policy_dir").value
        policy_dir = Path(policy_dir_param) if policy_dir_param else default_policy_dir

        model_path_param = self.get_parameter("model_path").value
        policy_path_param = self.get_parameter("policy_path").value

        explicit = None
        if policy_path_param:
            explicit = policy_path_param
        elif model_path_param:
            explicit = model_path_param

        model_path = Path(
            resolve_policy_path(
                str(policy_dir),
                policy_path=explicit,
                prefer_exported=True,
                logger=self.get_logger(),
            )
        )

        env_yaml_param = self.get_parameter("env_yaml_path").value
        env_yaml_path = Path(env_yaml_param) if env_yaml_param else (policy_dir / "params" / "env.yaml")

        controller_yaml_param = self.get_parameter("controller_yaml_path").value
        default_controller_yaml = desc_share / "config" / "ros2_control" / "joint_group_position_controller.yaml"
        controller_yaml_path = Path(controller_yaml_param) if controller_yaml_param else default_controller_yaml

        self.get_logger().info(f"Policy directory: {policy_dir}")
        self.get_logger().info(f"model_path: {model_path}")
        self.get_logger().info(f"env_yaml_path: {env_yaml_path}")
        self.get_logger().info(f"controller_yaml_path: {controller_yaml_path}")

        # ---- Load env spec ----
        env_spec = load_env_spec(str(env_yaml_path))
        self.policy_joint_names = env_spec.policy_joint_names
        self.obs_terms = env_spec.observation_terms_in_order
        self.default_joint_pos = np.array(env_spec.default_joint_pos, dtype=np.float32)

        ajp = env_spec.raw.get("actions", {}).get("joint_pos", {})
        self.action_scale = float(ajp.get("scale", 1.0))
        self.action_offset = float(ajp.get("offset", 0.0))
        self.use_default_offset = bool(ajp.get("use_default_offset", True))
        self.action_clip = ajp.get("clip", None)

        self.get_logger().info(
            f"Action config: scale={self.action_scale}, offset={self.action_offset}, "
            f"use_default_offset={self.use_default_offset}, clip={self.action_clip}"
        )
        self.get_logger().info(f"Observation terms order: {self.obs_terms}")

        # ---- Load policy ----
        self.policy = load_policy_callable(str(model_path), logger=self.get_logger())

        # ---- Observation builder ----
        self.obs_builder = ObservationBuilder(
            policy_joint_names=self.policy_joint_names,
            terms_in_order=self.obs_terms,
            default_joint_pos=self.default_joint_pos.tolist(),
        )

        self.get_logger().info(f"Observation dim: {self.obs_builder.obs_dim}")

        # ---- Controller mapping ----
        self.controller_joint_names = _load_controller_joint_names(controller_yaml_path)
        self.policy_to_controller_index = self._build_policy_to_controller_index()

        # ---- ROS I/O ----
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Imu, "/kuroko/sensors/imu/data", self._on_imu, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)

        self._latest_cmd = _zero_twist()
        self._latest_cmd_time: Optional[float] = None
        self._latest_imu = Imu()
        self._got_joint = False
        self._got_imu = False

        self._joint_state_name: Optional[list[str]] = None
        self._joint_state_pos: Optional[np.ndarray] = None
        self._joint_state_vel: Optional[np.ndarray] = None

        self._dq_est = _DQEstimator(len(self.policy_joint_names), self.get_parameter("dq_cutoff_hz").value)

        self._latest_obs: Optional[np.ndarray] = None
        self._latest_action: Optional[np.ndarray] = None
        self._lock = threading.Lock()

        self._use_inference_thread = self.get_parameter("use_inference_thread").value
        if self._use_inference_thread:
            self._infer_thread = threading.Thread(target=self._inference_loop, daemon=True)
            self._infer_thread.start()

        hz = self.get_parameter("publish_hz").value
        self.get_logger().info(f"Publishing commands at {hz} Hz")
        self.create_timer(1.0 / hz, self._on_timer)

    # ---------------- mapping ----------------
    def _build_policy_to_controller_index(self) -> list[int]:
        name_to_idx = {jn: i for i, jn in enumerate(self.controller_joint_names)}
        idx: list[int] = []
        for jn in self.policy_joint_names:
            idx.append(name_to_idx.get(jn, -1))
        return idx

    # ---------------- callbacks ----------------
    def _on_cmd(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._latest_cmd_time = time.time()

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg
        self._got_imu = True

    def _on_joint_states(self, msg: JointState) -> None:
        self._got_joint = True
        self._joint_state_name = list(msg.name)
        self._joint_state_pos = np.array(msg.position, dtype=np.float32)
        self._joint_state_vel = np.array(msg.velocity, dtype=np.float32) if msg.velocity else None

    # ---------------- timer ----------------
    def _on_timer(self) -> None:
        if not (self._got_joint and self._got_imu):
            return

        obs = self._build_observation()
        with self._lock:
            self._latest_obs = obs

        if not self._use_inference_thread:
            self._run_inference_once()

        with self._lock:
            action = self._latest_action
        if action is None:
            return

        self._publish_action(action)

    # ---------------- observation ----------------
    def _build_observation(self) -> np.ndarray:
        name_to_i = {n: i for i, n in enumerate(self._joint_state_name)}
        q = np.zeros(len(self.policy_joint_names), dtype=np.float32)
        dq = np.zeros_like(q)

        for pi, jn in enumerate(self.policy_joint_names):
            si = name_to_i.get(jn)
            if si is not None:
                q[pi] = self._joint_state_pos[si]

        dq = self._dq_est.update(q, time.time())

        cmd = self._latest_cmd
        imu = self._latest_imu
        return self.obs_builder.build(cmd, q, dq, imu)

    # ---------------- inference ----------------
    def _run_inference_once(self) -> None:
        with self._lock:
            obs = None if self._latest_obs is None else self._latest_obs.copy()
        if obs is None:
            return

        action = self.policy(obs)
        action = np.asarray(action, dtype=np.float32).reshape(-1)

        self.obs_builder.set_last_action(action)
        with self._lock:
            self._latest_action = action

    def _inference_loop(self) -> None:
        while rclpy.ok():
            self._run_inference_once()
            time.sleep(0.001)

    # ---------------- publish ----------------
    def _publish_action(self, action: np.ndarray) -> None:
        q_des = self.action_scale * action + self.action_offset
        if self.use_default_offset:
            q_des = q_des + self.default_joint_pos

        cmd = np.zeros(len(self.controller_joint_names), dtype=np.float64)
        for pi, ci in enumerate(self.policy_to_controller_index):
            if ci >= 0:
                cmd[ci] = float(q_des[pi])

        msg = Float64MultiArray()
        msg.data = cmd.tolist()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacPolicyPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
