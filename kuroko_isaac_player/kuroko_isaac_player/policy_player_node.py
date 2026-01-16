# kuroko_isaac_player/policy_player_node.py
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
from .policy_loader import load_policy_callable


def _find_latest_pt(policy_dir: Path) -> Path:
    pts = sorted(policy_dir.glob("*.pt"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not pts:
        raise FileNotFoundError(f"No .pt found in: {policy_dir}")
    return pts[0]


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

        # 1st-order low-pass: alpha = dt / (dt + rc), rc = 1/(2*pi*fc)
        rc = 1.0 / (2.0 * math.pi * max(self._cutoff_hz, 1e-3))
        alpha = dt / (dt + rc)
        self._dq_filt = (1.0 - alpha) * self._dq_filt + alpha * dq_raw

        self._prev_q = q.copy()
        self._prev_t = float(t)
        return self._dq_filt.copy()


class IsaacPolicyPlayer(Node):
    def __init__(self) -> None:
        super().__init__("isaac_policy_player")

        # ---- Parameters ----
        self.declare_parameter("model_path", "")
        self.declare_parameter("env_yaml_path", "")
        self.declare_parameter("controller_yaml_path", "")

        self.declare_parameter("policy_name", "kuroko_walk")
        self.declare_parameter("policy_dir", "")
        self.declare_parameter("policy_path", "")

        self.declare_parameter("publish_hz", 200.0)
        self.declare_parameter("use_inference_thread", True)

        # Logging controls
        self.declare_parameter("startup_print", True)
        self.declare_parameter("debug_print", True)
        self.declare_parameter("debug_print_every", 50)
        self.declare_parameter("debug_print_joint_values", True)

        # Safety / estimation
        self.declare_parameter("startup_ramp_sec", 2.0)
        self.declare_parameter("action_clip_abs", 1.0)
        self.declare_parameter("target_rate_limit", 6.0)
        self.declare_parameter("dq_cutoff_hz", 30.0)
        self.declare_parameter("require_all_topics", True)

        # NEW: choose velocity source
        #   "auto"     -> use /joint_states.velocity if valid else estimate
        #   "topic"    -> always use /joint_states.velocity (if missing -> zeros + warn once)
        #   "estimate" -> always estimate from dq (ignore topic velocity)
        self.declare_parameter("joint_velocity_source", "auto")

        # NEW: startup move (current pose -> default pose) before enabling policy
        self.declare_parameter("startup_move_to_default", False)
        self.declare_parameter("startup_move_duration_sec", 1.0)

        pkg_share = Path(get_package_share_directory("kuroko_isaac_player"))
        desc_share = Path(get_package_share_directory("kuroko_description"))

        policy_name = str(self.get_parameter("policy_name").value)
        default_policy_dir = pkg_share / "trained_data" / policy_name

        policy_dir_param = str(self.get_parameter("policy_dir").value).strip()
        policy_dir = Path(policy_dir_param) if policy_dir_param else default_policy_dir

        model_path_param = str(self.get_parameter("model_path").value).strip()
        policy_path_param = str(self.get_parameter("policy_path").value).strip()

        if model_path_param:
            model_path = Path(model_path_param)
        elif policy_path_param:
            model_path = Path(policy_path_param)
        else:
            model_path = _find_latest_pt(policy_dir)

        env_yaml_param = str(self.get_parameter("env_yaml_path").value).strip()
        env_yaml_path = Path(env_yaml_param) if env_yaml_param else (policy_dir / "params" / "env.yaml")

        controller_yaml_param = str(self.get_parameter("controller_yaml_path").value).strip()
        default_controller_yaml = desc_share / "config" / "gz_position_controller.yaml"
        controller_yaml_path = Path(controller_yaml_param) if controller_yaml_param else default_controller_yaml

        self.get_logger().info(f"Policy directory: {policy_dir}")
        self.get_logger().info(f"model_path: {model_path}")
        self.get_logger().info(f"env_yaml_path: {env_yaml_path}")
        self.get_logger().info(f"controller_yaml_path: {controller_yaml_path}")

        env_spec = load_env_spec(str(env_yaml_path))
        self.policy_joint_names = env_spec.policy_joint_names
        self.obs_terms = env_spec.observation_terms_in_order
        self.default_joint_pos = np.array(env_spec.default_joint_pos, dtype=np.float32)

        # ---- action config from env.yaml ----
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

        # Load policy
        self.policy = load_policy_callable(str(model_path), logger=self.get_logger())

        # Obs builder
        self.obs_builder = ObservationBuilder(
            policy_joint_names=self.policy_joint_names,
            terms_in_order=self.obs_terms,
            default_joint_pos=self.default_joint_pos.tolist(),
        )
        self.get_logger().info(f"Observation dim: {self.obs_builder.obs_dim}")

        # Controller order + mapping
        self.controller_joint_names = _load_controller_joint_names(controller_yaml_path)
        self.policy_to_controller_index = self._build_policy_to_controller_index()

        # Startup prints (guarded)
        self._startup_print = bool(self.get_parameter("startup_print").value)
        if self._startup_print:
            self.obs_builder.debug_dump_observation_layout(self.get_logger())
            self._debug_print_joint_orders_and_mapping()
            self._debug_print_default_pose()

        # Topic caches
        self._latest_cmd = Twist()
        self._latest_imu = Imu()

        self._joint_state_name: Optional[list[str]] = None
        self._joint_state_pos: Optional[np.ndarray] = None
        self._joint_state_vel: Optional[np.ndarray] = None

        self._got_cmd = False
        self._got_imu = False
        self._got_joint = False

        # dq estimator + publish limiting
        self._dq_est = _DQEstimator(
            n=len(self.policy_joint_names),
            cutoff_hz=float(self.get_parameter("dq_cutoff_hz").value),
        )
        self._prev_q_des_policy: Optional[np.ndarray] = None
        self._prev_pub_t: Optional[float] = None

        # Debug controls
        self._debug_print = bool(self.get_parameter("debug_print").value)
        self._debug_every = int(self.get_parameter("debug_print_every").value)
        self._debug_joint_values = bool(self.get_parameter("debug_print_joint_values").value)
        self._tick = 0

        # Velocity-source warning (only warn once for topic-missing)
        self._warned_vel_missing = False

        # Startup ramp
        self._start_wall = time.time()
        self._ramp_sec = float(self.get_parameter("startup_ramp_sec").value)

        self._printed_joint_states_order = False
        self._printed_first_infer = False

        # NEW: startup move state
        self._startup_move_enabled = bool(self.get_parameter("startup_move_to_default").value)
        self._startup_move_duration = float(self.get_parameter("startup_move_duration_sec").value)
        self._startup_move_active = False
        self._startup_move_t0: Optional[float] = None
        self._startup_move_q0_ctrl: Optional[np.ndarray] = None
        self._startup_move_qt_ctrl: Optional[np.ndarray] = None
        self._startup_move_logged = False

        # ROS interfaces
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Imu, "/kuroko/sensors/imu/data", self._on_imu, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)

        # Inference threading
        self._use_inference_thread = bool(self.get_parameter("use_inference_thread").value)
        self._latest_obs: Optional[np.ndarray] = None
        self._latest_action: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._stop = False

        if self._use_inference_thread:
            self.get_logger().info("Inference thread enabled")
            self._infer_thread = threading.Thread(target=self._inference_loop, daemon=True)
            self._infer_thread.start()

        hz = float(self.get_parameter("publish_hz").value)
        self.get_logger().info(f"Publishing commands at {hz} Hz")
        self.create_timer(1.0 / hz, self._on_timer)

    # -------------------------
    # Debug prints
    # -------------------------
    def _debug_print_joint_orders_and_mapping(self) -> None:
        self.get_logger().info("==== Policy Joint Order (env.yaml) ====")
        for i, jn in enumerate(self.policy_joint_names):
            self.get_logger().info(f"[{i:2d}] {jn}")

        self.get_logger().info("==== Controller Command Order (/joint_group_position_controller/commands) ====")
        for i, jn in enumerate(self.controller_joint_names):
            self.get_logger().info(f"[{i:2d}] {jn}")

        self.get_logger().info("==== Action → Controller Mapping ====")
        for pi, jn in enumerate(self.policy_joint_names):
            ci = self.policy_to_controller_index[pi]
            self.get_logger().info(f"policy[{pi:2d}] -> joint '{jn}' -> controller[{ci:2d}]")

    def _debug_print_default_pose(self) -> None:
        self.get_logger().info("==== Default Joint Pose (from env.yaml scene.robot.init_state.joint_pos) ====")
        self.get_logger().info("---- Policy order ----")
        for i, jn in enumerate(self.policy_joint_names):
            self.get_logger().info(f"[{i:2d}] {jn:20s} = {float(self.default_joint_pos[i]): .6f}")

        self.get_logger().info("---- Controller order ----")
        dmap = {jn: float(self.default_joint_pos[i]) for i, jn in enumerate(self.policy_joint_names)}
        for ci, jn in enumerate(self.controller_joint_names):
            v = dmap.get(jn, 0.0)
            self.get_logger().info(f"[{ci:2d}] {jn:20s} = {v: .6f}")

        self.get_logger().info("==============================================================")

    # -------------------------
    # Mapping
    # -------------------------
    def _build_policy_to_controller_index(self) -> list[int]:
        name_to_idx = {jn: i for i, jn in enumerate(self.controller_joint_names)}
        idx: list[int] = []
        missing: list[str] = []
        for jn in self.policy_joint_names:
            if jn not in name_to_idx:
                missing.append(jn)
                idx.append(-1)
            else:
                idx.append(name_to_idx[jn])
        if missing:
            self.get_logger().error(f"Missing joints in controller list: {missing}")
        return idx

    # -------------------------
    # Startup move helpers
    # -------------------------
    def _compute_controller_cmd_from_joint_states(self, msg: JointState) -> np.ndarray:
        """
        Build controller-order array from joint_states positions.
        Missing joints -> keep default pose for that joint (safer than 0).
        """
        name_to_i = {jn: i for i, jn in enumerate(msg.name)}
        # default pose in controller order
        dmap = {jn: float(self.default_joint_pos[i]) for i, jn in enumerate(self.policy_joint_names)}
        out = np.zeros((len(self.controller_joint_names),), dtype=np.float64)
        for ci, jn in enumerate(self.controller_joint_names):
            if jn in name_to_i and msg.position and len(msg.position) == len(msg.name):
                out[ci] = float(msg.position[name_to_i[jn]])
            else:
                out[ci] = float(dmap.get(jn, 0.0))
        return out

    def _compute_controller_cmd_from_default_pose(self) -> np.ndarray:
        dmap = {jn: float(self.default_joint_pos[i]) for i, jn in enumerate(self.policy_joint_names)}
        out = np.zeros((len(self.controller_joint_names),), dtype=np.float64)
        for ci, jn in enumerate(self.controller_joint_names):
            out[ci] = float(dmap.get(jn, 0.0))
        return out

    def _publish_controller_cmd(self, cmd_ctrl: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = np.asarray(cmd_ctrl, dtype=np.float64).reshape(-1).tolist()
        self.pub.publish(msg)

    # -------------------------
    # Callbacks
    # -------------------------
    def _on_cmd(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._got_cmd = True

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg
        self._got_imu = True

    def _on_joint_states(self, msg: JointState) -> None:
        self._got_joint = True

        # Startup prints for joint_states order can be noisy too: guard with startup_print.
        if self._startup_print and not self._printed_joint_states_order:
            self._printed_joint_states_order = True
            self.get_logger().info("==== /joint_states Order ====")
            for i, jn in enumerate(msg.name):
                self.get_logger().info(f"[{i:2d}] {jn}")

            if msg.velocity is None or len(msg.velocity) == 0:
                self.get_logger().warn("/joint_states.velocity is empty -> dq may be estimated depending on joint_velocity_source.")
            elif len(msg.velocity) != len(msg.name):
                self.get_logger().warn(
                    f"/joint_states.velocity length mismatch: {len(msg.velocity)} != {len(msg.name)} -> dq may be estimated depending on joint_velocity_source."
                )
            else:
                self.get_logger().info("/joint_states.velocity is present.")

        self._joint_state_name = list(msg.name)
        self._joint_state_pos = np.array(msg.position, dtype=np.float32) if msg.position else None
        self._joint_state_vel = np.array(msg.velocity, dtype=np.float32) if msg.velocity else None

        # NEW: initialize startup move (once) when first joint_states arrives
        if self._startup_move_enabled and (not self._startup_move_active) and (self._startup_move_t0 is None):
            self._startup_move_t0 = time.time()
            self._startup_move_q0_ctrl = self._compute_controller_cmd_from_joint_states(msg)
            self._startup_move_qt_ctrl = self._compute_controller_cmd_from_default_pose()
            self._startup_move_active = True

            # Reset buffers to avoid jumps after the move
            self._dq_est.reset()
            self.obs_builder.reset_last_action()
            with self._lock:
                self._latest_action = None  # wait for fresh inference after move
                self._latest_obs = None

            # Restart ramp timing after move completes
            self._start_wall = time.time()

            if self._startup_print and (not self._startup_move_logged):
                self._startup_move_logged = True
                self.get_logger().info(
                    f"Startup move enabled: interpolating current pose -> default pose over "
                    f"{self._startup_move_duration:.3f} s"
                )

    # -------------------------
    # JointState -> policy order
    # -------------------------
    def _collect_policy_order_q_dq(self) -> Tuple[np.ndarray, np.ndarray, bool]:
        if self._joint_state_name is None or self._joint_state_pos is None:
            raise RuntimeError("No /joint_states received yet")

        name_to_i = {jn: i for i, jn in enumerate(self._joint_state_name)}
        n = len(self.policy_joint_names)

        q = np.zeros((n,), dtype=np.float32)
        dq_from_msg = np.zeros((n,), dtype=np.float32)
        vel_ok = False

        for pi, jn in enumerate(self.policy_joint_names):
            si = name_to_i.get(jn, None)
            if si is None:
                continue
            q[pi] = float(self._joint_state_pos[si])

        if (
            self._joint_state_vel is not None
            and len(self._joint_state_name) == len(self._joint_state_vel)
            and len(self._joint_state_vel) == len(self._joint_state_pos)
        ):
            vel_ok = True
            for pi, jn in enumerate(self.policy_joint_names):
                si = name_to_i.get(jn, None)
                if si is None:
                    continue
                dq_from_msg[pi] = float(self._joint_state_vel[si])

        return q, dq_from_msg, vel_ok

    # -------------------------
    # Timer tick
    # -------------------------
    def _on_timer(self) -> None:
        self._tick += 1

        # NEW: if startup move is active, publish interpolated command and skip policy output
        if self._startup_move_active:
            if self._startup_move_q0_ctrl is None or self._startup_move_qt_ctrl is None or self._startup_move_t0 is None:
                return

            dur = max(float(self._startup_move_duration), 1e-3)
            now = time.time()
            alpha = float(np.clip((now - self._startup_move_t0) / dur, 0.0, 1.0))
            cmd_ctrl = (1.0 - alpha) * self._startup_move_q0_ctrl + alpha * self._startup_move_qt_ctrl
            self._publish_controller_cmd(cmd_ctrl)

            if alpha >= 1.0:
                # Move completed: enable policy starting next tick
                self._startup_move_active = False
                self._start_wall = time.time()
                self._prev_q_des_policy = None
                self._prev_pub_t = None
                self._dq_est.reset()
                self.obs_builder.reset_last_action()
                if self._startup_print:
                    self.get_logger().info("Startup move completed. Policy control enabled.")
            return

        require_all = bool(self.get_parameter("require_all_topics").value)
        if require_all and not (self._got_cmd and self._got_imu and self._got_joint):
            return

        cmd = self._latest_cmd
        imu = self._latest_imu

        q, dq_msg, vel_ok = self._collect_policy_order_q_dq()

        now = time.time()
        src = str(self.get_parameter("joint_velocity_source").value).strip().lower()
        if src not in ("auto", "topic", "estimate"):
            self.get_logger().warn(f"Invalid joint_velocity_source='{src}', fallback to 'auto'")
            src = "auto"

        if src == "estimate":
            dq = self._dq_est.update(q, now)
            vel_used = False
        elif src == "topic":
            if vel_ok:
                dq = dq_msg
                vel_used = True
            else:
                dq = np.zeros_like(q, dtype=np.float32)
                vel_used = False
                if not self._warned_vel_missing:
                    self._warned_vel_missing = True
                    self.get_logger().warn(
                        "joint_velocity_source='topic' but /joint_states.velocity is missing/invalid. Using zeros."
                    )
        else:  # auto
            if vel_ok:
                dq = dq_msg
                vel_used = True
            else:
                dq = self._dq_est.update(q, now)
                vel_used = False

        obs = self.obs_builder.build(cmd, q, dq, imu)

        with self._lock:
            self._latest_obs = obs

        if not self._use_inference_thread:
            self._run_inference_once()

        with self._lock:
            action = None if self._latest_action is None else self._latest_action.copy()

        if action is None:
            return

        self._publish_action_as_controller_command(action, q, dq, obs, vel_used)

    # -------------------------
    # Inference
    # -------------------------
    def _run_inference_once(self) -> None:
        with self._lock:
            obs = None if self._latest_obs is None else self._latest_obs.copy()

        if obs is None:
            return

        try:
            action = self.policy(obs)
            action = np.asarray(action, dtype=np.float32)
            if action.ndim == 2:
                action = action.reshape(-1)

            self.obs_builder.set_last_action(action)

            with self._lock:
                self._latest_action = action

            if self._startup_print and (not self._printed_first_infer):
                self._printed_first_infer = True
                self.get_logger().info("==== First inference completed ====")

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Inference error: {exc}")

    def _inference_loop(self) -> None:
        while not self._stop and rclpy.ok():
            self._run_inference_once()

    # -------------------------
    # Action -> target + publish
    # -------------------------
    def _startup_gain(self) -> float:
        if self._ramp_sec <= 0.0:
            return 1.0
        t = time.time() - self._start_wall
        return float(np.clip(t / self._ramp_sec, 0.0, 1.0))

    def _action_to_target_q(self, action_policy_order: np.ndarray, gain: float) -> np.ndarray:
        a = np.asarray(action_policy_order, dtype=np.float32).reshape(-1)

        clip_abs = float(self.get_parameter("action_clip_abs").value)
        if clip_abs > 0:
            a = np.clip(a, -clip_abs, clip_abs)

        a = gain * a

        tgt = (self.action_scale * a) + self.action_offset
        if self.use_default_offset:
            tgt = tgt + self.default_joint_pos

        if isinstance(self.action_clip, (list, tuple)) and len(self.action_clip) == 2:
            lo, hi = float(self.action_clip[0]), float(self.action_clip[1])
            tgt = np.clip(tgt, lo, hi)

        return tgt.astype(np.float32)

    def _rate_limit_target(self, q_des: np.ndarray, now: float) -> np.ndarray:
        q_des = np.asarray(q_des, dtype=np.float32).reshape(-1)

        if self._prev_q_des_policy is None or self._prev_pub_t is None:
            self._prev_q_des_policy = q_des.copy()
            self._prev_pub_t = float(now)
            return q_des

        dt = float(now - self._prev_pub_t)
        if dt <= 1e-6:
            return self._prev_q_des_policy.copy()

        rate = float(self.get_parameter("target_rate_limit").value)
        if rate <= 0:
            self._prev_q_des_policy = q_des.copy()
            self._prev_pub_t = float(now)
            return q_des

        max_delta = rate * dt
        dq = q_des - self._prev_q_des_policy
        dq = np.clip(dq, -max_delta, max_delta)
        out = self._prev_q_des_policy + dq

        self._prev_q_des_policy = out.copy()
        self._prev_pub_t = float(now)
        return out

    def _debug_dump_runtime(
        self,
        gain: float,
        q: np.ndarray,
        dq: np.ndarray,
        obs: np.ndarray,
        action: np.ndarray,
        q_des_policy: np.ndarray,
        cmd_ctrl: np.ndarray,
        vel_used: bool,
    ) -> None:
        if not self._debug_print:
            return
        if self._debug_every <= 0:
            return
        if (self._tick % self._debug_every) != 0:
            return

        self.get_logger().info("==== Runtime Debug Dump ====")
        self.get_logger().info(f"tick={self._tick} startup_gain={gain:.3f} vel_used={vel_used}")
        self.get_logger().info(
            f"obs: dim={obs.size} min={float(obs.min()):.4f} max={float(obs.max()):.4f} mean={float(obs.mean()):.4f}"
        )
        self.get_logger().info(
            f"action: dim={action.size} min={float(action.min()):.4f} max={float(action.max()):.4f} mean={float(action.mean()):.4f}"
        )
        self.get_logger().info(
            f"q_des_policy: min={float(q_des_policy.min()):.4f} max={float(q_des_policy.max()):.4f}"
        )

        if self._debug_joint_values:
            self.get_logger().info("---- Policy joints: q / q_rel / dq / action / q_des ----")
            for i, jn in enumerate(self.policy_joint_names):
                q_rel = float(q[i] - self.default_joint_pos[i])
                self.get_logger().info(
                    f"[{i:2d}] {jn:20s} q={float(q[i]): .4f} q_rel={q_rel: .4f} dq={float(dq[i]): .4f} "
                    f"a={float(action[i]): .4f} q_des={float(q_des_policy[i]): .4f}"
                )

            self.get_logger().info("---- Controller cmd (published order) ----")
            for ci, jn in enumerate(self.controller_joint_names):
                self.get_logger().info(f"[{ci:2d}] {jn:20s} cmd={float(cmd_ctrl[ci]): .4f}")

        self.get_logger().info("=================================")

    def _publish_action_as_controller_command(
        self,
        action_policy_order: np.ndarray,
        q: np.ndarray,
        dq: np.ndarray,
        obs: np.ndarray,
        vel_used: bool,
    ) -> None:
        now = time.time()
        gain = self._startup_gain()

        q_des_policy = self._action_to_target_q(action_policy_order, gain=gain)
        q_des_policy = self._rate_limit_target(q_des_policy, now)

        cmd_ctrl = np.zeros((len(self.controller_joint_names),), dtype=np.float64)
        for pi, ci in enumerate(self.policy_to_controller_index):
            if ci < 0:
                continue
            cmd_ctrl[ci] = float(q_des_policy[pi])

        self._debug_dump_runtime(
            gain=gain,
            q=q,
            dq=dq,
            obs=obs,
            action=np.asarray(action_policy_order, dtype=np.float32).reshape(-1),
            q_des_policy=q_des_policy,
            cmd_ctrl=cmd_ctrl,
            vel_used=vel_used,
        )

        self._publish_controller_cmd(cmd_ctrl)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacPolicyPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
