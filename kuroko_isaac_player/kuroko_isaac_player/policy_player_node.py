# kuroko_isaac_player/policy_player_node.py

from __future__ import annotations

import threading
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
    """
    Parse ros2_control yaml (gz_position_controller.yaml) to get controller joint order.

    Supports BOTH common layouts:

    A) controller_manager:
         ros__parameters:
           joint_group_position_controller:
             ros__parameters:
               joints: [...]

    B) joint_group_position_controller:
         ros__parameters:
           joints: [...]
    """
    import yaml

    with open(controller_yaml_path, "r") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise TypeError(f"controller yaml root must be dict: {controller_yaml_path}")

    # ---- Layout B (top-level) ----
    try:
        jgp = data.get("joint_group_position_controller", None)
        if isinstance(jgp, dict):
            ros_params = jgp.get("ros__parameters", None)
            if isinstance(ros_params, dict):
                joints = ros_params.get("joints", None)
                if isinstance(joints, list) and all(isinstance(x, str) for x in joints):
                    return joints
    except Exception:
        pass

    # ---- Layout A (under controller_manager) ----
    try:
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
    except Exception:
        pass

    # ---- Fallback: recursive search for the best "joints: [str...]" ----
    best: list[str] = []

    def walk(obj: Any) -> None:
        nonlocal best
        if isinstance(obj, dict):
            # If this dict directly contains a joints list, consider it
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


class IsaacPolicyPlayer(Node):
    def __init__(self) -> None:
        super().__init__("isaac_policy_player")

        # Explicit paths
        self.declare_parameter("model_path", "")
        self.declare_parameter("env_yaml_path", "")
        self.declare_parameter("controller_yaml_path", "")

        # Policy directory style (optional)
        self.declare_parameter("policy_name", "kuroko_walk")
        self.declare_parameter("policy_dir", "")
        self.declare_parameter("policy_path", "")  # alias of model_path

        # Runtime
        self.declare_parameter("publish_hz", 200.0)
        self.declare_parameter("use_inference_thread", True)

        # Resolve asset paths robustly
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

        if not model_path.exists():
            raise FileNotFoundError(f"model_path not found: {model_path}")
        if not env_yaml_path.exists():
            raise FileNotFoundError(f"env_yaml_path not found: {env_yaml_path}")
        if not controller_yaml_path.exists():
            raise FileNotFoundError(f"controller_yaml_path not found: {controller_yaml_path}")

        # Load env spec / policy
        env_spec = load_env_spec(str(env_yaml_path))
        self.policy_joint_names = env_spec.policy_joint_names
        self.obs_terms = env_spec.observation_terms_in_order
        self.default_joint_pos = env_spec.default_joint_pos

        self.get_logger().info(f"Observation terms order: {self.obs_terms}")

        self.policy = load_policy_callable(str(model_path), logger=self.get_logger())

        # Observation builder
        self.obs_builder = ObservationBuilder(
            policy_joint_names=self.policy_joint_names,
            terms_in_order=self.obs_terms,
            default_joint_pos=self.default_joint_pos,
        )

        self.get_logger().info(f"Observation dim: {self.obs_builder.obs_dim}")
        self.obs_builder.debug_dump_observation_layout(self.get_logger())

        # Controller joint order + mapping
        self.controller_joint_names = _load_controller_joint_names(controller_yaml_path)
        self.policy_to_controller_index = self._build_policy_to_controller_index()

        self._debug_print_joint_orders_and_mapping()

        # Sub/Pub
        self._latest_cmd = Twist()
        self._latest_imu = Imu()

        self._joint_state_name: Optional[list[str]] = None
        self._joint_state_pos: Optional[np.ndarray] = None
        self._joint_state_vel: Optional[np.ndarray] = None

        self._printed_joint_states_order = False
        self._printed_first_infer = False

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

    # Debug prints
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

    # Mapping
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

    # Callbacks
    def _on_cmd(self, msg: Twist) -> None:
        self._latest_cmd = msg

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg

    def _on_joint_states(self, msg: JointState) -> None:
        if not self._printed_joint_states_order:
            self._printed_joint_states_order = True
            self.get_logger().info("==== /joint_states Order ====")
            for i, jn in enumerate(msg.name):
                self.get_logger().info(f"[{i:2d}] {jn}")

        self._joint_state_name = list(msg.name)
        self._joint_state_pos = np.array(msg.position, dtype=np.float32) if msg.position else None
        self._joint_state_vel = np.array(msg.velocity, dtype=np.float32) if msg.velocity else None

    # JointState -> policy order
    def _collect_policy_order_q_dq(self) -> Tuple[np.ndarray, np.ndarray]:
        if self._joint_state_name is None or self._joint_state_pos is None:
            raise RuntimeError("No /joint_states received yet")

        name_to_i = {jn: i for i, jn in enumerate(self._joint_state_name)}
        n = len(self.policy_joint_names)

        q = np.zeros((n,), dtype=np.float32)
        dq = np.zeros((n,), dtype=np.float32)

        for pi, jn in enumerate(self.policy_joint_names):
            si = name_to_i.get(jn, None)
            if si is None:
                continue
            q[pi] = float(self._joint_state_pos[si])

            if self._joint_state_vel is not None and len(self._joint_state_vel) == len(self._joint_state_name):
                dq[pi] = float(self._joint_state_vel[si])

        return q, dq

    # Timer tick
    def _on_timer(self) -> None:
        cmd = self._latest_cmd
        imu = self._latest_imu

        q, dq = self._collect_policy_order_q_dq()
        obs = self.obs_builder.build(cmd, q, dq, imu)

        with self._lock:
            self._latest_obs = obs

        if not self._use_inference_thread:
            self._run_inference_once()

        with self._lock:
            action = None if self._latest_action is None else self._latest_action.copy()

        if action is None:
            return

        self._publish_action_as_controller_command(action)

    # Inference
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

            if not self._printed_first_infer:
                self._printed_first_infer = True
                self.get_logger().info("==== First inference completed ====")

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Inference error: {exc}")

    def _inference_loop(self) -> None:
        while not self._stop and rclpy.ok():
            self._run_inference_once()

    # Publish
    def _publish_action_as_controller_command(self, action_policy_order: np.ndarray) -> None:
        cmd = np.zeros((len(self.controller_joint_names),), dtype=np.float64)
        for pi, ci in enumerate(self.policy_to_controller_index):
            if ci < 0:
                continue
            cmd[ci] = float(action_policy_order[pi])

        msg = Float64MultiArray()
        msg.data = cmd.tolist()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacPolicyPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
