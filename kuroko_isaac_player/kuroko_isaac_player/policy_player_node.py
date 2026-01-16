#!/usr/bin/env python3

from __future__ import annotations

import os
import threading
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray

from ament_index_python.packages import get_package_share_directory

from .env_spec import load_env_spec
from .joint_mapping import load_controller_joint_names, build_index_map_by_name
from .observation_builder import ObservationBuilder
from .policy_loader import load_policy_callable
from .filters import LowPassFilter


@dataclass
class JointCalib:
    sign: float = 1.0
    offset: float = 0.0


class IsaacPolicyPlayer(Node):
    def __init__(self):
        super().__init__("isaac_policy_player")

        # Topics (command topic is fixed to /.../commands by your requirement)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/kuroko/sensors/imu/data")
        self.declare_parameter("command_topic", "/joint_group_position_controller/commands")

        # Rates
        self.declare_parameter("publish_rate_hz", 200.0)

        # Velocity estimation / filtering
        self.declare_parameter("dq_estimation_enabled", True)
        self.declare_parameter("dq_lowpass_alpha", 0.2)  # 0.0<alpha<=1.0
        self.declare_parameter("joint_state_stale_sec", 0.05)  # if vel older than this, estimate

        # Threading
        self.declare_parameter("inference_thread", True)

        # Files (inside installed share)
        self.declare_parameter("trained_data_dir", "trained_data/kuroko_walk")
        self.declare_parameter("model_file", "model_13400.pt")
        self.declare_parameter("env_yaml_file", "params/env.yaml")

        self.declare_parameter("kuroko_description_package", "kuroko_description")
        self.declare_parameter("controller_yaml_relpath", "config/gz_position_controller.yaml")

        # Optional per-joint calibration
        self.declare_parameter("joint_calibration_file", "")

        # -----------------------------
        # Resolve paths
        # -----------------------------
        share_dir = get_package_share_directory("kuroko_isaac_player")
        trained_dir = os.path.join(share_dir, self.get_parameter("trained_data_dir").value)
        self.model_path = os.path.join(trained_dir, self.get_parameter("model_file").value)
        self.env_yaml_path = os.path.join(trained_dir, self.get_parameter("env_yaml_file").value)

        desc_pkg = self.get_parameter("kuroko_description_package").value
        desc_share = get_package_share_directory(desc_pkg)
        self.controller_yaml_path = os.path.join(desc_share, self.get_parameter("controller_yaml_relpath").value)

        self.get_logger().info(f"model_path: {self.model_path}")
        self.get_logger().info(f"env_yaml_path: {self.env_yaml_path}")
        self.get_logger().info(f"controller_yaml_path: {self.controller_yaml_path}")

        # -----------------------------
        # Load env spec (FIXED keys)
        # -----------------------------
        env_spec = load_env_spec(self.env_yaml_path)
        self.policy_joint_names = env_spec.policy_joint_names
        self.default_joint_pos = env_spec.default_joint_pos
        self.obs_terms = env_spec.observation_terms_in_order

        # -----------------------------
        # Load controller joint ordering
        # -----------------------------
        self.controller_joint_names = load_controller_joint_names(self.controller_yaml_path)
        self.policy_to_controller = build_index_map_by_name(
            src_names=self.policy_joint_names,
            dst_names=self.controller_joint_names,
            what="policy_to_controller",
        )

        # -----------------------------
        # Joint calibration map
        # -----------------------------
        self.joint_calib: Dict[str, JointCalib] = {}

        calib_file = str(self.get_parameter("joint_calibration_file").value)
        if calib_file:
            # allow package-relative: "config/joint_calibration.yaml"
            if not os.path.isabs(calib_file):
                share_dir = get_package_share_directory("kuroko_isaac_player")
                calib_file = os.path.join(share_dir, calib_file)

            try:
                with open(calib_file, "r") as f:
                    calib_yaml = yaml.safe_load(f) or {}
                # expected format:
                # joint_calibration:
                #   hip_l_pitch: {sign: -1.0, offset: 0.02}
                calib_dict = calib_yaml.get("joint_calibration", calib_yaml)
                if isinstance(calib_dict, dict):
                    for jn, v in calib_dict.items():
                        if not isinstance(v, dict):
                            continue
                        sign = float(v.get("sign", 1.0))
                        offset = float(v.get("offset", 0.0))
                        self.joint_calib[jn] = JointCalib(sign=sign, offset=offset)
                self.get_logger().info(f"Loaded joint calibration: {len(self.joint_calib)} joints from {calib_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to load joint calibration file '{calib_file}': {e}")

        # -----------------------------
        # Policy
        # -----------------------------
        self.policy = load_policy_callable(self.model_path)

        # -----------------------------
        # Observation builder (exact match)
        # -----------------------------
        self.obs_builder = ObservationBuilder(
            policy_joint_names=self.policy_joint_names,
            default_joint_pos=self.default_joint_pos,
            terms_in_order=self.obs_terms,
        )
        self.get_logger().info(f"Observation terms order: {self.obs_terms}")
        self.get_logger().info(f"Observation dim: {self.obs_builder.obs_dim}")

        # -----------------------------
        # State buffers
        # -----------------------------
        self._lock = threading.Lock()
        self._latest_cmd: Optional[Twist] = None
        self._latest_js: Optional[JointState] = None
        self._latest_imu: Optional[Imu] = None
        self._name_to_index: Optional[Dict[str, int]] = None
        self._last_q: Optional[np.ndarray] = None
        self._last_q_time: Optional[float] = None

        # dq filters per joint
        alpha = float(self.get_parameter("dq_lowpass_alpha").value)
        alpha = max(min(alpha, 1.0), 1e-6)
        self._dq_filters = [LowPassFilter(alpha=alpha) for _ in self.policy_joint_names]

        # Inference thread buffers
        self._use_thread = bool(self.get_parameter("inference_thread").value)
        self._new_obs_event = threading.Event()
        self._stop_event = threading.Event()
        self._latest_obs: Optional[np.ndarray] = None
        self._latest_action: np.ndarray = np.zeros(len(self.policy_joint_names), dtype=np.float32)

        if self._use_thread:
            self._worker = threading.Thread(target=self._inference_worker, daemon=True)
            self._worker.start()
            self.get_logger().info("Inference thread enabled")
        else:
            self.get_logger().info("Inference thread disabled (inference in timer callback)")

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.create_subscription(Twist, self.get_parameter("cmd_vel_topic").value, self._on_cmd, 10)
        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value, self._on_js, 50)
        self.create_subscription(Imu, self.get_parameter("imu_topic").value, self._on_imu, 50)
        self._pub = self.create_publisher(Float64MultiArray, self.get_parameter("command_topic").value, 10)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(rate_hz, 1e-6)
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(f"Publishing commands at {rate_hz:.1f} Hz")

    # -----------------------------
    # Subscribers
    # -----------------------------
    def _on_cmd(self, msg: Twist):
        with self._lock:
            self._latest_cmd = msg

    def _on_js(self, msg: JointState):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._latest_js = msg
            if self._name_to_index is None and msg.name:
                self._name_to_index = {n: i for i, n in enumerate(msg.name)}
            # Store timestamp proxy for dq estimation
            self._latest_js_time = now

    def _on_imu(self, msg: Imu):
        with self._lock:
            self._latest_imu = msg

    # -----------------------------
    # Main timer
    # -----------------------------
    def _on_timer(self):
        with self._lock:
            cmd = self._latest_cmd
            js = self._latest_js
            imu = self._latest_imu
            name_to_index = self._name_to_index
            js_time = getattr(self, "_latest_js_time", None)

        if cmd is None or js is None or imu is None or name_to_index is None:
            return

        # Build q (absolute) and dq (measured or estimated) in policy joint order
        q = np.zeros(len(self.policy_joint_names), dtype=np.float32)
        dq_meas = np.full(len(self.policy_joint_names), np.nan, dtype=np.float32)

        for k, jn in enumerate(self.policy_joint_names):
            idx = name_to_index.get(jn, None)
            if idx is None:
                continue
            if idx < len(js.position):
                q[k] = float(js.position[idx])
            if idx < len(js.velocity):
                dq_meas[k] = float(js.velocity[idx])

        dq = self._compute_dq(q, dq_meas, js_time)

        obs = self.obs_builder.build(cmd, q, dq, imu)

        if self._use_thread:
            with self._lock:
                self._latest_obs = obs
            self._new_obs_event.set()
            action = None
        else:
            action = self._run_inference(obs)
            self._set_action(action)

        # Publish latest action (may be previous if inference not done yet)
        out = np.zeros(len(self.controller_joint_names), dtype=np.float64)
        with self._lock:
            a = self._latest_action.copy()

        for pj, jn in enumerate(self.policy_joint_names):
            cj = self.policy_to_controller[pj]
            v = float(a[pj])
            calib = self.joint_calib.get(jn, JointCalib())
            v = calib.sign * v + calib.offset
            out[cj] = v

        msg = Float64MultiArray()
        msg.data = out.tolist()
        self._pub.publish(msg)

    # -----------------------------
    # dq estimation
    # -----------------------------
    def _compute_dq(self, q: np.ndarray, dq_meas: np.ndarray, js_time: Optional[float]) -> np.ndarray:
        enabled = bool(self.get_parameter("dq_estimation_enabled").value)
        stale_sec = float(self.get_parameter("joint_state_stale_sec").value)

        # If we have measured velocities for all joints, use them (but filter)
        dq = np.zeros_like(q, dtype=np.float32)

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = None
        if self._last_q is not None and self._last_q_time is not None:
            dt = max(now - self._last_q_time, 1e-6)

        for i in range(len(q)):
            use_meas = not np.isnan(dq_meas[i])

            # Consider stale if stamp unknown or too old; with JointState we often don't have per-field timestamps
            if js_time is None:
                is_stale = False
            else:
                is_stale = (now - js_time) > stale_sec

            if use_meas and not is_stale:
                v = float(dq_meas[i])
            else:
                if enabled and dt is not None:
                    v = float((q[i] - self._last_q[i]) / dt)
                else:
                    v = 0.0

            dq[i] = self._dq_filters[i].update(v)

        # update history
        self._last_q = q.copy()
        self._last_q_time = now
        return dq

    # -----------------------------
    # Inference thread
    # -----------------------------
    def _inference_worker(self):
        while not self._stop_event.is_set():
            self._new_obs_event.wait(timeout=0.1)
            if self._stop_event.is_set():
                break
            self._new_obs_event.clear()

            with self._lock:
                obs = None if self._latest_obs is None else self._latest_obs.copy()

            if obs is None:
                continue

            try:
                action = self._run_inference(obs)
                self._set_action(action)
            except Exception as e:
                # Keep last action
                self.get_logger().error(f"Inference error: {e}")

    def _run_inference(self, obs: np.ndarray) -> np.ndarray:
        out = self.policy(obs)
        out = np.asarray(out, dtype=np.float32)
        # Some policies output (1, act_dim)
        out = out.reshape(-1)
        if out.shape[0] != len(self.policy_joint_names):
            raise RuntimeError(
                f"Policy output dim {out.shape[0]} != expected {len(self.policy_joint_names)}"
            )
        return out

    def _set_action(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        with self._lock:
            self._latest_action[:] = action
        # last_action observation term must reflect previous action at next step
        self.obs_builder.set_last_action(action)

    def destroy_node(self):
        self._stop_event.set()
        self._new_obs_event.set()
        try:
            if self._use_thread and hasattr(self, "_worker"):
                self._worker.join(timeout=1.0)
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = IsaacPolicyPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
