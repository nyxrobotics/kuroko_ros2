# kuroko_isaac_player

A ROS 2 Humble package to run an Isaac Lab–trained policy (`.pt`) and publish joint position targets to
`/joint_group_position_controller/commands` at **200 Hz (configurable)**.

- **Observations (inputs)**
  - `/cmd_vel` (velocity command; treated as 0 until received, and reset to 0 after a timeout)
  - `/joint_states` (joint positions and optionally velocities; can estimate velocities if missing/slow)
  - `/kuroko/sensors/imu/data` (IMU)
- **Actions (output)**
  - `/joint_group_position_controller/commands` (`std_msgs/Float64MultiArray`)

---

## 1. Recommended asset layout

Place the training artifacts under this package’s `share/` (or your workspace copy) like:

```
kuroko_isaac_player/
  trained_data/
    kuroko_walk/
      model_13400.pt
      params/
        env.yaml
```

> `env.yaml` is expected to be the original file produced by training. It may include Python-specific YAML tags
(e.g., `python/tuple`, slice tags), which the loader in this package supports.

---

## 2. Dependencies

- ROS 2 Humble
- `kuroko_description` (controller config YAML is referenced here)
  - `kuroko_description/config/gz_position_controller.yaml`
- PyTorch (for inference)
  - If the `.pt` cannot be loaded as TorchScript, this package falls back to loading a checkpoint/state_dict.

---

## 3. Build & run (using a conda environment)

This node is intended to run with **the conda Python/PyTorch environment** you activate before launching.

### 3.1 Build after activating conda

```bash
conda activate env_isaaclab
cd ~/ros2_humble
colcon build --packages-select kuroko_isaac_player
source install/setup.bash
```

> If `ros2` ends up using the system Python, verify `which python` and `which ros2`.
Always run `colcon build` and `ros2 launch` after activating conda.

### 3.2 Launch

```bash
conda activate env_isaaclab
source ~/ros2_humble/install/setup.bash
ros2 launch kuroko_isaac_player player.launch.py
```

---

## 4. Topics

### Subscribed

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/joint_states` (`sensor_msgs/JointState`)
- `/kuroko/sensors/imu/data` (`sensor_msgs/Imu`)

### Published

- `/joint_group_position_controller/commands` (`std_msgs/Float64MultiArray`)

---

## 5. Joint ordering and mapping

### 5.1 Observations (`/joint_states` → policy joint order)

The node remaps joint arrays using `/joint_states.name` into the **policy joint order** stored in `env.yaml`
(`policy_joint_names`). This keeps observations stable even if the order in `/joint_states` changes.

### 5.2 Actions (policy action order → controller command order)

`/joint_group_position_controller/commands` is a numeric array without joint names. Therefore this node builds a
mapping from:

- `gz_position_controller.yaml` (controller-side `joints` array order)
- `env.yaml` (policy `policy_joint_names` order)

On startup (for debugging), it prints:

- Observation layout (which term comes from which topic/fields)
- Policy joint order (from `env.yaml`)
- Controller joint command order (from `gz_position_controller.yaml`)
- Policy → controller index mapping
- Default pose (from `env.yaml`)

---

## 6. Startup behavior

### 6.1 `startup_move_to_default` (optional)

When `startup_move_to_default=true`:

1. After the first `/joint_states` is received, the node uses the measured pose as the start pose
2. It linearly interpolates to the default pose from `env.yaml` over **1 second** (configurable)
3. **Policy inference does not start until this startup move finishes**
4. While moving, the “previous action” observation is initialized from the **target pose being sent** during the move

### 6.2 Initializing “previous action” (`actions` observation term)

- If `init_last_action_from_joint_states=true`, the node initializes the `actions` term once at startup using the
  **measured pose from `/joint_states`**, converted back into action space.
- If `startup_move_to_default=true`, initialization is done using the **startup move target pose** instead.

This reduces violent motion at the first few inference steps.

---

## 7. `/cmd_vel` behavior (no waiting + timeout)

- The node **does not wait** for `/cmd_vel`. Until the first message is received, it behaves as if the command is zero.
- A timeout `cmd_vel_timeout_sec` (default: 1.0) is applied. If no new `/cmd_vel` arrives within this period,
  the command is overwritten with zeros.

---

## 8. Joint velocity (`dq`) source selection

To handle cases where `/joint_states.velocity` is missing or unreliable:

- `joint_velocity_source: auto`  
  Use `/joint_states.velocity` when valid; otherwise estimate from position differences.
- `joint_velocity_source: topic`  
  Always use `/joint_states.velocity` (if missing/invalid, zeros are used).
- `joint_velocity_source: estimate`  
  Always estimate from position differences (ignores topic velocity).  
  A low-pass filter cutoff is controlled by `dq_cutoff_hz`.

---

## 9. Key parameters (high level)

See `declare_parameter()` in `policy_player_node.py` for the full list.

### Rate / performance

- `publish_hz` (float, default: 200.0)
- `use_inference_thread` (bool, default: true)  
  Runs inference on the latest observation in a separate thread (helps when inference is heavy).

### Log suppression (for reaching 200 Hz)

- `startup_print` (bool, default: true)
- `debug_print` (bool, default: true)
- `debug_print_every` (int, default: 50)
- `debug_print_joint_values` (bool, default: true)

> If you cannot reach 200 Hz, first try `debug_print=false`.

### Startup move

- `startup_move_to_default` (bool, default: false)
- `startup_move_duration_sec` (float, default: 1.0)

### `/cmd_vel`

- `cmd_vel_timeout_sec` (float, default: 1.0)

### Velocity estimation

- `joint_velocity_source` ("auto"|"topic"|"estimate", default: "auto")
- `dq_cutoff_hz` (float, default: 30.0)

### Previous-action initialization

- `init_last_action_from_joint_states` (bool, default: true)

---

## 10. Troubleshooting

### 10.1 `.pt` cannot be loaded with TorchScript

It is normal to see:

- `TorchScript load failed ... Falling back to torch.load checkpoint.`

This means the node loaded a checkpoint/state_dict. If the inferred network structure does not match what was trained,
the behavior will be wrong. Prefer exporting TorchScript from the training side when possible.

### 10.2 Not reaching 200 Hz

Try, in order:

- `debug_print=false`
- `startup_print=false`
- `use_inference_thread=true`
- Temporarily lower `publish_hz` to validate behavior

### 10.3 Violent/unstable motion

Check the startup logs:

- Observation layout dump (which topics/fields are used)
- Policy joint order / controller joint order / mapping
- Default pose

A wrong controller joint order or mapping is a common cause of instability.

---

## 11. License

Follow your repository/project policy (see `package.xml` and the repository LICENSE if present).
