# kuroko_gazebo

This package provides a Gazebo (ROS 2) simulation setup for **Kuroko**, including
`ros2_control` integration and flexible controller configuration via YAML files.

The launch system is designed to **load and activate all controllers defined in a given
ros2_control YAML**, making it easy to experiment with different controller combinations
without modifying launch files.

---

## Features

- Gazebo simulation using `gazebo_ros`
- `ros2_control` integration via `gazebo_ros2_control`
- Controller configuration fully driven by YAML
- All controllers with a `type` field in YAML are:
  - detected automatically
  - printed at launch time (for debugging)
  - loaded and activated after the robot is spawned
- Simple controller switching via launch arguments

---

## Package Structure (relevant parts)

```
kuroko_gazebo/
├── launch/
│   └── kuroko_gazebo.launch.py
└── worlds/
    └── default.world
```

Controller YAML files are located in the **kuroko_description** package:

```
kuroko_description/
└── config/
    └── ros2_control/
        ├── joint_trajectory_controller.yaml
        └── joint_group_position_controller.yaml
```

---

## Default Controller Configuration

If no controller YAML is specified at launch time, the following file is used by default:

```
kuroko_description/config/ros2_control/joint_trajectory_controller.yaml
```

This is resolved internally using:

```python
get_package_share_directory("kuroko_description")
```

---

## Launching the Simulation

### Default (JointTrajectoryController)

```bash
ros2 launch kuroko_gazebo kuroko_gazebo.launch.py
```

This uses:

```
kuroko_description/config/ros2_control/joint_trajectory_controller.yaml
```

---

### Using JointGroupPositionController

To explicitly use `joint_group_position_controller.yaml`:

```bash
ros2 launch kuroko_gazebo kuroko_gazebo.launch.py \
  controller_yaml:=$(ros2 pkg prefix kuroko_description)/share/kuroko_description/config/ros2_control/joint_group_position_controller.yaml
```

> The `controller_yaml` argument must be an **absolute path**.

---

## Controller Loading Behavior

At launch time:

1. The specified controller YAML is parsed.
2. All entries under:
   ```
   controller_manager.ros__parameters
   ```
   that contain a `type` field are detected as controllers.
3. All detected controllers are printed to the console:
   ```
   [kuroko] Controllers found in YAML
   - joint_state_broadcaster: type=...
   - joint_trajectory_controller: type=...
   ```
4. After the robot is spawned in Gazebo:
   - All detected controllers are loaded
   - All are set to `active`
   - `joint_state_broadcaster` is activated first if present

If any controller fails to load (invalid type, missing plugin, etc.),
**the launch will fail immediately**, making configuration errors obvious.

---

## Notes and Design Philosophy

- The YAML file is treated as the **single source of truth**.
- The launch file does **not** try to guess which controller is “correct”.
- Multiple controllers controlling different joint sets are fully supported.
- If the YAML is incorrect, it is better to fail fast than to hide errors.

This design is intentional and optimized for development, debugging, and experimentation.

---

## Debugging Tips

- Check the console output for:
  ```
  Controllers found in YAML
  Controllers activation order
  ```
- If Gazebo starts but controllers do not work:
  - Verify controller `type` strings
  - Check that required controller plugins are installed
- Use `ros2 control list_controllers` to inspect runtime state.

---

## License

Apache License 2.0
