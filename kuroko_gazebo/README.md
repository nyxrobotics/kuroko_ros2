# kuroko_gazebo (ROS 2 Humble, Gazebo Classic)

This is a ROS 2 Humble port of the original ROS 1 `kuroko_gazebo` package, targeting **Gazebo Classic** (`gazebo_ros`).

## Provided launch files

- `ground_gazebo.launch.py`
- `roboone_gazebo.launch.py`

Both launch files:

1. start Gazebo Classic (paused)
2. generate the URDF from xacro (gazebo:=true/false)
3. spawn entities using `scripts/spawn_entity_from_param.py`
4. run `robot_state_publisher`
5. unpause physics once the expected models exist (`scripts/unpause_physics.py`)

## How to build

```bash
cd ~/ros2_ws/src
git clone <your repo containing this package>
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## How to run

```bash
ros2 launch kuroko_gazebo ground_gazebo.launch.py
ros2 launch kuroko_gazebo roboone_gazebo.launch.py
```

Useful overrides:

```bash
ros2 launch kuroko_gazebo ground_gazebo.launch.py gui:=false headless:=true
ros2 launch kuroko_gazebo ground_gazebo.launch.py robot_name:=kuroko controller:=trajectory
ros2 launch kuroko_gazebo ground_gazebo.launch.py robot_name:=kuroko controller:=group_position
```

## Controllers (ros2_control)

This package ships a ready-to-use controller config at:

- `config/ros2_controllers.yaml`

The launch files **attempt** to spawn controllers using `controller_manager/spawner`.
For this to work, your robot model must load `gazebo_ros2_control` (Gazebo Classic plugin) and pass the controller parameter file to the plugin, as documented by `gazebo_ros2_control`.

If your URDF does not start a `controller_manager`, you can still run Gazebo + spawn entities + TF, and add control later.

## Notes

- All Python scripts use the fixed shebang: `#!/usr/bin/python3`.
- Some Gazebo ROS service names differ across setups (`/spawn_entity` vs `/gazebo/spawn_entity`, etc.). The helper scripts try both variants.
