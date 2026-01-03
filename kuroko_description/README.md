# kuroko_description

Robot description package for the Kuroko humanoid in ROS 2.

This package provides the URDF/Xacro files, meshes, and visualization launch
utilities used across:

- RViz2 visualization
- Gazebo (gz-sim) simulation
- ros2_control
- Real hardware bringup

The description is designed so that visualization and physics simulation
use different robot representations generated from the same Xacro source.

## Xacro design

### `gazebo` argument

All Xacro files accept a `gazebo` argument to switch the generated robot model.

- `gazebo:=false` (default)
  - RViz2 / robot_state_publisher friendly
  - Mimic joints enabled
  - No closed-loop constraints
  - Pure tree-structured URDF

- `gazebo:=true`
  - Gazebo (gz-sim) / SDF friendly
  - Mimic joints disabled
  - Closed-loop mechanisms enabled via `<gazebo>` tags
  - Intended for physics simulation only

Gazebo-specific structures are enclosed in `<gazebo>` tags so that they
do not affect RViz2 or TF generation.

## Directory structure

```
kuroko_description/
├── launch/
│   └── test_xacro.launch.py
├── xacro/
│   └── kuroko/
│       ├── kuroko.xacro
│       └── parts/
│           ├── body.xacro
│           ├── arm_l.xacro
│           ├── arm_r.xacro
│           ├── leg_l.xacro
│           └── leg_r.xacro
├── meshes/
├── rviz/
│   └── test_xacro.rviz
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Launch files

### `test_xacro.launch.py`

RViz2-only launch file for validating the robot description.

- Always generates the robot with `gazebo:=false`
- Intended for:
  - URDF structure validation
  - TF tree inspection
  - Mimic joint verification
  - Visual inspection in RViz2

Example:

```bash
ros2 launch kuroko_description test_xacro.launch.py
```

Optional arguments:

```bash
gui:=true    # Use joint_state_publisher_gui (default)
rviz:=true  # Launch RViz2 (default)
```

The default RViz2 configuration file used by this launch is:

```
rviz/test_xacro.rviz
```

## Dependencies

This package expects the following ROS 2 packages to be installed:

```bash
sudo apt install -y \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2
```

## Gazebo integration

This package does not start Gazebo.

Gazebo (gz-sim) integration, URDF-to-SDF conversion, and robot spawning are
handled by the following packages:

- `kuroko_sim_gz`
- `kuroko_sim_bringup`

Those packages are responsible for invoking Xacro with `gazebo:=true`
and spawning the resulting model into the simulator.

## License

Apache License 2.0
