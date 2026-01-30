# kuroko_isaac_config_generator

This package hosts multiple ROS2 nodes that generate Isaac Lab configuration assets for the Kuroko robot.

## Node: usda_generator

Pipeline (mandatory intermediate):
1. xacro -> URDF (gazebo:=true, controller_yaml passed in exactly like kuroko_gazebo.launch.py)
2. URDF -> SDF (libsdformat helper; produces canonical SDF)
3. SDF URI normalization (package://, model://, file://)
4. (optional) SDF -> USDA via Isaac Sim/Kit external process

### Build

```bash
colcon build --packages-select kuroko_isaac_config_generator
source install/setup.bash
```

### Run (generate URDF + SDF)

```bash
ros2 run kuroko_isaac_config_generator usda_generator_node.py \
  --ros-args \
  -p output_dir:=/tmp/kuroko_usd_out \
  -p basename:=kuroko \
  -p gazebo:=true
```

### Isaac export

The Isaac export step is environment/version dependent.
- Edit: `share/kuroko_isaac_config_generator/usda_generator/isaac_usd_export_script.py`
- Then run with:

```bash
ros2 run kuroko_isaac_config_generator usda_generator_node.py \
  --ros-args -p run_isaac_export:=true -p isaac_app:=isaacsim
```

Notebook:
`share/kuroko_isaac_config_generator/notebooks/usda_generator/01_kuroko_urdf_sdf_usda.ipynb`
