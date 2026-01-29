# kuroko_joint_states

ROS 2 Humble package that subscribes to `/joint_states` and overwrites `velocity` using `(position - previous_position) / dt`.
`dt` source can be selected via parameter.

## Build

```bash
colcon build --packages-select kuroko_joint_states
source install/setup.bash
```

## Run

### With launch
```bash
ros2 launch kuroko_joint_states joint_states_velocity_overwriter.launch.py
```

### Launch arguments
```bash
ros2 launch kuroko_joint_states joint_states_velocity_overwriter.launch.py \
  input_topic:=/joint_states \
  output_topic:=/joint_states/velocity_overwritten \
  use_header_stamp:=true \
  zero_effort:=false
```
