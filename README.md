# kuroko_ros2

ROS 2 Humble mono-repo for the Kuroko humanoid: Gazebo (gz-sim) simulation and real-robot bringup, with cmd_vel teleop and learned locomotion policies.

## Repository structure

Packages are placed directly under the repository root (no extra `src/` directory).

### Common packages
- `kuroko_description`
- `kuroko_control`
- `kuroko_policy`

### Simulation-only packages
- `kuroko_sim_gz`
- `kuroko_sim_bringup`

### Hardware-only packages
- `kuroko_hw_interface`
- `kuroko_hw_bringup`

## License

Apache-2.0 (see `LICENSE`).
