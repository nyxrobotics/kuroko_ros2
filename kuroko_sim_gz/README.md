# kuroko_sim_gz

This package provides a Gazebo Sim (gz-sim) environment for **Kuroko**, integrated with
**ros2_control** and **joint_trajectory_controller (effort interface)**, allowing you to
verify joint motion interactively.

The package launches:

- Gazebo Sim world (started **paused**)
- Kuroko robot spawn
- Gazebo → ROS 2 `/clock` bridge
- `joint_state_broadcaster`
- `joint_trajectory_controller` (trajectory *positions* → PID → *effort*)

---

## Prerequisites

- ROS 2 Humble
- Gazebo Sim (gz-sim)
- `kuroko_description` package built and installed

---

## Install dependencies

### ros2_control and controllers
```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

### rqt Joint Trajectory Controller (for joint motion testing)
```bash
sudo apt install ros-humble-rqt-joint-trajectory-controller
```

---

## Environment setup

Before launching, make sure to source ROS 2 and your workspace
**in the same terminal**:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_humble/install/setup.bash
```

> If you are using `ROS_DOMAIN_ID`, ensure the same value is used for Gazebo and all ROS 2 nodes.

---

## Launch

### Start Gazebo + Kuroko + controllers + `/clock`

```bash
ros2 launch kuroko_sim_gz kuroko_gz_sim.launch.py
```

After launch:

- Gazebo starts in **Pause** state
- Press ▶ **Play** to advance simulation time
- `/clock` will be published and controllers will start updating

---

## Joint motion verification using rqt

### Start rqt
```bash
rqt --force-discover
```

### Open the correct plugin

In the rqt menu:

```
Plugins
  └ Robot Tools
      └ Joint Trajectory Controller
```

### How to test a joint

1. Select the controller  
   **`/joint_trajectory_controller`**
2. Select the joint to control (e.g. `chest`)
3. Enter a target position (radians)
4. Click **Send Trajectory**

If the joint moves in Gazebo, the controller is working correctly.

---

## Troubleshooting checklist

- Is Gazebo running (**▶ Play**, not paused)?
- Is `/clock` published?
  ```bash
  ros2 topic info /clock -v
  ```
  → `Publisher count` should be **1 or more**
- Are controllers running?
  ```bash
  ros2 node list | grep controller
  ```

---

## Notes

- `gz_trajectory_controller.yaml` is located in  
  **`kuroko_description/config`**
- The trajectory controller uses the **effort** hardware interface
- Trajectory commands specify **desired joint positions**

---

## Launch files overview

- `kuroko_gz_sim.launch.py`  
  Full system launch (world, robot, clock bridge, controllers)
- `spawn_world.launch.py`  
  Gazebo world launch
- `spwan_kuroko.launch.py`  
  Kuroko robot spawn
- `gz_bridge.launch.py`  
  Gazebo → ROS 2 `/clock` bridge
- `trajectory_controller.launch.py`  
  ros2_control controller spawner

---

