# kuroko_odom

`kuroko_odom` is a ROS 2 (Humble) package that estimates the odometry of the humanoid robot **Kuroko**
using **foot contact kinematics + IMU orientation**.

The odometry is computed under the assumption that the **currently contacting foot vertex is fixed
in the world**, and the base motion is inferred from the relative motion of that contact point.

This package depends on `kuroko_kinematics` for forward kinematics.

---

## Features

- Uses **joint states + IMU quaternion** to estimate base odometry
- Automatically selects the **stance foot** based on the lowest foot vertex
- Tracks the **current contacting vertex** and its displacement
- Separates:
  - **Body odometry** (full orientation from IMU)
  - **Foot odometry** (horizontal pose with projected yaw only)
- Supports `/initialpose` from RViz
- Dynamically adjusts **odometry covariance** based on:
  - Stance foot vertical acceleration
  - Foot tilt angle
- Publishes TF via a separate node with selectable direction

---

## Nodes

### 1. `foot_odom_node`

#### Subscribed Topics
| Topic | Type | Description |
|------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Joint angles |
| `/imu` | `sensor_msgs/Imu` | IMU orientation (quaternion only) |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Initial pose from RViz |

#### Published Topics
| Topic | Type | Description |
|------|------|-------------|
| `/body_odom` | `nav_msgs/Odometry` | Base odometry with full IMU orientation |
| `/foot_odom` | `nav_msgs/Odometry` | Horizontal odometry using projected yaw |

#### Core Logic
- Forward kinematics is computed for both legs
- All foot vertices are evaluated
- The **lowest vertex** is treated as the contact point
- The **displacement of the current contact vertex** between consecutive samples is used to update base translation
- Base yaw is updated from IMU projected yaw
- IMU timeout fallback:
  - Base is treated as upright
  - Yaw is estimated from stance-foot yaw motion
  - Covariance is set to maximum

---

### 2. `odom_tf_node`

#### Subscribed Topics
| Topic | Type |
|------|------|
| Odometry topic (configurable) | `nav_msgs/Odometry` |

#### Published TF
- Publishes a transform between `odom` and `base_link`
- Direction can be selected by parameter

---

## Parameters

### Common Frames
| Name | Default | Description |
|----|----|----|
| `odom_frame_id` | `odom` | Odometry frame |
| `base_frame_id` | `base_link` | Robot base frame |

### Topics
| Name | Default |
|----|----|
| `joint_states_topic` | `/joint_states` |
| `imu_topic` | `/imu` |
| `initialpose_topic` | `/initialpose` |
| `body_odom_topic` | `/body_odom` |
| `foot_odom_topic` | `/foot_odom` |

### Foot Geometry
| Name | Description |
|----|----|
| `right_footprint_xy` | Flattened XY vertices in foot frame |
| `left_footprint_xy` | Flattened XY vertices in foot frame |

Default footprint:
```
[[ 0.06,  0.0375],
 [ 0.06, -0.0375],
 [-0.06, -0.0375],
 [-0.06,  0.0375]]
```

### Joint Names
Must match `/joint_states`.

- `right_leg_joint_names` (6 DOF)
- `left_leg_joint_names` (6 DOF)

---

### Covariance Control

#### Translation
| Parameter | Default |
|----|----|
| `stance_up_acc_threshold` | `9.81` |
| `stance_down_acc_threshold` | `0.0` |
| `cov_trans_min` | `0.1` |
| `cov_trans_max` | `1e6` |

#### Rotation
| Parameter | Default |
|----|----|
| `foot_tilt_threshold_rad` | `0.2` |
| `cov_rot_min` | `0.1` |
| `cov_rot_max` | `1e6` |

Covariance is interpolated linearly between min/max based on:
- Vertical acceleration of the stance foot
- Foot tilt angle relative to world Z

---

### IMU Handling
| Parameter | Default |
|----|----|
| `imu_timeout_sec` | `0.1` |

If IMU data times out:
- Base orientation is treated as upright
- Yaw is estimated from stance-foot kinematics
- Covariance is set to maximum

---

## Launch

### Basic Usage
```bash
ros2 launch kuroko_odom foot_odom.launch.py
```

### Use body odometry for TF
```bash
ros2 launch kuroko_odom foot_odom.launch.py odom_topic:=/body_odom
```

---

## Design Notes

- Odometry translation is derived from **contact point displacement**, not from velocity integration
- The tracked contact point is **always the currently contacting vertex**
- The contact trajectory is continuous; position and velocity are continuous, while jerk may be non-linear
- No absolute world contact position is stored; only relative motion is used

---

## Dependencies

- ROS 2 Humble
- Eigen3
- `kuroko_kinematics`

---
