#ifndef KUROKO_KINEMATICS_JOINT_DATA_H_
#define KUROKO_KINEMATICS_JOINT_DATA_H_

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace motion_control
{

// This class stores parameters required for kinematics.
// - Mass/COM/inertia are intentionally omitted because they are not used.
// - "internal_*" variables are also omitted because they are not used.
// - The origin offset from the parent frame is expressed by offset_position_ and offset_orientation_.
class JointData
{
public:
  JointData();
  ~JointData();

  // Tree parameters
  std::string name_;
  int parent_;
  int child_;

  // Origin offset from parent frame to this joint frame
  Eigen::Vector3d offset_position_;
  Eigen::Matrix3d offset_orientation_;

  // Joint parameters
  Eigen::Vector3d joint_axis_;
  double joint_limit_upper_;
  double joint_limit_lower_;
  std::string joint_mimic_;
  double joint_mimic_multiplier_;
};

}  // namespace motion_control

#endif  // KUROKO_KINEMATICS_JOINT_DATA_H_
