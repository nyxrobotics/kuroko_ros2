#include "kuroko_kinematics/joint_data.h"

#include "kuroko_kinematics/eigen_math.hpp"

namespace motion_control
{

JointData::JointData()
{
  // Tree parameters
  name_ = "";
  parent_ = -1;
  child_ = -1;

  // Origin offset from parent frame
  offset_position_ = eigen_math::transitionXYZ(0.0, 0.0, 0.0);
  offset_orientation_ = eigen_math::rotationFromRPY(0.0, 0.0, 0.0);

  // Joint parameters
  joint_axis_ = eigen_math::transitionXYZ(0.0, 0.0, 0.0);
  joint_limit_lower_ = -100.0;
  joint_limit_upper_ = 100.0;
  joint_mimic_ = "";
  joint_mimic_multiplier_ = 1.0;
}

JointData::~JointData() = default;

}  // namespace motion_control
