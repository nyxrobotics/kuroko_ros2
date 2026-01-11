#ifndef KUROKO_KINEMATICS_H_
#define KUROKO_KINEMATICS_H_

#include <Eigen/Core>
#include <vector>

#include "kuroko_kinematics/joint_data.h"

namespace motion_control
{

class KurokoKinematics
{
public:
  KurokoKinematics();
  ~KurokoKinematics();

  bool solveInverseKinematicsForRightLeg(std::vector<double>& joints_out, std::vector<double> target_pose_in);
  bool solveInverseKinematicsForLeftLeg(std::vector<double>& joints_out, std::vector<double> target_pose_in);

  bool solveForwardKinematicsForRightLeg(const std::vector<double> joints_in, std::vector<double>& target_pose_out);
  bool solveForwardKinematicsForLeftLeg(const std::vector<double> joints_in, std::vector<double>& target_pose_out);

  Eigen::Vector3d getJointAxis(const std::string& link_name);
  double getJointDirection(const std::string& link_name);
  double getJointDirection(const int link_id);
  double leg_max_height_;
  double leg_side_offset_;
  double gripper_length_;

private:
  std::vector<JointData> joint_tree_;
  JointData* getJointData(const std::string& link_name);
  JointData* getJointData(const int link_id);
  int getLinkIndex(const std::string& link_name);
};

}  // namespace motion_control

#endif /* KUROKO_KINEMATICS_H_ */
