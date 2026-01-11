#ifndef KUROKO_KINEMATICS_EIGEN_MATH_HPP_
#define KUROKO_KINEMATICS_EIGEN_MATH_HPP_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace motion_control::eigen_math
{

inline Eigen::Vector3d transitionXYZ(const double x, const double y, const double z)
{
  return Eigen::Vector3d(x, y, z);
}

inline Eigen::Matrix3d rotationFromRPY(const double roll, const double pitch, const double yaw)
{
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  // R = Rz(yaw) * Ry(pitch) * Rx(roll)
  return (rz * ry * rx).toRotationMatrix();
}

inline Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d& R)
{
  // Assumes R = Rz(yaw) * Ry(pitch) * Rx(roll)
  const double pitch = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));

  double roll = 0.0;
  double yaw = 0.0;
  const double cp = std::cos(pitch);
  if (std::abs(cp) < 1e-12)
  {
    // Gimbal lock: roll is not observable; set roll=0 and solve yaw from remaining terms.
    roll = 0.0;
    yaw = std::atan2(-R(0, 1), R(1, 1));
  }
  else
  {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));
  }

  return Eigen::Vector3d(roll, pitch, yaw);
}

inline Eigen::Matrix4d transformXYZRPY(
  const double x, const double y, const double z, const double roll, const double pitch, const double yaw)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = rotationFromRPY(roll, pitch, yaw);
  T.block<3, 1>(0, 3) = transitionXYZ(x, y, z);
  return T;
}

}  // namespace motion_control::eigen_math

#endif  // KUROKO_KINEMATICS_EIGEN_MATH_HPP_
