#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

namespace kuroko_odom
{

inline double wrapToPi(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0)
    angle += 2.0 * M_PI;
  return angle - M_PI;
}

inline Eigen::Vector3d quaternionToProjectedRollPitchYaw(const Eigen::Quaterniond& q)
{
  // Uses relative displacement of basis vectors to avoid gimbal lock.
  // This is NOT identical to XYZ Euler angles.
  Eigen::Vector3d x_origin = Eigen::Vector3d::UnitX();  // Front
  Eigen::Vector3d y_origin = Eigen::Vector3d::UnitY();  // Left
  Eigen::Vector3d z_origin = Eigen::Vector3d::UnitZ();  // Up

  Eigen::Vector3d x_robot = q * x_origin;
  Eigen::Vector3d y_robot = q * y_origin;
  Eigen::Vector3d z_robot = q * z_origin;

  double roll = 0, pitch = 0, yaw = 0;

  // pitch: how much the robot's x-axis tilts from the origin XY plane
  pitch = std::atan2(-x_robot.z(), std::sqrt(x_robot.x() * x_robot.x() + x_robot.y() * x_robot.y()));
  if (z_robot.z() < 0.0)
    pitch = M_PI - pitch;  // upside-down correction

  // roll: how much the robot's y-axis tilts from the origin XY plane
  roll = std::atan2(y_robot.z(), std::sqrt(y_robot.x() * y_robot.x() + y_robot.y() * y_robot.y()));

  // yaw: rotation around origin Z axis
  if (std::fabs(x_robot.z()) < std::fabs(y_robot.z()))
  {
    // If x-axis tilt is smaller, use x-axis to compute yaw
    yaw = std::atan2(x_robot.y(), x_robot.x());
    if (z_robot.z() < 0.0)
      yaw = yaw + M_PI;  // upside-down correction
  }
  else
  {
    // Else use y-axis to compute yaw
    yaw = std::atan2(y_robot.y(), y_robot.x()) - M_PI / 2.0;
  }

  roll = wrapToPi(roll);
  pitch = wrapToPi(pitch);
  yaw = wrapToPi(yaw);
  return Eigen::Vector3d(roll, pitch, yaw);
}

inline Eigen::Quaterniond projectedRollPitchYawToQuaternion(const Eigen::Vector3d& rpy)
{
  // Reconstruct a rotation whose projected-RPY matches quaternionToProjectedRollPitchYaw().
  const double roll  = wrapToPi(rpy.x());
  const double pitch = wrapToPi(rpy.y());
  const double yaw   = wrapToPi(rpy.z());

  const bool inverted = (std::cos(pitch) < 0.0);

  // --- Reconstruct x_robot from (pitch, yaw) ---
  const double xz = -std::sin(pitch);
  const double hx = std::abs(std::cos(pitch));
  const double alpha = wrapToPi(yaw - (inverted ? M_PI : 0.0));

  Eigen::Vector3d x_robot;
  x_robot << hx * std::cos(alpha), hx * std::sin(alpha), xz;

  // --- Build y candidate from (roll) and yaw-consistent heading ---
  const double yz = std::sin(roll);
  const double hy = std::abs(std::cos(roll));
  const double beta = wrapToPi(alpha + M_PI / 2.0);

  Eigen::Vector3d y_candidate;
  y_candidate << hy * std::cos(beta), hy * std::sin(beta), yz;

  // --- Orthonormalize ---
  Eigen::Vector3d y_robot = y_candidate - x_robot * (x_robot.dot(y_candidate));
  double yn = y_robot.norm();
  if (yn < 1e-12)
  {
    Eigen::Vector3d tmp = (std::abs(x_robot.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
    y_robot = tmp - x_robot * (x_robot.dot(tmp));
    yn = y_robot.norm();
  }
  y_robot /= yn;

  Eigen::Vector3d z_robot = x_robot.cross(y_robot);
  z_robot.normalize();

  if ((!inverted && z_robot.z() < 0.0) || (inverted && z_robot.z() > 0.0))
  {
    y_robot = -y_robot;
    z_robot = -z_robot;
  }

  Eigen::Matrix3d R;
  R.col(0) = x_robot;
  R.col(1) = y_robot;
  R.col(2) = z_robot;

  Eigen::Quaterniond q(R);
  q.normalize();
  return q;
}

inline Eigen::Quaterniond yawToQuaternion(double yaw)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

inline double lerpClamped(double x, double x0, double x1, double y0, double y1)
{
  if (x <= x0) return y0;
  if (x >= x1) return y1;
  const double t = (x - x0) / (x1 - x0);
  return y0 + t * (y1 - y0);
}

}  // namespace kuroko_odom
