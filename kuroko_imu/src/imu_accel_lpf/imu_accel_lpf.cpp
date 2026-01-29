#include "imu_accel_lpf/imu_accel_lpf.hpp"

#include <algorithm>
#include <cmath>

namespace kuroko_imu
{

ImuAccelLpf::ImuAccelLpf(const rclcpp::NodeOptions & options)
: Node("imu_accel_lpf", options)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/imu/data");
  output_topic_ = declare_parameter<std::string>("output_topic", "/imu/data_accel_lpf");
  use_header_stamp_ = declare_parameter<bool>("use_header_stamp", true);
  min_dt_ = declare_parameter<double>("min_dt", 1e-4);

  time_constant_norm_sec_ = declare_parameter<double>("time_constant_norm_sec", 0.01);
  time_constant_dir_sec_ = declare_parameter<double>("time_constant_dir_sec", 0.01);

  pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());
  sub_ = create_subscription<sensor_msgs::msg::Imu>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ImuAccelLpf::onImu, this, std::placeholders::_1));
}

double ImuAccelLpf::clamp(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

double ImuAccelLpf::computeAlpha(double dt, double tau)
{
  const double tau_safe = std::max(tau, 0.0);
  if (tau_safe <= 0.0) {
    return 1.0;
  }
  return dt / (tau_safe + dt);
}

tf2::Vector3 ImuAccelLpf::safeNormalized(
  const tf2::Vector3 & v, double eps, const tf2::Vector3 & fallback)
{
  const double n = v.length();
  if (n > eps) {
    return v / n;
  }
  return fallback;
}

tf2::Vector3 ImuAccelLpf::slerpUnitVector(const tf2::Vector3 & u0, const tf2::Vector3 & u1, double t)
{
  const double tt = clamp(t, 0.0, 1.0);
  const tf2::Vector3 a = safeNormalized(u0, 1e-12, tf2::Vector3(0.0, 0.0, 1.0));
  const tf2::Vector3 b = safeNormalized(u1, 1e-12, a);

  double dot = clamp(a.dot(b), -1.0, 1.0);

  // If the vectors are extremely close, fall back to normalized lerp.
  if (dot > 0.9995) {
    return safeNormalized((1.0 - tt) * a + tt * b, 1e-12, a);
  }

  // If the vectors are nearly opposite, rotate a around an arbitrary orthogonal axis.
  if (dot < -0.9995) {
    tf2::Vector3 ortho = std::abs(a.x()) < 0.9 ? tf2::Vector3(1.0, 0.0, 0.0) : tf2::Vector3(0.0, 1.0, 0.0);
    tf2::Vector3 axis = a.cross(ortho);
    axis = safeNormalized(axis, 1e-12, tf2::Vector3(0.0, 0.0, 1.0));

    const double angle = M_PI * tt;
    const double c = std::cos(angle);
    const double s = std::sin(angle);

    // Rodrigues' rotation formula
    const tf2::Vector3 v = a * c + axis.cross(a) * s + axis * (axis.dot(a)) * (1.0 - c);
    return safeNormalized(v, 1e-12, a);
  }

  const double omega = std::acos(dot);
  const double sin_omega = std::sin(omega);
  const double s0 = std::sin((1.0 - tt) * omega) / sin_omega;
  const double s1 = std::sin(tt * omega) / sin_omega;
  return safeNormalized(s0 * a + s1 * b, 1e-12, a);
}

void ImuAccelLpf::onImu(const sensor_msgs::msg::Imu::SharedPtr msg_in)
{
  sensor_msgs::msg::Imu msg = *msg_in;

  const rclcpp::Time t_curr = use_header_stamp_ ? rclcpp::Time(msg.header.stamp) : now();
  if (!has_prev_) {
    const tf2::Vector3 a(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    norm_filt_ = a.length();
    dir_filt_ = safeNormalized(a, 1e-6, tf2::Vector3(0.0, 0.0, 1.0));
    t_prev_ = t_curr;
    has_prev_ = true;
    pub_->publish(msg);
    return;
  }

  double dt = (t_curr - t_prev_).seconds();
  if (dt < 0.0) {
    // Time went backwards: reset the filter state.
    const tf2::Vector3 a(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    norm_filt_ = a.length();
    dir_filt_ = safeNormalized(a, 1e-6, dir_filt_);
    t_prev_ = t_curr;
    pub_->publish(msg);
    return;
  }
  dt = std::max(dt, min_dt_);

  const tf2::Vector3 a_raw(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
  const double norm_raw = a_raw.length();
  const tf2::Vector3 dir_raw = safeNormalized(a_raw, 1e-6, dir_filt_);

  const double alpha_norm = computeAlpha(dt, time_constant_norm_sec_);
  const double alpha_dir = computeAlpha(dt, time_constant_dir_sec_);

  norm_filt_ = norm_filt_ + alpha_norm * (norm_raw - norm_filt_);
  dir_filt_ = slerpUnitVector(dir_filt_, dir_raw, alpha_dir);

  const tf2::Vector3 a_filt = dir_filt_ * norm_filt_;
  msg.linear_acceleration.x = a_filt.x();
  msg.linear_acceleration.y = a_filt.y();
  msg.linear_acceleration.z = a_filt.z();

  t_prev_ = t_curr;

  pub_->publish(msg);
}

}  // namespace kuroko_imu
