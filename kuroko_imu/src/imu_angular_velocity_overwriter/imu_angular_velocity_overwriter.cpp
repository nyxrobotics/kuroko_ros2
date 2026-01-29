#include "imu_angular_velocity_overwriter/imu_angular_velocity_overwriter.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/LinearMath/Vector3.h"

namespace kuroko_imu
{

ImuAngularVelocityOverwriter::ImuAngularVelocityOverwriter(const rclcpp::NodeOptions & options)
: Node("imu_angular_velocity_overwriter", options)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/imu/data");
  output_topic_ = declare_parameter<std::string>("output_topic", "/imu/data_overwritten");
  use_header_stamp_ = declare_parameter<bool>("use_header_stamp", true);
  min_dt_ = declare_parameter<double>("min_dt", 1e-4);
  zero_on_first_msg_ = declare_parameter<bool>("zero_on_first_msg", true);

  pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());
  sub_ = create_subscription<sensor_msgs::msg::Imu>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ImuAngularVelocityOverwriter::onImu, this, std::placeholders::_1));
}

tf2::Quaternion ImuAngularVelocityOverwriter::toQuat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  const double n = tq.length();
  if (n > 0.0) {
    tq /= n;
  } else {
    tq.setValue(0.0, 0.0, 0.0, 1.0);
  }
  return tq;
}

void ImuAngularVelocityOverwriter::onImu(const sensor_msgs::msg::Imu::SharedPtr msg_in)
{
  sensor_msgs::msg::Imu msg = *msg_in;

  const tf2::Quaternion q_curr = toQuat(msg.orientation);
  const rclcpp::Time t_curr = use_header_stamp_
    ? rclcpp::Time(msg.header.stamp)
    : now();

  if (!has_prev_) {
    q_prev_ = q_curr;
    t_prev_ = t_curr;
    has_prev_ = true;
    if (zero_on_first_msg_) {
      msg.angular_velocity.x = 0.0;
      msg.angular_velocity.y = 0.0;
      msg.angular_velocity.z = 0.0;
    }
    pub_->publish(msg);
    return;
  }

  double dt = (t_curr - t_prev_).seconds();
  if (dt < 0.0) {
    q_prev_ = q_curr;
    t_prev_ = t_curr;
    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = 0.0;
    pub_->publish(msg);
    return;
  }
  dt = std::max(dt, min_dt_);

  tf2::Quaternion dq = q_prev_.inverse() * q_curr;
  dq.normalize();
  if (dq.w() < 0.0) {
    dq = tf2::Quaternion(-dq.x(), -dq.y(), -dq.z(), -dq.w());
  }

  const double w = std::clamp(dq.w(), -1.0, 1.0);
  const double angle = 2.0 * std::acos(w);
  const double s = std::sqrt(std::max(0.0, 1.0 - w * w));

  tf2::Vector3 axis(0.0, 0.0, 0.0);
  if (s > 1e-12) {
    axis = tf2::Vector3(dq.x() / s, dq.y() / s, dq.z() / s);
  }

  const tf2::Vector3 ang_vel = axis * (angle / dt);
  msg.angular_velocity.x = ang_vel.x();
  msg.angular_velocity.y = ang_vel.y();
  msg.angular_velocity.z = ang_vel.z();

  q_prev_ = q_curr;
  t_prev_ = t_curr;

  pub_->publish(msg);
}

}  // namespace kuroko_imu
