#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Vector3.h"

namespace kuroko_imu
{

class ImuAccelLpf : public rclcpp::Node
{
public:
  explicit ImuAccelLpf(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg_in);

  static double clamp(double v, double lo, double hi);
  static double computeAlpha(double dt, double tau);
  static tf2::Vector3 safeNormalized(const tf2::Vector3 & v, double eps, const tf2::Vector3 & fallback);
  static tf2::Vector3 slerpUnitVector(const tf2::Vector3 & u0, const tf2::Vector3 & u1, double t);

  std::string input_topic_;
  std::string output_topic_;
  bool use_header_stamp_{true};
  double min_dt_{1e-4};

  double time_constant_norm_sec_{0.01};
  double time_constant_dir_sec_{0.01};

  bool has_prev_{false};
  rclcpp::Time t_prev_;
  double norm_filt_{0.0};
  tf2::Vector3 dir_filt_{0.0, 0.0, 1.0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

}  // namespace kuroko_imu
