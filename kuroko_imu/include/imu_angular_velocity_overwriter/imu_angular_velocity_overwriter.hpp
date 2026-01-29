#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace kuroko_imu
{

class ImuAngularVelocityOverwriter : public rclcpp::Node
{
public:
  explicit ImuAngularVelocityOverwriter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg_in);
  static tf2::Quaternion toQuat(const geometry_msgs::msg::Quaternion & q);

  std::string input_topic_;
  std::string output_topic_;
  bool use_header_stamp_{true};
  double min_dt_{1e-4};
  bool zero_on_first_msg_{true};

  bool has_prev_{false};
  tf2::Quaternion q_prev_;
  rclcpp::Time t_prev_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

}  // namespace kuroko_imu
