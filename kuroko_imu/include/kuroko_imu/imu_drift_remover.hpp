#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>
#include <vector>

namespace kuroko_imu
{

class ImuDriftRemover : public rclcpp::Node
{
public:
  explicit ImuDriftRemover(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  // NOTE: onJoint must NOT do bias estimation. It only updates joint motion state and threshold result.
  void onJoint(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Helpers
  static double l2Norm3(double x, double y, double z);
  static double clamp(double v, double lo, double hi);
  void publishImuCorrected(const sensor_msgs::msg::Imu & in);

  // Topics
  std::string imu_topic_;
  std::string joint_topic_;
  std::string output_topic_;

  // Thresholds / parameters
  double joint_vel_norm_thresh_{0.02};      // rad/s (L2 of joint velocity vector)
  double gyro_norm_thresh_{0.03};           // rad/s (L2)
  double accel_diff_norm_thresh_{0.5};      // (m/s^2)/s (L2 of accel derivative)
  double stable_time_sec_{1.5};             // seconds
  double bias_time_constant_sec_{10.0};     // seconds
  bool publish_debug_{false};
  bool print_internal_state_{false};

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_out_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_bias_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stationary_;

  // Joint tracking (updated only in onJoint)
  bool has_prev_joint_{false};
  bool joint_ready_{false};
  rclcpp::Time prev_joint_time_;
  std::vector<double> prev_joint_pos_;
  bool joint_stationary_{false};            // threshold decision made in onJoint
  double joint_vel_norm_{1e9};              // optional (debug)

  // IMU tracking (used in onImu)
  bool has_prev_imu_{false};
  rclcpp::Time prev_imu_time_;
  double prev_ax_{0.0}, prev_ay_{0.0}, prev_az_{0.0};

  // Bias estimate (updated only in onImu)
  double bias_gx_{0.0}, bias_gy_{0.0}, bias_gz_{0.0};

  // Stationary accumulation (updated only in onImu)
  double stationary_accum_sec_{0.0};
};

}  // namespace kuroko_imu
