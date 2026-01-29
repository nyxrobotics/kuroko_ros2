#include "imu_drift_remover/imu_drift_remover.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace kuroko_imu
{

ImuDriftRemover::ImuDriftRemover(const rclcpp::NodeOptions & options)
: Node("imu_drift_remover", options)
{
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu_data");
  joint_topic_ = declare_parameter<std::string>("joint_topic", "/joint_states");
  output_topic_ = declare_parameter<std::string>("output_topic", "/imu_drift");

  joint_vel_norm_thresh_ = declare_parameter<double>("joint_vel_norm_thresh", joint_vel_norm_thresh_);
  gyro_norm_thresh_ = declare_parameter<double>("gyro_norm_thresh", gyro_norm_thresh_);
  accel_diff_norm_thresh_ = declare_parameter<double>("accel_diff_norm_thresh", accel_diff_norm_thresh_);

  stable_time_sec_ = declare_parameter<double>("stable_time_sec", stable_time_sec_);
  bias_time_constant_sec_ = declare_parameter<double>("bias_time_constant_sec", bias_time_constant_sec_);

  publish_debug_ = declare_parameter<bool>("publish_debug", publish_debug_);
  print_internal_state_ = declare_parameter<bool>("print_internal_state", print_internal_state_);

  pub_imu_out_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());

  if (publish_debug_) {
    pub_bias_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_ + std::string("/bias"), 10);
    pub_stationary_ = create_publisher<std_msgs::msg::Bool>(output_topic_ + std::string("/stationary"), 10);
  }

  sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_topic_, 10,
    std::bind(&ImuDriftRemover::onJoint, this, std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ImuDriftRemover::onImu, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "imu_drift_remover started. imu: %s joint: %s -> out: %s",
    imu_topic_.c_str(), joint_topic_.c_str(), output_topic_.c_str());
}

double ImuDriftRemover::l2Norm3(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

double ImuDriftRemover::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

// Joint callback: compute joint motion metric and decide threshold here.
// Bias estimation must NOT be done here.
void ImuDriftRemover::onJoint(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.empty()) {
    joint_ready_ = false;
    joint_stationary_ = false;
    return;
  }

  rclcpp::Time t = msg->header.stamp;
  if (t.nanoseconds() == 0) {
    t = now();
  }

  if (!has_prev_joint_) {
    prev_joint_time_ = t;
    prev_joint_pos_ = msg->position;
    has_prev_joint_ = true;
    joint_ready_ = false;
    joint_stationary_ = false;
    return;
  }

  const double dt = (t - prev_joint_time_).seconds();
  if (dt <= 0.0) {
    joint_ready_ = false;
    joint_stationary_ = false;
    return;
  }

  if (prev_joint_pos_.size() != msg->position.size()) {
    prev_joint_pos_ = msg->position;
    prev_joint_time_ = t;
    joint_ready_ = false;
    joint_stationary_ = false;
    return;
  }

  double sum_sq = 0.0;
  for (size_t i = 0; i < msg->position.size(); ++i) {
    const double v = (msg->position[i] - prev_joint_pos_[i]) / dt;
    sum_sq += v * v;
  }
  joint_vel_norm_ = std::sqrt(sum_sq);

  // Threshold decision is made HERE
  joint_stationary_ = (joint_vel_norm_ <= joint_vel_norm_thresh_);

  prev_joint_pos_ = msg->position;
  prev_joint_time_ = t;
  joint_ready_ = true;
}

void ImuDriftRemover::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  rclcpp::Time t = msg->header.stamp;
  if (t.nanoseconds() == 0) {
    t = now();
  }

  // dt handling
  if (!has_prev_imu_) {
    prev_imu_time_ = t;
    prev_ax_ = msg->linear_acceleration.x;
    prev_ay_ = msg->linear_acceleration.y;
    prev_az_ = msg->linear_acceleration.z;
    has_prev_imu_ = true;

    publishImuCorrected(*msg);
    return;
  }

  const double dt = (t - prev_imu_time_).seconds();
  if (dt <= 0.0) {
    publishImuCorrected(*msg);
    return;
  }

  // Read IMU
  const double gx = msg->angular_velocity.x;
  const double gy = msg->angular_velocity.y;
  const double gz = msg->angular_velocity.z;
  const double gyro_norm = l2Norm3(gx, gy, gz);

  const double ax = msg->linear_acceleration.x;
  const double ay = msg->linear_acceleration.y;
  const double az = msg->linear_acceleration.z;

  // Accel time derivative (finite diff)
  const double dax = (ax - prev_ax_) / dt;
  const double day = (ay - prev_ay_) / dt;
  const double daz = (az - prev_az_) / dt;
  const double accel_diff_norm = l2Norm3(dax, day, daz);

  prev_ax_ = ax;
  prev_ay_ = ay;
  prev_az_ = az;
  prev_imu_time_ = t;

  // Stationary decision uses the latest joint threshold result computed in onJoint.
  const bool joint_ok = joint_ready_ && joint_stationary_;
  const bool gyro_ok = (gyro_norm <= gyro_norm_thresh_);
  const bool accel_ok = (accel_diff_norm <= accel_diff_norm_thresh_);
  const bool stationary_now = joint_ok && gyro_ok && accel_ok;

  if (stationary_now) {
    stationary_accum_sec_ += dt;
  } else {
    stationary_accum_sec_ = 0.0;
  }

  const bool stationary_confirmed = stationary_accum_sec_ >= stable_time_sec_;

  // Bias update ONLY here (IMU callback)
  if (stationary_confirmed) {
    const double tau = std::max(1e-6, bias_time_constant_sec_);
    const double alpha = clamp(dt / tau, 0.0, 1.0);

    // First-order low-pass toward current gyro (assumed drift when stationary)
    bias_gx_ += alpha * (gx - bias_gx_);
    bias_gy_ += alpha * (gy - bias_gy_);
    bias_gz_ += alpha * (gz - bias_gz_);
  }

  publishImuCorrected(*msg);
  if (print_internal_state_) {
    RCLCPP_INFO(
      get_logger(),
      "[imu_drift_remover] joint_vel_norm: %.6f (th %.6f) | gyro_norm: %.6f (th %.6f) | accel_diff: %.6f (th %.6f) | stat_now: %d | stat_t: %.2f/%.2f | bias: [%.5f %.5f %.5f]",
      joint_vel_norm_, joint_vel_norm_thresh_,
      gyro_norm, gyro_norm_thresh_,
      accel_diff_norm, accel_diff_norm_thresh_,
      stationary_now,
      stationary_accum_sec_, stable_time_sec_,
      bias_gx_, bias_gy_, bias_gz_
    );
  }
  if (publish_debug_) {
    std_msgs::msg::Bool s;
    s.data = stationary_confirmed;
    pub_stationary_->publish(s);

    sensor_msgs::msg::Imu bias_msg;
    bias_msg.header = msg->header;
    bias_msg.angular_velocity.x = bias_gx_;
    bias_msg.angular_velocity.y = bias_gy_;
    bias_msg.angular_velocity.z = bias_gz_;
    pub_bias_->publish(bias_msg);
  }
}

void ImuDriftRemover::publishImuCorrected(const sensor_msgs::msg::Imu & in)
{
  sensor_msgs::msg::Imu out = in;
  out.angular_velocity.x = in.angular_velocity.x - bias_gx_;
  out.angular_velocity.y = in.angular_velocity.y - bias_gy_;
  out.angular_velocity.z = in.angular_velocity.z - bias_gz_;
  pub_imu_out_->publish(out);
}

}  // namespace kuroko_imu

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kuroko_imu::ImuDriftRemover)
