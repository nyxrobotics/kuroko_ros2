#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Dense>

#include "kuroko_kinematics/kuroko_kinematics.h"

namespace
{

double angular_distance(double a, double b)
{
  // Smallest signed difference between angles (rad)
  return std::remainder(a - b, 2.0 * M_PI);
}

std::unordered_map<std::string, double> make_name_to_position_map(
  const sensor_msgs::msg::JointState & msg)
{
  std::unordered_map<std::string, double> m;
  m.reserve(msg.name.size());
  const size_t n = std::min(msg.name.size(), msg.position.size());
  for (size_t i = 0; i < n; ++i) {
    m[msg.name[i]] = msg.position[i];
  }
  return m;
}

struct LegConfig
{
  std::string label;
  // The kinematics library expects this order:
  // [0] hip_roll, [1] hip_pitch, [2] thigh_pitch, [3] shin_active, [4] ankle_roll, [5] ankle_yaw
  std::vector<std::string> joint_names;
};

Eigen::Quaterniond rpy_to_quat(double roll, double pitch, double yaw)
{
  // RPY means X then Y then Z in the existing kinematics code.
  const Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd y(yaw, Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(y * p * r);
}

geometry_msgs::msg::PoseStamped to_pose_stamped(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::vector<double> & pose)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = frame_id;
  out.header.stamp = stamp;

  out.pose.position.x = pose[0];
  out.pose.position.y = pose[1];
  out.pose.position.z = pose[2];

  const Eigen::Quaterniond q = rpy_to_quat(pose[3], pose[4], pose[5]);
  out.pose.orientation.x = q.x();
  out.pose.orientation.y = q.y();
  out.pose.orientation.z = q.z();
  out.pose.orientation.w = q.w();
  return out;
}

}  // namespace

namespace motion_control
{

class FkIkConsistencyCheckerNode : public rclcpp::Node
{
public:
  FkIkConsistencyCheckerNode()
  : Node("fk_ik_consistency_checker"), kin_()
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    tolerance_rad_ = this->declare_parameter<double>("tolerance_rad", 1e-3);
    warn_tolerance_rad_ = this->declare_parameter<double>("warn_tolerance_rad", 5e-3);
    check_side_ = this->declare_parameter<std::string>("side", "both");  // right|left|both
    log_every_n_ = this->declare_parameter<int>("log_every_n", 50);
    publish_fk_pose_ = this->declare_parameter<bool>("publish_fk_pose", true);

    right_leg_ = {
      "right",
      {
        "hip_r_roll",
        "hip_r_pitch",
        "thigh_r_active",
        "shin_r_active",
        "ankle_r_roll",
        "ankle_r_yaw",
      }
    };

    left_leg_ = {
      "left",
      {
        "hip_l_roll",
        "hip_l_pitch",
        "thigh_l_active",
        "shin_l_active",
        "ankle_l_roll",
        "ankle_l_yaw",
      }
    };

    fk_right_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/fk/right_foot_pose", rclcpp::SystemDefaultsQoS());
    fk_left_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/fk/left_foot_pose", rclcpp::SystemDefaultsQoS());

    // Publish IK results per leg (separate topics)
    ik_right_joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/ik/right_joint_states", rclcpp::SystemDefaultsQoS());
    ik_left_joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/ik/left_joint_states", rclcpp::SystemDefaultsQoS());

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&FkIkConsistencyCheckerNode::on_joint_states, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "FK->IK consistency checker started. side=%s, base_frame=%s, tolerance_rad=%.6f (warn=%.6f).",
      check_side_.c_str(), base_frame_.c_str(), tolerance_rad_, warn_tolerance_rad_);
  }

private:
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const auto name_to_pos = make_name_to_position_map(*msg);
    const rclcpp::Time stamp = msg->header.stamp;

    bool did_any = false;
    if (check_side_ == "right" || check_side_ == "both") {
      did_any |= check_leg(right_leg_, stamp, name_to_pos);
    }
    if (check_side_ == "left" || check_side_ == "both") {
      did_any |= check_leg(left_leg_, stamp, name_to_pos);
    }

    if (!did_any) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No configured leg joints found in /joint_states (side=%s).", check_side_.c_str());
    }
  }

  bool check_leg(
    const LegConfig & cfg,
    const rclcpp::Time & stamp,
    const std::unordered_map<std::string, double> & name_to_pos)
  {
    // Gather input joints in the order expected by kinematics
    std::vector<double> q_in(6, 0.0);
    for (size_t i = 0; i < 6; ++i) {
      auto it = name_to_pos.find(cfg.joint_names[i]);
      if (it == name_to_pos.end()) {
        return false;
      }
      q_in[i] = it->second;
      if (!std::isfinite(q_in[i])) {
        return false;
      }
    }

    // FK
    std::vector<double> pose;
    bool fk_ok = false;
    if (cfg.label == "right") {
      fk_ok = kin_.solveForwardKinematicsForRightLeg(q_in, pose);
    } else {
      fk_ok = kin_.solveForwardKinematicsForLeftLeg(q_in, pose);
    }
    if (!fk_ok || pose.size() < 6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "FK failed for %s leg.", cfg.label.c_str());
      return true;
    }

    // IK
    std::vector<double> q_out;
    bool ik_ok = false;
    if (cfg.label == "right") {
      ik_ok = kin_.solveInverseKinematicsForRightLeg(q_out, pose);
    } else {
      ik_ok = kin_.solveInverseKinematicsForLeftLeg(q_out, pose);
    }
    if (!ik_ok || q_out.size() < 6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "IK failed for %s leg.", cfg.label.c_str());
      return true;
    }

    publish_outputs(cfg, stamp, pose, q_out);

    // Compare IK result with original input
    double max_abs_err = 0.0;
    size_t worst_i = 0;
    for (size_t i = 0; i < 6; ++i) {
      const double e = std::fabs(angular_distance(q_out[i], q_in[i]));
      if (e > max_abs_err) {
        max_abs_err = e;
        worst_i = i;
      }
    }

    const bool pass = (max_abs_err <= tolerance_rad_);
    const bool warn = (max_abs_err > warn_tolerance_rad_);
    const auto & worst_name = cfg.joint_names[worst_i];

    if (warn) {
      RCLCPP_WARN(get_logger(),
        "[%s] FK->IK mismatch: max|err|=%.6f rad at %s (in=%.6f, out=%.6f)",
        cfg.label.c_str(), max_abs_err, worst_name.c_str(), q_in[worst_i], q_out[worst_i]);
    } else if (!pass) {
      RCLCPP_INFO(get_logger(),
        "[%s] FK->IK near-miss: max|err|=%.6f rad at %s (in=%.6f, out=%.6f)",
        cfg.label.c_str(), max_abs_err, worst_name.c_str(), q_in[worst_i], q_out[worst_i]);
    } else {
      const int count = ++ok_counter_[cfg.label];
      if (log_every_n_ > 0 && (count % log_every_n_) == 0) {
        RCLCPP_INFO(get_logger(), "[%s] OK (max|err|=%.6f rad)", cfg.label.c_str(), max_abs_err);
      }
    }

    return true;
  }

  void publish_outputs(
    const LegConfig & cfg,
    const rclcpp::Time & stamp,
    const std::vector<double> & pose,
    const std::vector<double> & q_out)
  {
    if (publish_fk_pose_) {
      const auto pose_msg = to_pose_stamped(base_frame_, stamp, pose);
      if (cfg.label == "right") {
        fk_right_pose_pub_->publish(pose_msg);
      } else {
        fk_left_pose_pub_->publish(pose_msg);
      }
    }

    // Publish IK results as JointState so you can echo/record/compare easily.
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = cfg.joint_names;
    js.position.resize(6);
    for (size_t i = 0; i < 6; ++i) {
      js.position[i] = q_out[i];
    }

    // Split topics by leg:
    //  - right: /ik/right_joint_states
    //  - left : /ik/left_joint_states
    if (cfg.label == "right") {
      ik_right_joint_states_pub_->publish(js);
    } else {
      ik_left_joint_states_pub_->publish(js);
    }
  }

private:
  KurokoKinematics kin_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fk_right_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fk_left_pose_pub_;

  // Separate IK topics for right/left
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ik_right_joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ik_left_joint_states_pub_;

  std::string base_frame_{"base_link"};
  bool publish_fk_pose_{true};

  double tolerance_rad_{1e-3};
  double warn_tolerance_rad_{5e-3};
  std::string check_side_{"both"};
  int log_every_n_{50};

  LegConfig right_leg_;
  LegConfig left_leg_;

  std::unordered_map<std::string, int> ok_counter_;
};

}  // namespace motion_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motion_control::FkIkConsistencyCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
