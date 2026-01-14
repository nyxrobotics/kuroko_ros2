#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <optional>
#include <limits>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "kuroko_kinematics/kuroko_kinematics.h"
#include "kuroko_odom/odom_utils.hpp"

namespace kuroko_odom
{

struct FootVertexResult
{
  Eigen::Vector3d vertex_in_base = Eigen::Vector3d::Zero();
  Eigen::Vector3d vertex_in_world_rel = Eigen::Vector3d::Zero();  // world axes, base origin
  double z_world_rel = 0.0;
};

struct FootState
{
  Eigen::Vector3d foot_pos_base = Eigen::Vector3d::Zero();
  Eigen::Quaterniond foot_q_base = Eigen::Quaterniond::Identity();

  std::vector<FootVertexResult> vertices;
  int lowest_vertex_index = -1;
  double lowest_z_world_rel = 0.0;

  Eigen::Quaterniond foot_q_world = Eigen::Quaterniond::Identity();
  double tilt_angle_rad = 0.0;
};

class FootOdomNode : public rclcpp::Node
{
public:
  FootOdomNode()
  : Node("foot_odom"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();

    body_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(body_odom_topic_, 10);
    foot_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(foot_odom_topic_, 10);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FootOdomNode::onJointState, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FootOdomNode::onImu, this, std::placeholders::_1));

    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, 10,
      std::bind(&FootOdomNode::onInitialPose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "foot_odom node started.");
  }

private:
  void declareParameters()
  {
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");

    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");

    body_odom_topic_ = declare_parameter<std::string>("body_odom_topic", "/body_odom");
    foot_odom_topic_ = declare_parameter<std::string>("foot_odom_topic", "/foot_odom");

    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.1);

    right_joint_names_ = declare_parameter<std::vector<std::string>>(
      "right_leg_joint_names",
      std::vector<std::string>{
        "hip_r_roll", "hip_r_pitch", "thigh_r_active", "shin_r_active", "ankle_r_roll", "ankle_r_yaw"});

    left_joint_names_ = declare_parameter<std::vector<std::string>>(
      "left_leg_joint_names",
      std::vector<std::string>{
        "hip_l_roll", "hip_l_pitch", "thigh_l_active", "shin_l_active", "ankle_l_roll", "ankle_l_yaw"});

    const std::vector<double> default_fp = {0.06, 0.0375, 0.06, -0.0375, -0.06, -0.0375, -0.06, 0.0375};
    right_footprint_xy_ = declare_parameter<std::vector<double>>("right_footprint_xy", default_fp);
    left_footprint_xy_ = declare_parameter<std::vector<double>>("left_footprint_xy", default_fp);

    stance_up_acc_threshold_ = declare_parameter<double>("stance_up_acc_threshold", 9.81);  // +g
    stance_down_acc_threshold_ = declare_parameter<double>("stance_down_acc_threshold", 0.0);

    cov_trans_min_ = declare_parameter<double>("cov_trans_min", 0.1);
    cov_trans_max_ = declare_parameter<double>("cov_trans_max", 1e6);

    foot_tilt_threshold_rad_ = declare_parameter<double>("foot_tilt_threshold_rad", 0.2);
    cov_rot_min_ = declare_parameter<double>("cov_rot_min", 0.1);
    cov_rot_max_ = declare_parameter<double>("cov_rot_max", 1e6);

    if (right_footprint_xy_.size() < 4 || (right_footprint_xy_.size() % 2 != 0) ||
        left_footprint_xy_.size() < 4 || (left_footprint_xy_.size() % 2 != 0))
    {
      throw std::runtime_error("footprint_xy must be a flattened array with even length >= 4.");
    }
    if (right_joint_names_.size() != 6 || left_joint_names_.size() != 6)
    {
      throw std::runtime_error("right_leg_joint_names and left_leg_joint_names must have 6 entries each.");
    }
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_time_ = msg->header.stamp;
    const auto& q = msg->orientation;
    imu_q_raw_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    imu_q_raw_.normalize();
    imu_received_ = true;
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose_in, pose_odom;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;

    pose_odom = pose_in;

    if (!pose_in.header.frame_id.empty() && pose_in.header.frame_id != odom_frame_id_)
    {
      try
      {
        const auto tf = tf_buffer_.lookupTransform(odom_frame_id_, pose_in.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(pose_in, pose_odom, tf);
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(get_logger(), "initialpose transform failed (%s). Using pose as-is.", e.what());
        pose_odom = pose_in;
        pose_odom.header.frame_id = odom_frame_id_;
      }
    }

    const double desired_x = pose_odom.pose.position.x;
    const double desired_y = pose_odom.pose.position.y;

    tf2::Quaternion q_msg;
    tf2::fromMsg(pose_odom.pose.orientation, q_msg);
    const double desired_yaw = tf2::getYaw(q_msg);

    if (!have_pose_)
    {
      odom_pos_xy_ = Eigen::Vector2d(desired_x, desired_y);
      odom_yaw_ = wrapToPi(desired_yaw);
      have_pose_ = true;
      RCLCPP_INFO(get_logger(), "initialpose applied (no prior pose). x=%.3f y=%.3f yaw=%.3f",
                  desired_x, desired_y, desired_yaw);
      return;
    }

    const double current_foot_yaw = current_foot_yaw_.value_or(odom_yaw_);
    const double delta = wrapToPi(desired_yaw - current_foot_yaw);

    odom_pos_xy_ = Eigen::Vector2d(desired_x, desired_y);
    odom_yaw_ = wrapToPi(odom_yaw_ + delta);

    // Keep roll/pitch (IMU), adjust only yaw offset
    q_offset_yaw_ = yawToQuaternion(delta) * q_offset_yaw_;
    q_offset_yaw_.normalize();
    q_offset_initialized_ = true;

    RCLCPP_INFO(get_logger(), "initialpose applied. x=%.3f y=%.3f desired_yaw=%.3f (delta=%.3f)",
                desired_x, desired_y, desired_yaw, delta);
  }

  std::optional<double> getJoint(const sensor_msgs::msg::JointState& msg, const std::string& name) const
  {
    for (size_t i = 0; i < msg.name.size(); ++i)
    {
      if (msg.name[i] == name)
      {
        if (i < msg.position.size())
          return msg.position[i];
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  bool extractLegJoints(const sensor_msgs::msg::JointState& msg,
                        const std::vector<std::string>& names,
                        std::vector<double>& out) const
  {
    out.resize(6);
    for (size_t i = 0; i < 6; ++i)
    {
      const auto v = getJoint(msg, names[i]);
      if (!v.has_value() || !std::isfinite(v.value()))
        return false;
      out[i] = v.value();
    }
    return true;
  }

  static Eigen::Quaterniond rpyToQuat(double r, double p, double y)
  {
    const Eigen::AngleAxisd ax(r, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ay(p, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd az(y, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = az * ay * ax;
    q.normalize();
    return q;
  }

  FootState computeFootState(const Eigen::Quaterniond& q_base_world,
                             const std::vector<double>& joints,
                             bool is_left)
  {
    FootState fs;

    std::vector<double> pose;
    if (is_left)
      kinematics_.solveForwardKinematicsForLeftLeg(joints, pose);
    else
      kinematics_.solveForwardKinematicsForRightLeg(joints, pose);

    // pose: [x,y,z, roll,pitch,yaw] in base frame
    fs.foot_pos_base = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    fs.foot_q_base = rpyToQuat(pose[3], pose[4], pose[5]);

    const std::vector<double>& fp = is_left ? left_footprint_xy_ : right_footprint_xy_;
    const size_t n = fp.size() / 2;
    fs.vertices.resize(n);

    fs.lowest_vertex_index = -1;
    fs.lowest_z_world_rel = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < n; ++i)
    {
      const Eigen::Vector3d off(fp[2 * i + 0], fp[2 * i + 1], 0.0);

      FootVertexResult vr;
      vr.vertex_in_base = fs.foot_pos_base + (fs.foot_q_base * off);
      vr.vertex_in_world_rel = q_base_world * vr.vertex_in_base;
      vr.z_world_rel = vr.vertex_in_world_rel.z();
      fs.vertices[i] = vr;

      if (vr.z_world_rel < fs.lowest_z_world_rel)
      {
        fs.lowest_z_world_rel = vr.z_world_rel;
        fs.lowest_vertex_index = static_cast<int>(i);
      }
    }

    fs.foot_q_world = q_base_world * fs.foot_q_base;
    fs.foot_q_world.normalize();

    const Eigen::Vector3d foot_z = fs.foot_q_world * Eigen::Vector3d::UnitZ();
    const double c = std::max(-1.0, std::min(1.0, foot_z.dot(Eigen::Vector3d::UnitZ())));
    fs.tilt_angle_rad = std::acos(c);

    return fs;
  }

  bool imuAvailable(const rclcpp::Time& now) const
  {
    if (!imu_received_) return false;
    const double age = (now - last_imu_time_).seconds();
    return (age <= imu_timeout_sec_);
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const rclcpp::Time now = msg->header.stamp;

    if (last_joint_time_.nanoseconds() == 0)
      last_joint_time_ = now;
    const double dt = std::max(1e-6, (now - last_joint_time_).seconds());
    last_joint_time_ = now;

    std::vector<double> rj, lj;
    if (!extractLegJoints(*msg, right_joint_names_, rj) || !extractLegJoints(*msg, left_joint_names_, lj))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Missing required leg joints in /joint_states. Check right_leg_joint_names/left_leg_joint_names.");
      return;
    }

    const bool imu_ok = imuAvailable(now);

    Eigen::Quaterniond q_base_world = Eigen::Quaterniond::Identity();
    double delta_yaw = 0.0;

    if (imu_ok)
    {
      if (!q_offset_initialized_)
      {
        q_offset_yaw_ = Eigen::Quaterniond::Identity();
        q_offset_initialized_ = true;
      }

      // Base orientation matches IMU (plus optional yaw offset from initialpose)
      q_base_world = q_offset_yaw_ * imu_q_raw_;
      q_base_world.normalize();

      const double yaw_now = quaternionToProjectedRollPitchYaw(q_base_world).z();
      if (prev_yaw_valid_)
      {
        delta_yaw = wrapToPi(yaw_now - prev_projected_yaw_);
        odom_yaw_ = wrapToPi(odom_yaw_ + delta_yaw);
      }
      prev_projected_yaw_ = yaw_now;
      prev_yaw_valid_ = true;
    }
    else
    {
      // IMU timeout: treat as upright, yaw update uses stance-foot yaw delta
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "IMU timeout: treating as upright and using stance-foot yaw for odom yaw.");

      q_base_world = yawToQuaternion(odom_yaw_);

      if (prev_leg_joints_.has_value())
      {
        // Use previous stance estimate from previous sample's stance (lowest point at that time is unknown here).
        // For simplicity, approximate with current stance side when IMU is unavailable.
        std::vector<double> pose_prev, pose_now;

        const bool stance_is_left_guess = false;  // will be overwritten below after current stance is computed
        (void)stance_is_left_guess;

        // Delay yaw update until stance is computed below
      }
    }

    // Feet states (for stance selection and base height)
    FootState right = computeFootState(q_base_world, rj, false);
    FootState left  = computeFootState(q_base_world, lj, true);

    const bool stance_is_left = (left.lowest_z_world_rel < right.lowest_z_world_rel);
    const FootState& stance = stance_is_left ? left : right;

    const double min_z = std::min(left.lowest_z_world_rel, right.lowest_z_world_rel);
    const double base_z = -min_z;  // ground z=0 by definition

    // IMU timeout yaw update (after stance is known)
    if (!imu_ok && prev_leg_joints_.has_value())
    {
      std::vector<double> pose_prev, pose_now;

      if (stance_is_left)
      {
        kinematics_.solveForwardKinematicsForLeftLeg(prev_leg_joints_->second, pose_prev);
        kinematics_.solveForwardKinematicsForLeftLeg(lj, pose_now);
      }
      else
      {
        kinematics_.solveForwardKinematicsForRightLeg(prev_leg_joints_->first, pose_prev);
        kinematics_.solveForwardKinematicsForRightLeg(rj, pose_now);
      }

      const double d = wrapToPi(pose_now[5] - pose_prev[5]);
      delta_yaw = wrapToPi(-d);
      odom_yaw_ = wrapToPi(odom_yaw_ + delta_yaw);
      q_base_world = yawToQuaternion(odom_yaw_);
    }

    if (!have_pose_)
    {
      odom_pos_xy_ = Eigen::Vector2d::Zero();
      odom_yaw_ = wrapToPi(quaternionToProjectedRollPitchYaw(q_base_world).z());
      have_pose_ = true;
    }

    // Contact-point displacement of the CURRENT stance vertex (no absolute world contact needed)
    Eigen::Vector3d delta_pos_world = Eigen::Vector3d::Zero();
    if (prev_leg_joints_.has_value() && prev_q_base_world_.has_value())
    {
      const Eigen::Quaterniond q_prev = prev_q_base_world_.value();

      // Recompute previous foot state for the SAME foot as current stance
      FootState prev_same =
        stance_is_left
          ? computeFootState(q_prev, prev_leg_joints_->second, true)
          : computeFootState(q_prev, prev_leg_joints_->first, false);

      const int idx = stance.lowest_vertex_index;

      // Use the same vertex index "as of now" for both previous and current samples
      const Eigen::Vector3d prev_contact_base = prev_same.vertices[idx].vertex_in_base;
      const Eigen::Vector3d now_contact_base  = stance.vertices[idx].vertex_in_base;

      const Eigen::Vector3d prev_contact_world_rel = q_prev * prev_contact_base;
      const Eigen::Vector3d now_contact_world_rel  = q_base_world * now_contact_base;

      const Eigen::Vector3d delta_contact_world = now_contact_world_rel - prev_contact_world_rel;

      // If the contact is fixed in world, base translation is the negative of contact motion.
      delta_pos_world = -delta_contact_world;

      // Apply XY only
      odom_pos_xy_.x() += delta_pos_world.x();
      odom_pos_xy_.y() += delta_pos_world.y();
    }

    // Velocity in current base frame (use world delta, rotate back to base)
    const Eigen::Vector3d v_world = delta_pos_world / dt;
    const Eigen::Vector3d v_base = q_base_world.conjugate() * v_world;
    const double w_z = delta_yaw / dt;

    // Covariance selection
    double cov_trans = cov_trans_max_;
    double cov_rot = cov_rot_max_;

    if (!imu_ok)
    {
      cov_trans = cov_trans_max_;
      cov_rot = cov_rot_max_;
    }
    else
    {
      // Stance foot vertical acceleration (world), derived from stance foot origin
      // Note: this is a heuristic; it does not require absolute world position.
      const Eigen::Vector3d stance_pos_world_rel = q_base_world * stance.foot_pos_base;

      const double z = stance_pos_world_rel.z();
      const double vz = (z - prev_stance_z_world_) / dt;
      const double az = (vz - prev_stance_vz_world_) / dt;

      prev_stance_z_world_ = z;
      prev_stance_vz_world_ = vz;

      cov_trans = lerpClamped(az, stance_down_acc_threshold_, stance_up_acc_threshold_,
                              cov_trans_min_, cov_trans_max_);

      cov_rot = lerpClamped(stance.tilt_angle_rad, 0.0, foot_tilt_threshold_rad_,
                            cov_rot_min_, cov_rot_max_);
    }

    // Update prev states
    base_z_prev_ = base_z;
    prev_q_base_world_ = q_base_world;
    prev_leg_joints_ = std::make_pair(rj, lj);

    publishOdometry(now, q_base_world, base_z, v_base, w_z, cov_trans, cov_rot);
  }

  void publishOdometry(const rclcpp::Time& stamp,
                       const Eigen::Quaterniond& q_base_world,
                       double base_z,
                       const Eigen::Vector3d& v_base,
                       double w_z,
                       double cov_trans,
                       double cov_rot)
  {
    nav_msgs::msg::Odometry body, foot;

    body.header.stamp = stamp;
    body.header.frame_id = odom_frame_id_;
    body.child_frame_id = base_frame_id_;

    body.pose.pose.position.x = odom_pos_xy_.x();
    body.pose.pose.position.y = odom_pos_xy_.y();
    body.pose.pose.position.z = base_z;

    body.pose.pose.orientation = tf2::toMsg(
      tf2::Quaternion(q_base_world.x(), q_base_world.y(), q_base_world.z(), q_base_world.w()));

    // /foot_odom: horizontal pose, yaw from quaternionToProjectedRollPitchYaw()
    const double yaw_proj = quaternionToProjectedRollPitchYaw(q_base_world).z();
    current_foot_yaw_ = yaw_proj;

    const Eigen::Quaterniond q_foot = projectedRollPitchYawToQuaternion(Eigen::Vector3d(0.0, 0.0, yaw_proj));

    foot.header = body.header;
    foot.child_frame_id = base_frame_id_;
    foot.pose.pose.position = body.pose.pose.position;
    foot.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(q_foot.x(), q_foot.y(), q_foot.z(), q_foot.w()));

    // Twist: expressed in child (base) frame
    body.twist.twist.linear.x = v_base.x();
    body.twist.twist.linear.y = v_base.y();
    body.twist.twist.linear.z = 0.0;

    body.twist.twist.angular.x = 0.0;
    body.twist.twist.angular.y = 0.0;
    body.twist.twist.angular.z = w_z;

    foot.twist = body.twist;

    // Covariance (simple diagonal)
    for (auto* cov : {&body.pose.covariance, &foot.pose.covariance})
    {
      cov->fill(0.0);
      (*cov)[0]  = cov_trans;
      (*cov)[7]  = cov_trans;
      (*cov)[14] = cov_trans;
      (*cov)[21] = cov_rot;
      (*cov)[28] = cov_rot;
      (*cov)[35] = cov_rot;
    }
    for (auto* cov : {&body.twist.covariance, &foot.twist.covariance})
    {
      cov->fill(0.0);
      (*cov)[0]  = cov_trans;
      (*cov)[7]  = cov_trans;
      (*cov)[14] = cov_trans;
      (*cov)[21] = cov_rot;
      (*cov)[28] = cov_rot;
      (*cov)[35] = cov_rot;
    }

    body_odom_pub_->publish(body);
    foot_odom_pub_->publish(foot);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr foot_odom_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Params
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string joint_states_topic_;
  std::string imu_topic_;
  std::string initialpose_topic_;
  std::string body_odom_topic_;
  std::string foot_odom_topic_;

  double imu_timeout_sec_ = 0.1;

  std::vector<std::string> right_joint_names_;
  std::vector<std::string> left_joint_names_;
  std::vector<double> right_footprint_xy_;
  std::vector<double> left_footprint_xy_;

  double stance_up_acc_threshold_ = 9.81;
  double stance_down_acc_threshold_ = 0.0;
  double cov_trans_min_ = 0.1;
  double cov_trans_max_ = 1e6;

  double foot_tilt_threshold_rad_ = 0.2;
  double cov_rot_min_ = 0.1;
  double cov_rot_max_ = 1e6;

  // State
  motion_control::KurokoKinematics kinematics_;

  bool imu_received_ = false;
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  Eigen::Quaterniond imu_q_raw_ = Eigen::Quaterniond::Identity();

  rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};

  bool have_pose_ = false;
  Eigen::Vector2d odom_pos_xy_ = Eigen::Vector2d::Zero();
  double odom_yaw_ = 0.0;

  bool q_offset_initialized_ = false;
  Eigen::Quaterniond q_offset_yaw_ = Eigen::Quaterniond::Identity();

  bool prev_yaw_valid_ = false;
  double prev_projected_yaw_ = 0.0;

  double base_z_prev_ = 0.0;
  std::optional<Eigen::Quaterniond> prev_q_base_world_;
  std::optional<std::pair<std::vector<double>, std::vector<double>>> prev_leg_joints_;

  double prev_stance_z_world_ = 0.0;
  double prev_stance_vz_world_ = 0.0;

  std::optional<double> current_foot_yaw_;
};

}  // namespace kuroko_odom

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kuroko_odom::FootOdomNode>());
  rclcpp::shutdown();
  return 0;
}
