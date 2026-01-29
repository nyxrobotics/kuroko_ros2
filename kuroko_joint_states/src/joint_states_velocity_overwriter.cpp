#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <unordered_map>
#include <string>

class JointStatesVelocityOverwriter : public rclcpp::Node
{
public:
  JointStatesVelocityOverwriter()
  : Node("joint_states_velocity_overwriter")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/joint_states");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/joint_states_vel");
    use_header_stamp_ = this->declare_parameter<bool>("use_header_stamp", true);
    zero_effort_ = this->declare_parameter<bool>("zero_effort", false);

    auto qos = rclcpp::SensorDataQoS();

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, qos,
      std::bind(&JointStatesVelocityOverwriter::onJointState, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Subscribed: %s | Publishing: %s | use_header_stamp=%s | zero_effort=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      use_header_stamp_ ? "true" : "false",
      zero_effort_ ? "true" : "false");
  }

private:
  struct PrevState
  {
    double position = 0.0;
    bool has_position = false;
  };

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    rclcpp::Time current_time;
    if (use_header_stamp_) {
      current_time = rclcpp::Time(msg->header.stamp);
    } else {
      current_time = this->now();
    }

    double dt = 0.0;
    if (has_prev_time_) {
      dt = (current_time - prev_time_).seconds();
    }
    prev_time_ = current_time;
    has_prev_time_ = true;

    sensor_msgs::msg::JointState out = *msg;

    const size_t n = out.name.size();

    if (out.position.size() != n) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "JointState size mismatch: name=%zu position=%zu. Publishing without velocity overwrite.",
        n, out.position.size());

      applyEffortPolicy(out);
      pub_->publish(out);
      return;
    }

    out.velocity.assign(n, 0.0);

    if (dt <= 0.0) {
      applyEffortPolicy(out);
      updatePrevPositions(out);
      pub_->publish(out);
      return;
    }

    for (size_t i = 0; i < n; ++i) {
      const std::string & joint_name = out.name[i];
      const double pos = out.position[i];

      auto & prev = prev_positions_[joint_name];
      if (prev.has_position) {
        out.velocity[i] = (pos - prev.position) / dt;
      } else {
        out.velocity[i] = 0.0;
      }

      prev.position = pos;
      prev.has_position = true;
    }

    applyEffortPolicy(out);
    pub_->publish(out);
  }

  void applyEffortPolicy(sensor_msgs::msg::JointState & out)
  {
    const size_t n = out.name.size();
    if (zero_effort_) {
      out.effort.assign(n, 0.0);
      return;
    }

    if (!out.effort.empty() && out.effort.size() != n) {
      out.effort.resize(n, 0.0);
    }
  }

  void updatePrevPositions(const sensor_msgs::msg::JointState & msg)
  {
    const size_t n = msg.name.size();
    for (size_t i = 0; i < n; ++i) {
      const std::string & joint_name = msg.name[i];
      const double pos = msg.position[i];
      auto & prev = prev_positions_[joint_name];
      prev.position = pos;
      prev.has_position = true;
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  bool use_header_stamp_;
  bool zero_effort_;

  std::unordered_map<std::string, PrevState> prev_positions_;

  rclcpp::Time prev_time_{0, 0, RCL_ROS_TIME};
  bool has_prev_time_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatesVelocityOverwriter>());
  rclcpp::shutdown();
  return 0;
}
