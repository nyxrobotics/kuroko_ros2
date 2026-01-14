#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

namespace kuroko_odom
{

class OdomTfNode : public rclcpp::Node
{
public:
  OdomTfNode()
  : Node("odom_tf"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/body_odom");
    publish_body_as_child_ = declare_parameter<bool>("publish_body_as_child", true);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&OdomTfNode::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "odom_tf node started. odom_topic=%s publish_body_as_child=%s",
      odom_topic_.c_str(), publish_body_as_child_ ? "true" : "false");
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg->header;

    const std::string parent = msg->header.frame_id;
    const std::string child  = msg->child_frame_id;

    if (publish_body_as_child_)
    {
      tf.header.frame_id = parent;
      tf.child_frame_id = child;

      tf.transform.translation.x = msg->pose.pose.position.x;
      tf.transform.translation.y = msg->pose.pose.position.y;
      tf.transform.translation.z = msg->pose.pose.position.z;
      tf.transform.rotation = msg->pose.pose.orientation;
    }
    else
    {
      // Publish inverse: parent=body_link, child=odom (same spatial relationship).
      tf.header.frame_id = child;
      tf.child_frame_id = parent;

      tf2::Transform T;
      tf2::fromMsg(msg->pose.pose, T);
      tf2::Transform T_inv = T.inverse();
      tf.transform = tf2::toMsg(T_inv);
    }

    tf_broadcaster_->sendTransform(tf);
  }

private:
  std::string odom_topic_;
  bool publish_body_as_child_ = true;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace kuroko_odom

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kuroko_odom::OdomTfNode>());
  rclcpp::shutdown();
  return 0;
}
