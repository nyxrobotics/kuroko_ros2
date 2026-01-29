#include <rclcpp/rclcpp.hpp>
#include "imu_angular_velocity_overwriter/imu_angular_velocity_overwriter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kuroko_imu::ImuAngularVelocityOverwriter>());
  rclcpp::shutdown();
  return 0;
}
