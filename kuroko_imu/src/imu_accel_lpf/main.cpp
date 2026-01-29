#include <rclcpp/rclcpp.hpp>

#include "imu_accel_lpf/imu_accel_lpf.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kuroko_imu::ImuAccelLpf>());
  rclcpp::shutdown();
  return 0;
}
