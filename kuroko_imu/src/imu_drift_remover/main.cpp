#include <rclcpp/rclcpp.hpp>
#include "imu_drift_remover/imu_drift_remover.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kuroko_imu::ImuDriftRemover>());
  rclcpp::shutdown();
  return 0;
}
