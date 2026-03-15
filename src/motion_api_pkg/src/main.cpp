#include "motion_api_pkg/motion_api_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motion_api_pkg::MotionApiNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}