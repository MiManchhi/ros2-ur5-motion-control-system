#include "rclcpp/rclcpp.hpp"
#include "robot_interface_pkg/robot_interface_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<robot_interface_pkg::RobotInterfaceNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}