#include "rclcpp/rclcpp.hpp"
#include "system_manager_pkg/system_manager_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<system_manager_pkg::SystemManagerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}