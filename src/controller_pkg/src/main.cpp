#include "rclcpp/rclcpp.hpp"
#include "controller_pkg/controller_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<controller_pkg::ControllerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}