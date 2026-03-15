#include "planner_pkg/planner_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<planner_pkg::PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}