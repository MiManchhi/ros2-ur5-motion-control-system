#include "robot_interface_pkg/robot_interface_node.hpp"

#include <utility>

RobotInterfaceNode::RobotInterfaceNode(const rclcpp::NodeOptions & options)
: Node("robot_interface_node", options)
{
  // =========================
  // 1. 声明参数
  // =========================
  this->declare_parameter<std::string>("joint_cmd_topic", "/joint_cmd");
  this->declare_parameter<std::string>(
    "sim_command_topic",
    "/joint_trajectory_controller/joint_trajectory");

  // =========================
  // 2. 读取参数
  // =========================
  joint_cmd_topic_ = this->get_parameter("joint_cmd_topic").as_string();
  sim_command_topic_ = this->get_parameter("sim_command_topic").as_string();

  // =========================
  // 3. 初始化订阅器和发布器
  // =========================
  joint_cmd_sub_ =
    this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    joint_cmd_topic_,
    10,
    std::bind(&RobotInterfaceNode::joint_cmd_callback, this, std::placeholders::_1));

  sim_command_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(sim_command_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "RobotInterfaceNode started.");
  RCLCPP_INFO(this->get_logger(), "Input joint_cmd topic: %s", joint_cmd_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output sim command topic: %s", sim_command_topic_.c_str());
}

void RobotInterfaceNode::joint_cmd_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received joint command: joint_count=%zu, point_count=%zu",
    msg->joint_names.size(),
    msg->points.size());

  // 阶段2中先通过后端做简单透传
  auto sim_cmd = backend_.convert_to_sim_command(*msg);

  // 发布到 Gazebo / ros2_control 控制器
  sim_command_pub_->publish(sim_cmd);

  RCLCPP_INFO(
    this->get_logger(),
    "Published sim command to controller: joint_count=%zu, point_count=%zu",
    sim_cmd.joint_names.size(),
    sim_cmd.points.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}