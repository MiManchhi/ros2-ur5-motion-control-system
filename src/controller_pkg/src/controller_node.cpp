#include "controller_pkg/controller_node.hpp"

#include <utility>

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller_node", options)
{
  // =========================
  // 1. 声明参数
  // =========================
  this->declare_parameter<std::string>("planned_traj_topic", "/planned_traj");
  this->declare_parameter<std::string>("joint_cmd_topic", "/joint_cmd");

  // =========================
  // 2. 读取参数
  // =========================
  planned_traj_topic_ = this->get_parameter("planned_traj_topic").as_string();
  joint_cmd_topic_ = this->get_parameter("joint_cmd_topic").as_string();

  // =========================
  // 3. 初始化订阅器和发布器
  // =========================
  planned_traj_sub_ =
    this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    planned_traj_topic_,
    10,
    std::bind(&ControllerNode::planned_traj_callback, this, std::placeholders::_1));

  joint_cmd_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_cmd_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "ControllerNode started.");
  RCLCPP_INFO(this->get_logger(), "Planned trajectory topic: %s", planned_traj_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Joint command topic: %s", joint_cmd_topic_.c_str());
}

void ControllerNode::planned_traj_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received planned trajectory: joint_count=%zu, point_count=%zu",
    msg->joint_names.size(),
    msg->points.size());

  // 阶段2中先由执行器做简单转换（当前为透传）
  auto joint_cmd = executor_.convert_to_command(*msg);

  // 发布到下游 joint_cmd
  joint_cmd_pub_->publish(joint_cmd);

  RCLCPP_INFO(
    this->get_logger(),
    "Published joint command: joint_count=%zu, point_count=%zu",
    joint_cmd.joint_names.size(),
    joint_cmd.points.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}