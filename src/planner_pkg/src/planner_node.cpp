#include "planner_pkg/planner_node.hpp"

#include <utility>

PlannerNode::PlannerNode(const rclcpp::NodeOptions & options)
: Node("planner_node", options)
{
  this->declare_parameter<std::string>("input_topic", "/motion_command");
  this->declare_parameter<std::string>("output_topic", "/planned_traj");
  this->declare_parameter<double>("default_duration_sec", 3.0);
  this->declare_parameter<int>("interpolation_points", 10);

  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  default_duration_sec_ = this->get_parameter("default_duration_sec").as_double();
  interpolation_points_ = this->get_parameter("interpolation_points").as_int();

  motion_command_sub_ =
    this->create_subscription<robot_motion_msgs::msg::MotionCommand>(
    input_topic_,
    10,
    std::bind(&PlannerNode::motion_command_callback, this, std::placeholders::_1));

  planned_traj_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(output_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "PlannerNode started.");
  RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
}

void PlannerNode::motion_command_callback(
  const robot_motion_msgs::msg::MotionCommand::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received motion command: task_id=%s, joint_count=%zu",
    msg->task_id.c_str(),
    msg->joint_names.size());

  try {
    auto traj = planner_.plan(
      msg->task_id,
      msg->joint_names,
      msg->positions,
      default_duration_sec_,
      interpolation_points_);

    planned_traj_pub_->publish(traj);

    RCLCPP_INFO(
      this->get_logger(),
      "Published planned trajectory: task_id=%s, points=%zu",
      msg->task_id.c_str(),
      traj.points.size());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Planning failed for task_id=%s: %s",
      msg->task_id.c_str(),
      e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}