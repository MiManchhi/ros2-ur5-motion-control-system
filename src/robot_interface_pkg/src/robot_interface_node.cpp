#include "robot_interface_pkg/robot_interface_node.hpp"

namespace robot_interface_pkg
{

RobotInterfaceNode::RobotInterfaceNode(const rclcpp::NodeOptions & options)
: Node("robot_interface_node", options)
{
  // =========================
  // 声明并读取参数
  // =========================
  this->declare_parameter<std::string>(
    "controller_topic",
    "/joint_trajectory_controller/joint_trajectory");
  this->declare_parameter<double>("point_time_from_start_sec", 0.1);

  UrGazeboBackend::Config config;
  this->get_parameter("controller_topic", config.controller_topic);
  this->get_parameter("point_time_from_start_sec", config.point_time_from_start_sec);

  backend_.set_config(config);

  // =========================
  // 创建订阅器
  // =========================
  joint_cmd_sub_ =
    this->create_subscription<robot_motion_msgs::msg::MotionCommand>(
      "/joint_cmd",
      10,
      std::bind(&RobotInterfaceNode::on_joint_command, this, std::placeholders::_1));

  // =========================
  // 创建发布器
  // =========================
  controller_cmd_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      backend_.controller_topic(),
      10);

  system_state_pub_ =
    this->create_publisher<robot_motion_msgs::msg::SystemState>(
      "/system_state_raw",
      10);

  RCLCPP_INFO(this->get_logger(), "robot_interface_node 已启动");
  RCLCPP_INFO(
    this->get_logger(),
    "底层控制器话题：%s",
    backend_.controller_topic().c_str());
}

void RobotInterfaceNode::on_joint_command(
  const robot_motion_msgs::msg::MotionCommand::SharedPtr msg)
{
  const std::string & task_id = msg->task_id;

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 收到 /joint_cmd，准备适配到底层控制器",
    task_id.c_str());

  trajectory_msgs::msg::JointTrajectory traj;
  std::string error_msg;
  if (!backend_.convert_command(*msg, traj, error_msg)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 命令转换失败：%s",
      task_id.c_str(),
      error_msg.c_str());

    publish_system_state(task_id, "ERROR", error_msg, true);
    return;
  }

  controller_cmd_pub_->publish(traj);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 已发布到底层控制器：%s，关节数=%zu",
    task_id.c_str(),
    backend_.controller_topic().c_str(),
    traj.joint_names.size());
}

void RobotInterfaceNode::publish_system_state(
  const std::string & task_id,
  const std::string & state,
  const std::string & message,
  bool is_error)
{
  robot_motion_msgs::msg::SystemState msg;
  msg.task_id = task_id;
  msg.state = state;
  msg.message = message;
  msg.is_error = is_error;
  msg.stamp = this->now();

  system_state_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 发布 /system_state：state=%s, message=%s, is_error=%s",
    task_id.c_str(),
    state.c_str(),
    message.c_str(),
    is_error ? "true" : "false");
}

}  // namespace robot_interface_pkg

// 如果需要组件化加载节点，则取消下面的注释，并在 CMakeLists.txt 中添加相关配置
// 组件化加载不需要main函数，直接由 rclcpp_components 管理
// 如果不需要组件化加载，则可以在 main.cpp 中直接创建并运行 MotionApiNode 实例

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robot_interface_pkg::RobotInterfaceNode)