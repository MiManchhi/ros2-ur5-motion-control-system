#include "robot_interface_pkg/robot_interface_node.hpp"

#include "robot_common_pkg/constants.hpp"

namespace robot_interface_pkg
{

namespace c = robot_common_pkg::constants;

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

  // 发布到底层控制器
  controller_cmd_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      backend_.controller_topic(),
      10);

  // 发布任务事件到 manager
  motion_event_pub_ =
    this->create_publisher<robot_motion_msgs::msg::MotionEvent>(
      "/motion_event",
      20);

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

  // 调用后端适配器进行转换
  if (!backend_.convert_command(*msg, traj, error_msg)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 命令转换失败：%s",
      task_id.c_str(),
      error_msg.c_str());

    // 接口层失败不再直接写 /system_state_raw
    // 而是上报 execution_failed 事件，由 manager 统一收敛为正式 task_state=failed
    publish_motion_event(
      task_id,
      c::event::kExecutionFailed,
      c::task_state::kFailed,
      error_msg,
      0.0F,
      0.0,
      true);
    return;
  }

  // 转换成功，发布到底层控制器
  controller_cmd_pub_->publish(traj);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 已发布到底层控制器：%s，关节数=%zu",
    task_id.c_str(),
    backend_.controller_topic().c_str(),
    traj.joint_names.size());
}

void RobotInterfaceNode::publish_motion_event(
  const std::string & task_id,
  const std::string & event_name,
  const std::string & related_state,
  const std::string & detail,
  float progress,
  double current_error,
  bool is_error)
{
  robot_motion_msgs::msg::MotionEvent msg;
  msg.task_id = task_id;
  msg.module_name = c::module::kRobotInterface;
  msg.event_name = event_name;
  msg.related_state = related_state;
  msg.detail = detail;
  msg.progress = progress;
  msg.current_error = current_error;
  msg.is_error = is_error;
  msg.stamp = this->now();

  motion_event_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[motion_event] task_id=%s, module=%s, event=%s, related_state=%s, progress=%.3f, error=%.6f, is_error=%s, detail=%s",
    task_id.c_str(),
    c::module::kRobotInterface,
    event_name.c_str(),
    related_state.c_str(),
    progress,
    current_error,
    is_error ? "true" : "false",
    detail.c_str());
}

}  // namespace robot_interface_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_interface_pkg::RobotInterfaceNode)