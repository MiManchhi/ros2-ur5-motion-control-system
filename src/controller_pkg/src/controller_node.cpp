#include "controller_pkg/controller_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace controller_pkg
{

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller_node", options)
{
  // =========================
  // 声明并读取参数
  // =========================
  this->declare_parameter<double>("control_rate_hz", 50.0);
  this->declare_parameter<double>("goal_tolerance", 0.01);
  this->declare_parameter<double>("execution_timeout_sec", 15.0);
  this->declare_parameter<double>("feedback_timeout_sec", 1.0);

  this->get_parameter("control_rate_hz", control_rate_hz_);

  TrajectoryExecutor::Config config;
  this->get_parameter("goal_tolerance", config.goal_tolerance);
  this->get_parameter("execution_timeout_sec", config.execution_timeout_sec);
  this->get_parameter("feedback_timeout_sec", config.feedback_timeout_sec);

  if (control_rate_hz_ <= 0.0) {
    control_rate_hz_ = 50.0;
  }

  executor_.set_config(config);

  // =========================
  // 创建订阅器
  // =========================
  planned_traj_sub_ =
    this->create_subscription<robot_motion_msgs::msg::PlannedTrajectory>(
      "/planned_traj",
      10,
      std::bind(&ControllerNode::on_planned_trajectory, this, std::placeholders::_1));

  joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      50,
      std::bind(&ControllerNode::on_joint_state, this, std::placeholders::_1));

  // =========================
  // 创建发布器
  // =========================
  joint_cmd_pub_ =
    this->create_publisher<robot_motion_msgs::msg::MotionCommand>("/joint_cmd", 10);

  system_state_pub_ =
    this->create_publisher<robot_motion_msgs::msg::SystemState>("/system_state_raw", 10);

  // =========================
  // 创建控制定时器
  // =========================
  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&ControllerNode::on_control_timer, this));

  RCLCPP_INFO(this->get_logger(), "controller_node 已启动");
  RCLCPP_INFO(this->get_logger(), "参数：control_rate_hz=%.2f", control_rate_hz_);
}

void ControllerNode::on_planned_trajectory(
  const robot_motion_msgs::msg::PlannedTrajectory::SharedPtr msg)
{
  const std::string & task_id = msg->task_id;

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 收到 /planned_traj，轨迹点数=%zu",
    task_id.c_str(),
    msg->trajectory.points.size());

  std::string error_msg;
  if (!executor_.start(task_id, msg->trajectory, this->now(), error_msg)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 启动轨迹执行失败：%s",
      task_id.c_str(),
      error_msg.c_str());

    publish_system_state(task_id, "ERROR", error_msg, true);
    return;
  }

  publish_system_state(task_id, "RUNNING", "控制器开始执行轨迹", false);
}

void ControllerNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  executor_.update_joint_state(*msg, this->now());
}

void ControllerNode::on_control_timer()
{
  if (!executor_.is_active()) {
    return;
  }

  const std::string task_id = executor_.active_task_id();
  const auto result = executor_.step(this->now());

  if (result.has_error) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 执行异常：%s",
      task_id.c_str(),
      result.message.c_str());

    publish_system_state(task_id, "ERROR", result.message, true);
    executor_.stop();
    return;
  }

  if (result.need_publish_command) {
    publish_joint_command(task_id, result.joint_names, result.positions);
    return;
  }

  if (result.finished) {
    RCLCPP_INFO(
      this->get_logger(),
      "[task_id=%s] 任务完成，final_error=%.6f",
      task_id.c_str(),
      result.current_error);

    publish_system_state(task_id, "FINISHED", result.message, false);
    executor_.stop();
    return;
  }

  // 当前既没有错误，也不需要发命令，也没完成
  // 说明正在等待最终到位，可按需打印调试信息
  if (!result.message.empty()) {
    RCLCPP_INFO(
      this->get_logger(),
      "[task_id=%s] %s，current_error=%.6f",
      task_id.c_str(),
      result.message.c_str(),
      result.current_error);
  }
}

void ControllerNode::publish_joint_command(
  const std::string & task_id,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions)
{
  robot_motion_msgs::msg::MotionCommand msg;
  msg.task_id = task_id;
  msg.joint_names = joint_names;
  msg.positions = positions;
  msg.stamp = this->now();

  joint_cmd_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 已发布 /joint_cmd，关节数=%zu",
    task_id.c_str(),
    joint_names.size());
}

void ControllerNode::publish_system_state(
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

}  // namespace controller_pkg

// 如果需要组件化加载节点，则取消下面的注释，并在 CMakeLists.txt 中添加相关配置
// 组件化加载不需要main函数，直接由 rclcpp_components 管理
// 如果不需要组件化加载，则可以在 main.cpp 中直接创建并运行 MotionApiNode 实例

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(controller_pkg::ControllerNode)