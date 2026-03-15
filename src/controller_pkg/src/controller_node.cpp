#include "controller_pkg/controller_node.hpp"

#include <chrono>

#include "robot_common_pkg/constants.hpp"

using namespace std::chrono_literals;

namespace controller_pkg
{

namespace c = robot_common_pkg::constants;

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

  // 接收规划层输出的轨迹
  planned_traj_sub_ =
    this->create_subscription<robot_motion_msgs::msg::PlannedTrajectory>(
      "/planned_traj",
      10,
      std::bind(&ControllerNode::on_planned_trajectory, this, std::placeholders::_1));

  // 接收底层 joint_states
  joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      50,
      std::bind(&ControllerNode::on_joint_state, this, std::placeholders::_1));

  // 接收正式 task_state
  task_state_sub_ =
    this->create_subscription<robot_motion_msgs::msg::TaskState>(
      "/task_state",
      20,
      std::bind(&ControllerNode::on_task_state, this, std::placeholders::_1));

  // =========================
  // 创建发布器
  // =========================

  // 向接口层发布关节控制命令
  joint_cmd_pub_ =
    this->create_publisher<robot_motion_msgs::msg::MotionCommand>(
      "/joint_cmd",
      10);

  // 向 system_manager_node 发布任务事件
  motion_event_pub_ =
    this->create_publisher<robot_motion_msgs::msg::MotionEvent>(
      "/motion_event",
      20);

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

    publish_motion_event(
      task_id,
      c::event::kExecutionFailed,
      c::task_state::kFailed,
      error_msg,
      0.20F,
      0.0,
      true);
    return;
  }

  // 轨迹执行启动成功，发布 execution_started 事件
  publish_motion_event(
    task_id,
    c::event::kExecutionStarted,
    c::task_state::kExecuting,
    "控制器开始执行轨迹",
    0.30F,
    0.0,
    false);
}

void ControllerNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  executor_.update_joint_state(*msg, this->now());
}

void ControllerNode::on_task_state(const robot_motion_msgs::msg::TaskState::SharedPtr msg)
{
  // 当前没有活动任务，则无需处理
  if (!executor_.is_active()) {
    return;
  }

  // 只响应当前活动任务
  if (msg->task_id != executor_.active_task_id()) {
    return;
  }

  // 如果正式任务状态已经进入 canceled / failed / rejected
  // 控制层必须立即停止执行器，避免下游继续发控制命令
  if (msg->state == c::task_state::kCanceled ||
      msg->state == c::task_state::kFailed ||
      msg->state == c::task_state::kRejected) {
    RCLCPP_WARN(
      this->get_logger(),
      "[task_id=%s] 收到终止性 task_state=%s，停止当前执行器",
      msg->task_id.c_str(),
      msg->state.c_str());

    executor_.stop();
    return;
  }
}

void ControllerNode::on_control_timer()
{
  // 当前无活动轨迹，不需要推进
  if (!executor_.is_active()) {
    return;
  }

  const std::string task_id = executor_.active_task_id();

  // 推进一步执行
  const auto result = executor_.step(this->now());

  // 1. 执行异常：发布 execution_failed，并停止执行器
  if (result.has_error) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 执行异常：%s",
      task_id.c_str(),
      result.message.c_str());

    publish_motion_event(
      task_id,
      c::event::kExecutionFailed,
      c::task_state::kFailed,
      result.message,
      compute_progress(),
      result.current_error,
      true);

    executor_.stop();
    return;
  }

  // 2. 需要发控制命令：继续下发到接口层
  if (result.need_publish_command) {
    publish_joint_command(task_id, result.joint_names, result.positions);

    // 这里不每个周期都发布 event，避免事件风暴
    // progress / error 将在关键事件（完成/失败）上报
    return;
  }

  // 3. 已经完成：发布 execution_done，并停止执行器
  if (result.finished) {
    RCLCPP_INFO(
      this->get_logger(),
      "[task_id=%s] 执行完成，final_error=%.6f",
      task_id.c_str(),
      result.current_error);

    publish_motion_event(
      task_id,
      c::event::kExecutionDone,
      c::task_state::kCompleted,
      result.message.empty() ? "轨迹执行完成" : result.message,
      1.0F,
      result.current_error,
      false);

    executor_.stop();
    return;
  }

  // 4. 正常执行中，可按需输出调试日志
  if (!result.message.empty()) {
    RCLCPP_INFO(
      this->get_logger(),
      "[task_id=%s] %s，current_error=%.6f，progress=%.3f",
      task_id.c_str(),
      result.message.c_str(),
      result.current_error,
      compute_progress());
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

void ControllerNode::publish_motion_event(
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
  msg.module_name = c::module::kController;
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
    c::module::kController,
    event_name.c_str(),
    related_state.c_str(),
    progress,
    current_error,
    is_error ? "true" : "false",
    detail.c_str());
}

float ControllerNode::compute_progress() const
{
  // 当前无活动执行器，进度记为 0
  if (!executor_.is_active()) {
    return 0.0F;
  }

  const size_t total_points = executor_.total_points();
  if (total_points == 0U) {
    return 0.0F;
  }

  // 当前点索引 / 总点数，映射到执行阶段进度区间 [0.3, 0.95]
  const double ratio =
    static_cast<double>(executor_.current_point_index()) /
    static_cast<double>(total_points);

  double progress = 0.30 + ratio * 0.65;

  if (progress < 0.30) {
    progress = 0.30;
  }
  if (progress > 0.95) {
    progress = 0.95;
  }

  return static_cast<float>(progress);
}

}  // namespace controller_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(controller_pkg::ControllerNode)