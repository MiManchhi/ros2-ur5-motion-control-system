#include "motion_api_pkg/motion_api_node.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <thread>
#include <utility>

#include "robot_common_pkg/constants.hpp"

using namespace std::chrono_literals;

MotionApiNode::MotionApiNode(const rclcpp::NodeOptions & options)
: Node("motion_api_node", options), task_counter_(0)
{
  // =========================
  // 1. 声明参数
  // =========================
  this->declare_parameter<std::string>("action_name", "/move_joints");
  this->declare_parameter<std::string>("system_state_topic", "/system_state");
  this->declare_parameter<std::string>("motion_command_topic", "/motion_command");
  this->declare_parameter<double>("default_timeout_sec", 10.0);
  this->declare_parameter<double>("feedback_period_sec", 0.5);
  this->declare_parameter<double>("min_speed_scale", 0.1);
  this->declare_parameter<double>("max_speed_scale", 1.0);
  this->declare_parameter<int>("expected_joint_count", 6);

  // =========================
  // 2. 读取参数
  // =========================
  action_name_ = this->get_parameter("action_name").as_string();
  system_state_topic_ = this->get_parameter("system_state_topic").as_string();
  motion_command_topic_ = this->get_parameter("motion_command_topic").as_string();
  default_timeout_sec_ = this->get_parameter("default_timeout_sec").as_double();
  feedback_period_sec_ = this->get_parameter("feedback_period_sec").as_double();
  min_speed_scale_ = this->get_parameter("min_speed_scale").as_double();
  max_speed_scale_ = this->get_parameter("max_speed_scale").as_double();
  expected_joint_count_ = this->get_parameter("expected_joint_count").as_int();

  // =========================
  // 3. 初始化发布器
  // =========================
  system_state_pub_ =
    this->create_publisher<robot_motion_msgs::msg::SystemState>(system_state_topic_, 10);

  motion_command_pub_ =
    this->create_publisher<robot_motion_msgs::msg::MotionCommand>(motion_command_topic_, 10);

  // =========================
  // 4. 创建 Action Server
  // =========================
  action_server_ = rclcpp_action::create_server<MoveJoints>(
    this,
    action_name_,
    std::bind(&MotionApiNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionApiNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&MotionApiNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "MotionApiNode started.");
  RCLCPP_INFO(this->get_logger(), "Action server: %s", action_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "System state topic: %s", system_state_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Motion command topic: %s", motion_command_topic_.c_str());
}

rclcpp_action::GoalResponse MotionApiNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveJoints::Goal> goal)
{
  (void)uuid;

  // 收到新任务，先打印日志
  RCLCPP_INFO(
    this->get_logger(),
    "Received MoveJoints goal: task_name=%s",
    goal->task_name.c_str());

  // 参数合法性校验
  std::string reason;
  if (!validate_goal(goal, reason)) {
    RCLCPP_WARN(this->get_logger(), "Goal rejected: %s", reason.c_str());

    // 发布系统状态：rejected
    publish_system_state(
      "",
      robot_common_pkg::constants::state::kRejected,
      reason,
      true);

    return rclcpp_action::GoalResponse::REJECT;
  }

  // 阶段 2 中，只要参数合法就接收任务
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionApiNode::handle_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  (void)goal_handle;

  RCLCPP_WARN(this->get_logger(), "Received request to cancel goal.");

  // 阶段 2 中允许取消
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionApiNode::handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  // Action 执行通常不能阻塞 executor，所以放到独立线程
  std::thread(&MotionApiNode::execute, this, goal_handle).detach();
}

void MotionApiNode::execute(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  // 1. 生成系统内部唯一 task_id
  const std::string task_id = generate_task_id();

  RCLCPP_INFO(
    this->get_logger(),
    "Accepted task: task_id=%s, task_name=%s",
    task_id.c_str(),
    goal->task_name.c_str());

  // 2. 发布系统状态：accepted
  publish_system_state(
    task_id,
    robot_common_pkg::constants::state::kAccepted,
    "task accepted by motion_api_node",
    false);

  // 3. 准备 Action 的 feedback/result
  auto feedback = std::make_shared<MoveJoints::Feedback>();
  auto result = std::make_shared<MoveJoints::Result>();

  // 4. 先反馈：planning
  feedback->task_id = task_id;
  feedback->current_state = robot_common_pkg::constants::state::kPlanning;
  feedback->progress = 0.2;
  feedback->current_error = 0.0;
  goal_handle->publish_feedback(feedback);

  publish_system_state(
    task_id,
    robot_common_pkg::constants::state::kPlanning,
    "motion api is forwarding motion command to planner",
    false);

  // 5. 将 action 目标转成内部 MotionCommand 消息，发给 planner
  publish_motion_command(task_id, goal);

  RCLCPP_INFO(
    this->get_logger(),
    "Published MotionCommand: task_id=%s -> topic=%s",
    task_id.c_str(),
    motion_command_topic_.c_str());

  // 6. 再反馈一次：
  // 阶段 2 中这里只表示“已成功发给规划层”，并不表示底层已经完全执行结束
  std::this_thread::sleep_for(std::chrono::duration<double>(feedback_period_sec_));

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->task_id = task_id;
    result->message = "task canceled before planner handoff finished";
    result->final_error = 0.0;

    publish_system_state(
      task_id,
      robot_common_pkg::constants::state::kCanceled,
      "task canceled by client",
      true);

    goal_handle->canceled(result);

    RCLCPP_WARN(this->get_logger(), "Task canceled: %s", task_id.c_str());
    return;
  }

  feedback->task_id = task_id;
  feedback->current_state = robot_common_pkg::constants::state::kPlanning;
  feedback->progress = 0.8;
  feedback->current_error = 0.0;
  goal_handle->publish_feedback(feedback);

  // 7. 阶段 2 简化逻辑：
  // 只要成功发给 planner，就先认为“上游任务提交成功”
  result->success = true;
  result->task_id = task_id;
  result->message = "motion command has been forwarded to planner";
  result->final_error = 0.0;

  publish_system_state(
    task_id,
    robot_common_pkg::constants::state::kPlanning,
    "motion command forwarded successfully, waiting for downstream execution",
    false);

  goal_handle->succeed(result);

  RCLCPP_INFO(
    this->get_logger(),
    "Task finished in stage2 handoff flow: task_id=%s",
    task_id.c_str());
}

bool MotionApiNode::validate_goal(
  const std::shared_ptr<const MoveJoints::Goal> & goal,
  std::string & reason) const
{
  // joint_names 不能为空
  if (goal->joint_names.empty()) {
    reason = "joint_names must not be empty";
    return false;
  }

  // target_positions 不能为空
  if (goal->target_positions.empty()) {
    reason = "target_positions must not be empty";
    return false;
  }

  // joint_names 和 target_positions 必须一一对应
  if (goal->joint_names.size() != goal->target_positions.size()) {
    reason = "joint_names size must equal target_positions size";
    return false;
  }

  // 如果设置了预期关节数，则严格检查
  if (expected_joint_count_ > 0 &&
      static_cast<int>(goal->joint_names.size()) != expected_joint_count_) {
    std::ostringstream oss;
    oss << "joint count mismatch, expected " << expected_joint_count_
        << ", got " << goal->joint_names.size();
    reason = oss.str();
    return false;
  }

  // 检查速度缩放系数范围
  if (goal->speed_scale < min_speed_scale_ || goal->speed_scale > max_speed_scale_) {
    std::ostringstream oss;
    oss << "speed_scale out of range [" << min_speed_scale_
        << ", " << max_speed_scale_ << "]";
    reason = oss.str();
    return false;
  }

  // 超时时间：如果 goal 中传的是无效值，则后面会回退到默认值
  const double timeout_sec =
    (goal->timeout_sec > 0.0) ? goal->timeout_sec : default_timeout_sec_;

  if (timeout_sec <= 0.0) {
    reason = "timeout_sec must be > 0";
    return false;
  }

  reason = "ok";
  return true;
}

std::string MotionApiNode::generate_task_id()
{
  // 使用当前 ROS 时间戳 + 自增计数，生成唯一 task_id
  const auto now_ns = this->now().nanoseconds();
  const auto id = ++task_counter_;

  std::ostringstream oss;
  oss << "task_" << now_ns << "_" << id;
  return oss.str();
}

void MotionApiNode::publish_system_state(
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
}

void MotionApiNode::publish_motion_command(
  const std::string & task_id,
  const std::shared_ptr<const MoveJoints::Goal> & goal)
{
  robot_motion_msgs::msg::MotionCommand cmd_msg;

  // 将 action 任务中的目标信息转换为内部命令消息
  cmd_msg.task_id = task_id;
  cmd_msg.joint_names = goal->joint_names;
  cmd_msg.positions = goal->target_positions;
  cmd_msg.stamp = this->now();

  motion_command_pub_->publish(cmd_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionApiNode>());
  rclcpp::shutdown();
  return 0;
}