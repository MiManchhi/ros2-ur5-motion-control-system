#include "motion_api_pkg/motion_api_node.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>

#include "robot_common_pkg/constants.hpp"

using namespace std::chrono_literals;

namespace motion_api_pkg
{

namespace c = robot_common_pkg::constants;

MotionApiNode::MotionApiNode(const rclcpp::NodeOptions & options)
: Node("motion_api_node", options)
{
  // =========================
  // 声明并读取参数
  // =========================
  this->declare_parameter<double>("default_speed_scale", 1.0);
  this->declare_parameter<double>("default_timeout_sec", 10.0);
  this->declare_parameter<bool>("enable_preempt", false);

  this->get_parameter("default_speed_scale", default_speed_scale_);
  this->get_parameter("default_timeout_sec", default_timeout_sec_);
  this->get_parameter("enable_preempt", enable_preempt_);

  // =========================
  // 创建发布器
  // =========================

  // 向 planner_node 发布内部运动命令
  motion_cmd_pub_ = this->create_publisher<robot_motion_msgs::msg::MotionCommand>(
    "/motion_command", 10);

  // 向 system_manager_node 发布任务事件
  motion_event_pub_ = this->create_publisher<robot_motion_msgs::msg::MotionEvent>(
    "/motion_event", 20);

  // =========================
  // 创建订阅器
  // =========================

  // 订阅正式任务状态
  task_state_sub_ = this->create_subscription<robot_motion_msgs::msg::TaskState>(
    "/task_state",
    20,
    std::bind(&MotionApiNode::on_task_state, this, std::placeholders::_1));

  // =========================
  // 创建 Action Server
  // =========================
  action_server_ = rclcpp_action::create_server<MoveJoints>(
    this,
    "/move_joints",
    std::bind(&MotionApiNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionApiNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&MotionApiNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "motion_api_node 已启动");
  RCLCPP_INFO(
    this->get_logger(),
    "参数：default_speed_scale=%.2f, default_timeout_sec=%.2f, enable_preempt=%s",
    default_speed_scale_,
    default_timeout_sec_,
    enable_preempt_ ? "true" : "false");
}

rclcpp_action::GoalResponse MotionApiNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveJoints::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(this->get_logger(), "收到 /move_joints 新任务请求");

  // 先校验输入参数
  std::string reason;
  if (!validate_goal(*goal, reason)) {
    RCLCPP_WARN(this->get_logger(), "任务校验失败：%s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::lock_guard<std::mutex> lock(goal_mutex_);

  // 当前阶段仍采用单任务串行执行策略
  if (active_goal_ctx_.active) {
    if (!enable_preempt_) {
      RCLCPP_WARN(
        this->get_logger(),
        "当前已有活动任务 task_id=%s，拒绝新任务",
        active_goal_ctx_.task_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    // 预留抢占扩展点，当前版本仍拒绝
    RCLCPP_WARN(
      this->get_logger(),
      "enable_preempt=true，但当前版本尚未实现抢占逻辑，仍然拒绝新任务");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionApiNode::handle_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);

  // 无活动任务，拒绝取消
  if (!active_goal_ctx_.active || !active_goal_ctx_.goal_handle) {
    RCLCPP_WARN(this->get_logger(), "收到取消请求，但当前无活动任务");
    return rclcpp_action::CancelResponse::REJECT;
  }

  // 只接受当前活动任务的取消
  if (goal_handle != active_goal_ctx_.goal_handle) {
    RCLCPP_WARN(this->get_logger(), "收到非当前活动任务的取消请求，拒绝");
    return rclcpp_action::CancelResponse::REJECT;
  }

  active_goal_ctx_.cancel_requested = true;

  RCLCPP_WARN(
    this->get_logger(),
    "[task_id=%s] 收到任务取消请求，准备发布 task_canceled 事件",
    active_goal_ctx_.task_id.c_str());

  // 不在这里直接结束 action
  // 而是先发布事件，由 manager 收敛成正式 task_state=canceled
  publish_motion_event(
    active_goal_ctx_.task_id,
    c::event::kTaskCanceled,
    c::task_state::kCanceled,
    "任务被客户端取消",
    static_cast<float>(active_goal_ctx_.latest_progress),
    active_goal_ctx_.latest_error,
    false);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionApiNode::handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  // 避免在回调线程中做阻塞逻辑，开启线程执行任务主流程
  std::thread(
    std::bind(&MotionApiNode::execute_goal, this, std::placeholders::_1),
    goal_handle).detach();
}

void MotionApiNode::on_task_state(const robot_motion_msgs::msg::TaskState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);

  // 当前没有活动任务，不处理
  if (!active_goal_ctx_.active || !active_goal_ctx_.goal_handle) {
    return;
  }

  // 只处理当前活动任务的状态
  if (msg->task_id != active_goal_ctx_.task_id) {
    return;
  }

  active_goal_ctx_.latest_progress = msg->progress;
  active_goal_ctx_.latest_error = msg->current_error;
  active_goal_ctx_.latest_task_state = msg->state;

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 收到正式 task_state：state=%s, progress=%.3f, error=%.6f, terminal=%s, is_error=%s, message=%s",
    msg->task_id.c_str(),
    msg->state.c_str(),
    msg->progress,
    msg->current_error,
    msg->is_terminal ? "true" : "false",
    msg->is_error ? "true" : "false",
    msg->message.c_str());

  // 非终态：只发 feedback
  if (!msg->is_terminal) {
    publish_feedback(
      active_goal_ctx_.goal_handle,
      active_goal_ctx_.task_id,
      msg->state,
      msg->progress,
      msg->current_error);
    return;
  }

  // 终态：根据正式任务状态结束 action
  if (msg->state == c::task_state::kCompleted) {
    finish_goal_success(
      active_goal_ctx_.goal_handle,
      active_goal_ctx_.task_id,
      msg->message,
      msg->current_error);
    return;
  }

  if (msg->state == c::task_state::kCanceled) {
    finish_goal_cancel(
      active_goal_ctx_.goal_handle,
      active_goal_ctx_.task_id,
      msg->message,
      msg->current_error);
    return;
  }

  if (msg->state == c::task_state::kFailed ||
      msg->state == c::task_state::kRejected) {
    finish_goal_abort(
      active_goal_ctx_.goal_handle,
      active_goal_ctx_.task_id,
      msg->message,
      msg->current_error);
    return;
  }
}

bool MotionApiNode::validate_goal(const MoveJoints::Goal & goal, std::string & reason)
{
  // joint_names 不能为空
  if (goal.joint_names.empty()) {
    reason = "joint_names 不能为空";
    return false;
  }

  // target_positions 不能为空
  if (goal.target_positions.empty()) {
    reason = "target_positions 不能为空";
    return false;
  }

  // 两者长度必须一致
  if (goal.joint_names.size() != goal.target_positions.size()) {
    reason = "joint_names 与 target_positions 长度不一致";
    return false;
  }

  // 当前按 UR5 六关节实现校验
  if (goal.joint_names.size() != 6) {
    reason = "当前实现要求关节数量为 6";
    return false;
  }

  // speed_scale 不能为负
  if (goal.speed_scale < 0.0) {
    reason = "speed_scale 不能小于 0";
    return false;
  }

  // timeout_sec 不能为负
  if (goal.timeout_sec < 0.0) {
    reason = "timeout_sec 不能小于 0";
    return false;
  }

  return true;
}

std::string MotionApiNode::generate_task_id()
{
  // task_id 形式：
  // task_YYYYMMDD_HHMMSS_001
  auto now = this->now();
  auto sec = now.seconds();

  std::time_t time_sec = static_cast<std::time_t>(sec);
  std::tm tm_time {};

#ifdef _WIN32
  localtime_s(&tm_time, &time_sec);
#else
  localtime_r(&time_sec, &tm_time);
#endif

  std::ostringstream oss;
  oss << "task_"
      << std::put_time(&tm_time, "%Y%m%d_%H%M%S")
      << "_"
      << std::setw(3) << std::setfill('0') << (++task_counter_);

  return oss.str();
}

void MotionApiNode::execute_goal(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  // 生成任务 id
  const std::string task_id = generate_task_id();

  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_ctx_.active = true;
    active_goal_ctx_.cancel_requested = false;
    active_goal_ctx_.task_id = task_id;
    active_goal_ctx_.goal_handle = goal_handle;
    active_goal_ctx_.latest_progress = 0.0;
    active_goal_ctx_.latest_error = 0.0;
    active_goal_ctx_.latest_task_state = c::task_state::kAccepted;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 开始执行任务，task_name=%s",
    task_id.c_str(),
    goal->task_name.c_str());

  // 先给客户端发一个初始 feedback
  publish_feedback(goal_handle, task_id, c::task_state::kAccepted, 0.0, 0.0);

  // 正式上报“任务已接收”事件
  publish_motion_event(
    task_id,
    c::event::kTaskReceived,
    c::task_state::kAccepted,
    "任务已接收",
    0.0F,
    0.0,
    false);

  // 将任务转成内部 MotionCommand 发给规划层
  publish_motion_command(task_id, *goal);

  // 超时保护
  const double timeout_sec =
    (goal->timeout_sec > 0.0) ? goal->timeout_sec : default_timeout_sec_;

  auto start_time = this->now();
  rclcpp::Rate rate(10.0);

  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);

      // 任务已经结束并被清理，线程退出
      if (!active_goal_ctx_.active || active_goal_ctx_.task_id != task_id) {
        return;
      }

      // 如果已经发起 cancel，则等待 manager 发布正式 canceled 状态
      if (active_goal_ctx_.cancel_requested) {
        rate.sleep();
        continue;
      }
    }

    const double elapsed = (this->now() - start_time).seconds();
    if (elapsed > timeout_sec) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[task_id=%s] 任务入口层等待超时，timeout=%.2f sec",
        task_id.c_str(),
        timeout_sec);

      // 不直接 abort，而是发布失败事件，由 manager 正式收敛为 failed
      publish_motion_event(
        task_id,
        c::event::kExecutionFailed,
        c::task_state::kFailed,
        "任务入口层等待超时",
        static_cast<float>(active_goal_ctx_.latest_progress),
        active_goal_ctx_.latest_error,
        true);
      return;
    }

    rate.sleep();
  }
}

void MotionApiNode::publish_motion_command(
  const std::string & task_id,
  const MoveJoints::Goal & goal)
{
  robot_motion_msgs::msg::MotionCommand cmd_msg;
  cmd_msg.task_id = task_id;
  cmd_msg.joint_names = goal.joint_names;
  cmd_msg.positions = goal.target_positions;
  cmd_msg.stamp = this->now();

  motion_cmd_pub_->publish(cmd_msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 已发布 /motion_command，关节数=%zu",
    task_id.c_str(),
    cmd_msg.joint_names.size());
}

void MotionApiNode::publish_motion_event(
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
  msg.module_name = c::module::kMotionApi;
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
    c::module::kMotionApi,
    event_name.c_str(),
    related_state.c_str(),
    progress,
    current_error,
    is_error ? "true" : "false",
    detail.c_str());
}

void MotionApiNode::publish_feedback(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
  const std::string & task_id,
  const std::string & current_state,
  double progress,
  double current_error)
{
  auto feedback = std::make_shared<MoveJoints::Feedback>();
  feedback->task_id = task_id;
  feedback->current_state = current_state;
  feedback->progress = progress;
  feedback->current_error = current_error;

  goal_handle->publish_feedback(feedback);

  RCLCPP_INFO(
    this->get_logger(),
    "[feedback] task_id=%s, state=%s, progress=%.3f, error=%.6f",
    task_id.c_str(),
    current_state.c_str(),
    progress,
    current_error);
}

void MotionApiNode::finish_goal_success(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
  const std::string & task_id,
  const std::string & message,
  double final_error)
{
  auto result = std::make_shared<MoveJoints::Result>();
  result->success = true;
  result->task_id = task_id;
  result->message = message;
  result->final_error = final_error;

  goal_handle->succeed(result);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 任务成功结束：%s，final_error=%.6f",
    task_id.c_str(),
    message.c_str(),
    final_error);

  reset_active_goal_context();
}

void MotionApiNode::finish_goal_abort(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
  const std::string & task_id,
  const std::string & message,
  double final_error)
{
  auto result = std::make_shared<MoveJoints::Result>();
  result->success = false;
  result->task_id = task_id;
  result->message = message;
  result->final_error = final_error;

  goal_handle->abort(result);

  RCLCPP_ERROR(
    this->get_logger(),
    "[task_id=%s] 任务失败结束：%s，final_error=%.6f",
    task_id.c_str(),
    message.c_str(),
    final_error);

  reset_active_goal_context();
}

void MotionApiNode::finish_goal_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
  const std::string & task_id,
  const std::string & message,
  double final_error)
{
  auto result = std::make_shared<MoveJoints::Result>();
  result->success = false;
  result->task_id = task_id;
  result->message = message;
  result->final_error = final_error;

  goal_handle->canceled(result);

  RCLCPP_WARN(
    this->get_logger(),
    "[task_id=%s] 任务取消结束：%s，final_error=%.6f",
    task_id.c_str(),
    message.c_str(),
    final_error);

  reset_active_goal_context();
}

void MotionApiNode::reset_active_goal_context()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);

  active_goal_ctx_.active = false;
  active_goal_ctx_.cancel_requested = false;
  active_goal_ctx_.task_id.clear();
  active_goal_ctx_.goal_handle.reset();
  active_goal_ctx_.latest_progress = 0.0;
  active_goal_ctx_.latest_error = 0.0;
  active_goal_ctx_.latest_task_state.clear();
}

}  // namespace motion_api_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motion_api_pkg::MotionApiNode)