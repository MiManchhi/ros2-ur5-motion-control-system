#include "system_manager_pkg/system_manager_node.hpp"

#include <chrono>

#include "robot_common_pkg/constants.hpp"

using namespace std::chrono_literals;

namespace system_manager_pkg
{

namespace c = robot_common_pkg::constants;

SystemManagerNode::SystemManagerNode(const rclcpp::NodeOptions & options)
: Node("system_manager_node", options)
{
  // =========================
  // 声明并读取参数
  // =========================
  this->declare_parameter<double>("task_timeout_sec", 20.0);
  this->declare_parameter<double>("watchdog_rate_hz", 2.0);
  this->declare_parameter<bool>("auto_reset_on_failure", true);

  this->get_parameter("task_timeout_sec", task_timeout_sec_);
  this->get_parameter("watchdog_rate_hz", watchdog_rate_hz_);
  this->get_parameter("auto_reset_on_failure", auto_reset_on_failure_);

  if (task_timeout_sec_ <= 0.0) {
    task_timeout_sec_ = 20.0;
  }

  if (watchdog_rate_hz_ <= 0.0) {
    watchdog_rate_hz_ = 2.0;
  }

  // =========================
  // 创建订阅、发布、服务、定时器
  // =========================
  motion_event_sub_ =
    this->create_subscription<robot_motion_msgs::msg::MotionEvent>(
      "/motion_event",
      50,
      std::bind(&SystemManagerNode::on_motion_event, this, std::placeholders::_1));

  task_state_pub_ =
    this->create_publisher<robot_motion_msgs::msg::TaskState>(
      "/task_state",
      50);

  system_state_pub_ =
    this->create_publisher<robot_motion_msgs::msg::SystemState>(
      "/system_state",
      50);

  reset_service_ =
    this->create_service<robot_motion_msgs::srv::ResetSystem>(
      "/reset_system",
      std::bind(
        &SystemManagerNode::on_reset_system,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  const auto period = std::chrono::duration<double>(1.0 / watchdog_rate_hz_);
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&SystemManagerNode::on_watchdog_timer, this));

  init_system_state();

  RCLCPP_INFO(this->get_logger(), "system_manager_node 已启动");
  RCLCPP_INFO(
    this->get_logger(),
    "参数：task_timeout_sec=%.2f, watchdog_rate_hz=%.2f, auto_reset_on_failure=%s",
    task_timeout_sec_,
    watchdog_rate_hz_,
    auto_reset_on_failure_ ? "true" : "false");
}

void SystemManagerNode::on_motion_event(
  const robot_motion_msgs::msg::MotionEvent::SharedPtr msg)
{
  handle_motion_event(*msg);
}

void SystemManagerNode::on_reset_system(
  const std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Request> request,
  std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Response> response)
{
  (void)request;

  RCLCPP_WARN(this->get_logger(), "收到 /reset_system 请求");

  // 先广播系统正在重置
  publish_system_state(
    active_task_ctx_.active ? active_task_ctx_.task_id : "",
    c::system_state::kResetting,
    "系统正在重置",
    false);

  // 如果当前存在活动任务，需要先收口该任务
  if (active_task_ctx_.active) {
    publish_task_state(
      active_task_ctx_.task_id,
      c::task_state::kFailed,
      c::module::kSystemManager,
      "系统重置导致当前任务被终止",
      active_task_ctx_.progress,
      active_task_ctx_.current_error,
      true,
      true);
  }

  // 清空上下文并恢复系统状态
  reset_active_task_context();

  publish_system_state(
    "",
    c::system_state::kIdle,
    "系统已重置并恢复空闲",
    false);

  response->success = true;
  response->message = "system reset success";
}

void SystemManagerNode::on_watchdog_timer()
{
  // 当前无活动任务，则无需 watchdog
  if (!active_task_ctx_.active) {
    return;
  }

  const double elapsed = (this->now() - active_task_ctx_.start_time).seconds();
  if (elapsed <= task_timeout_sec_) {
    return;
  }

  RCLCPP_ERROR(
    this->get_logger(),
    "[task_id=%s] 任务整体超时，elapsed=%.3f sec",
    active_task_ctx_.task_id.c_str(),
    elapsed);

  // 任务整体超时，收敛为 failed
  publish_task_state(
    active_task_ctx_.task_id,
    c::task_state::kFailed,
    c::module::kSystemManager,
    "任务整体超时",
    active_task_ctx_.progress,
    active_task_ctx_.current_error,
    true,
    true);

  finish_active_task();

  if (auto_reset_on_failure_) {
    publish_system_state(
      "",
      c::system_state::kIdle,
      "系统已从任务超时中恢复空闲",
      false);
  } else {
    publish_system_state(
      "",
      c::system_state::kError,
      "任务超时后系统进入 error 状态",
      true);
  }
}

void SystemManagerNode::init_system_state()
{
  publish_system_state(
    "",
    c::system_state::kInit,
    "系统初始化中",
    false);

  publish_system_state(
    "",
    c::system_state::kIdle,
    "系统初始化完成，进入空闲",
    false);
}

bool SystemManagerNode::can_accept_new_task() const
{
  return !active_task_ctx_.active;
}

void SystemManagerNode::start_new_task(
  const std::string & task_id,
  const std::string & initial_state,
  const std::string & source_module,
  float progress,
  double current_error)
{
  active_task_ctx_.active = true;
  active_task_ctx_.task_id = task_id;
  active_task_ctx_.state = initial_state;
  active_task_ctx_.source_module = source_module;
  active_task_ctx_.start_time = this->now();
  active_task_ctx_.last_event_time = this->now();
  active_task_ctx_.progress = progress;
  active_task_ctx_.current_error = current_error;

  TaskStateMachine::State init_state;
  if (TaskStateMachine::from_string(initial_state, init_state)) {
    task_state_machine_.reset(init_state);
  } else {
    task_state_machine_.reset(TaskStateMachine::State::ACCEPTED);
  }
}

void SystemManagerNode::update_active_task(
  const std::string & state,
  const std::string & source_module,
  float progress,
  double current_error)
{
  active_task_ctx_.state = state;
  active_task_ctx_.source_module = source_module;
  active_task_ctx_.last_event_time = this->now();
  active_task_ctx_.progress = progress;
  active_task_ctx_.current_error = current_error;
}

void SystemManagerNode::finish_active_task()
{
  reset_active_task_context();
}

void SystemManagerNode::reset_active_task_context()
{
  active_task_ctx_.active = false;
  active_task_ctx_.task_id.clear();
  active_task_ctx_.state.clear();
  active_task_ctx_.source_module.clear();
  active_task_ctx_.start_time = this->now();
  active_task_ctx_.last_event_time = this->now();
  active_task_ctx_.progress = 0.0F;
  active_task_ctx_.current_error = 0.0;

  task_state_machine_.reset(TaskStateMachine::State::ACCEPTED);
}

void SystemManagerNode::handle_motion_event(
  const robot_motion_msgs::msg::MotionEvent & event_msg)
{
  const std::string & task_id = event_msg.task_id;
  const std::string & module_name = event_msg.module_name;
  const std::string & event_name = event_msg.event_name;

  RCLCPP_INFO(
    this->get_logger(),
    "[event] task_id=%s, module=%s, event=%s, related_state=%s, detail=%s, is_error=%s",
    task_id.c_str(),
    module_name.c_str(),
    event_name.c_str(),
    event_msg.related_state.c_str(),
    event_msg.detail.c_str(),
    event_msg.is_error ? "true" : "false");

  // 系统级 reset 事件，可不依赖 task_id
  if (event_name == c::event::kSystemReset) {
    publish_system_state(
      "",
      c::system_state::kResetting,
      event_msg.detail.empty() ? "收到系统重置事件" : event_msg.detail,
      false);
    return;
  }

  // 非 reset 事件必须有 task_id
  if (task_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "收到无 task_id 的任务事件，忽略");
    return;
  }

  const std::string target_state = resolve_target_task_state(event_msg);
  if (target_state.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[task_id=%s] 无法根据事件推导目标状态：event=%s",
      task_id.c_str(),
      event_name.c_str());
    return;
  }

  TaskStateMachine::State new_state;
  if (!TaskStateMachine::from_string(target_state, new_state)) {
    RCLCPP_WARN(
      this->get_logger(),
      "[task_id=%s] 目标状态字符串无法识别：%s",
      task_id.c_str(),
      target_state.c_str());
    return;
  }

  // =========================
  // 新任务启动逻辑
  // accepted 是正式任务进入 manager 管理的入口
  // =========================
  if (!active_task_ctx_.active) {
    if (new_state != TaskStateMachine::State::ACCEPTED) {
      RCLCPP_WARN(
        this->get_logger(),
        "[task_id=%s] 当前无活动任务，只接受 accepted 作为新任务起点，忽略 state=%s",
        task_id.c_str(),
        target_state.c_str());
      return;
    }

    if (!can_accept_new_task()) {
      publish_system_state(
        task_id,
        c::system_state::kError,
        "系统忙碌，无法接受新任务",
        true);
      return;
    }

    start_new_task(
      task_id,
      target_state,
      module_name,
      event_msg.progress,
      event_msg.current_error);

    publish_task_state(
      task_id,
      target_state,
      module_name,
      event_msg.detail.empty() ? "任务已进入正式管理流程" : event_msg.detail,
      event_msg.progress,
      event_msg.current_error,
      false,
      event_msg.is_error);

    refresh_system_state();
    return;
  }

  // =========================
  // 当前有活动任务时，只处理当前任务
  // =========================
  if (task_id != active_task_ctx_.task_id) {
    RCLCPP_WARN(
      this->get_logger(),
      "[task_id=%s] 当前活动任务=%s，忽略其他任务事件",
      task_id.c_str(),
      active_task_ctx_.task_id.c_str());
    return;
  }

  // 状态机流转校验
  if (!task_state_machine_.transition_to(new_state)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 非法任务状态流转：%s -> %s",
      task_id.c_str(),
      active_task_ctx_.state.c_str(),
      target_state.c_str());

    publish_task_state(
      task_id,
      c::task_state::kFailed,
      c::module::kSystemManager,
      "非法任务状态流转",
      active_task_ctx_.progress,
      active_task_ctx_.current_error,
      true,
      true);

    finish_active_task();
    refresh_system_state();
    return;
  }

  // 更新上下文
  update_active_task(
    target_state,
    module_name,
    event_msg.progress,
    event_msg.current_error);

  // 发布正式 task_state
  publish_task_state(
    task_id,
    target_state,
    module_name,
    event_msg.detail.empty() ? "任务状态已更新" : event_msg.detail,
    event_msg.progress,
    event_msg.current_error,
    TaskStateMachine::is_terminal(new_state),
    event_msg.is_error);

  // 若为终态，则清理任务上下文
  if (TaskStateMachine::is_terminal(new_state)) {
    finish_active_task();
  }

  refresh_system_state();
}

std::string SystemManagerNode::resolve_target_task_state(
  const robot_motion_msgs::msg::MotionEvent & event_msg) const
{
  // 若事件中显式带了 related_state，则优先使用
  if (!event_msg.related_state.empty()) {
    return event_msg.related_state;
  }

  // 否则根据事件名映射
  return c::event_to_task_state(event_msg.event_name);
}

void SystemManagerNode::publish_task_state(
  const std::string & task_id,
  const std::string & state,
  const std::string & source_module,
  const std::string & message,
  float progress,
  double current_error,
  bool is_terminal,
  bool is_error)
{
  robot_motion_msgs::msg::TaskState msg;
  msg.task_id = task_id;
  msg.state = state;
  msg.source_module = source_module;
  msg.message = message;
  msg.progress = progress;
  msg.current_error = current_error;
  msg.is_terminal = is_terminal;
  msg.is_error = is_error;
  msg.stamp = this->now();

  task_state_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_state] task_id=%s, state=%s, source=%s, progress=%.3f, error=%.6f, terminal=%s, is_error=%s, message=%s",
    task_id.c_str(),
    state.c_str(),
    source_module.c_str(),
    progress,
    current_error,
    is_terminal ? "true" : "false",
    is_error ? "true" : "false",
    message.c_str());
}

void SystemManagerNode::publish_system_state(
  const std::string & active_task_id,
  const std::string & state,
  const std::string & message,
  bool is_error)
{
  robot_motion_msgs::msg::SystemState msg;
  msg.task_id = active_task_id;
  msg.state = state;
  msg.message = message;
  msg.is_error = is_error;
  msg.stamp = this->now();

  system_state_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[system_state] active_task_id=%s, state=%s, is_error=%s, message=%s",
    active_task_id.c_str(),
    state.c_str(),
    is_error ? "true" : "false",
    message.c_str());
}

void SystemManagerNode::refresh_system_state()
{
  if (active_task_ctx_.active) {
    publish_system_state(
      active_task_ctx_.task_id,
      c::system_state::kBusy,
      "系统存在活动任务",
      false);
    return;
  }

  publish_system_state(
    "",
    c::system_state::kIdle,
    "系统当前无活动任务",
    false);
}

}  // namespace system_manager_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(system_manager_pkg::SystemManagerNode)