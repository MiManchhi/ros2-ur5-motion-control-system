#include "system_manager_pkg/system_manager_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace system_manager_pkg
{

SystemManagerNode::SystemManagerNode(const rclcpp::NodeOptions & options)
: Node("system_manager_node", options)
{
  // =========================
  // 声明并读取参数
  // =========================
  this->declare_parameter<double>("task_timeout_sec", 20.0);
  this->declare_parameter<double>("watchdog_rate_hz", 2.0);

  this->get_parameter("task_timeout_sec", task_timeout_sec_);
  this->get_parameter("watchdog_rate_hz", watchdog_rate_hz_);

  if (task_timeout_sec_ <= 0.0) {
    task_timeout_sec_ = 20.0;
  }

  if (watchdog_rate_hz_ <= 0.0) {
    watchdog_rate_hz_ = 2.0;
  }

  // =========================
  // 创建订阅器
  // =========================
  system_state_raw_sub_ =
    this->create_subscription<robot_motion_msgs::msg::SystemState>(
      "/system_state_raw",
      50,
      std::bind(&SystemManagerNode::on_system_state_raw, this, std::placeholders::_1));

  // =========================
  // 创建发布器
  // =========================
  system_state_pub_ =
    this->create_publisher<robot_motion_msgs::msg::SystemState>(
      "/system_state",
      50);

  // =========================
  // 创建服务
  // =========================
  reset_service_ =
    this->create_service<robot_motion_msgs::srv::ResetSystem>(
      "/reset_system",
      std::bind(
        &SystemManagerNode::on_reset_system,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  // =========================
  // 创建监控定时器
  // =========================
  const auto period = std::chrono::duration<double>(1.0 / watchdog_rate_hz_);
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&SystemManagerNode::on_watchdog_timer, this));

  init_state_machine();

  RCLCPP_INFO(this->get_logger(), "system_manager_node 已启动");
  RCLCPP_INFO(
    this->get_logger(),
    "参数：task_timeout_sec=%.2f, watchdog_rate_hz=%.2f",
    task_timeout_sec_,
    watchdog_rate_hz_);
}

void SystemManagerNode::on_system_state_raw(
  const robot_motion_msgs::msg::SystemState::SharedPtr msg)
{
  handle_raw_state(*msg);
}

void SystemManagerNode::on_reset_system(
  const std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Request> request,
  std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Response> response)
{
  (void)request;

  RCLCPP_WARN(this->get_logger(), "收到 /reset_system 请求");

  state_machine_.reset();
  reset_active_task_context();

  publish_system_state("", "IDLE", "系统已重置并恢复到 IDLE", false);

  response->success = true;
  response->message = "system reset success";
}

void SystemManagerNode::on_watchdog_timer()
{
  if (!active_task_ctx_.active) {
    return;
  }

  const double elapsed = (this->now() - active_task_ctx_.start_time).seconds();
  if (elapsed > task_timeout_sec_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 任务整体超时",
      active_task_ctx_.task_id.c_str());

    if (state_machine_.transition_to(StateMachine::State::ERROR)) {
      publish_system_state(
        active_task_ctx_.task_id,
        "ERROR",
        "任务整体超时",
        true);
    }

    finish_active_task();

    // ERROR 状态结束后恢复 IDLE
    if (state_machine_.transition_to(StateMachine::State::IDLE)) {
      publish_system_state("", "IDLE", "系统恢复空闲", false);
    }
  }
}

void SystemManagerNode::init_state_machine()
{
  // 状态机初始化后，从 INIT 切到 IDLE
  if (state_machine_.transition_to(StateMachine::State::IDLE)) {
    publish_system_state("", "IDLE", "系统初始化完成，进入空闲状态", false);
  }
}

bool SystemManagerNode::can_accept_new_task() const
{
  return !active_task_ctx_.active &&
         state_machine_.current_state() == StateMachine::State::IDLE;
}

void SystemManagerNode::start_new_task(const std::string & task_id)
{
  active_task_ctx_.active = true;
  active_task_ctx_.task_id = task_id;
  active_task_ctx_.start_time = this->now();
  active_task_ctx_.last_event_time = this->now();
}

void SystemManagerNode::finish_active_task()
{
  reset_active_task_context();
}

void SystemManagerNode::reset_active_task_context()
{
  active_task_ctx_.active = false;
  active_task_ctx_.task_id.clear();
  active_task_ctx_.start_time = this->now();
  active_task_ctx_.last_event_time = this->now();
}

void SystemManagerNode::handle_raw_state(
  const robot_motion_msgs::msg::SystemState & raw_msg)
{
  const std::string & task_id = raw_msg.task_id;
  const std::string & state_str = raw_msg.state;

  RCLCPP_INFO(
    this->get_logger(),
    "[raw] task_id=%s, state=%s, message=%s, is_error=%s",
    task_id.c_str(),
    state_str.c_str(),
    raw_msg.message.c_str(),
    raw_msg.is_error ? "true" : "false");

  StateMachine::State target_state;
  if (!StateMachine::from_string(state_str, target_state)) {
    RCLCPP_WARN(
      this->get_logger(),
      "[task_id=%s] 未知状态字符串：%s",
      task_id.c_str(),
      state_str.c_str());
    return;
  }

  // =========================
  // 处理新任务开始
  // 当系统处于 IDLE 且收到 PLANNING 时，认为新任务开始
  // =========================
  if (target_state == StateMachine::State::PLANNING &&
      state_machine_.current_state() == StateMachine::State::IDLE) {
    if (!can_accept_new_task()) {
      publish_system_state(task_id, "ERROR", "系统忙碌，无法接受新任务", true);
      return;
    }

    start_new_task(task_id);

    if (!state_machine_.transition_to(StateMachine::State::PLANNING)) {
      publish_system_state(task_id, "ERROR", "状态切换到 PLANNING 失败", true);
      finish_active_task();
      return;
    }

    publish_system_state(task_id, "PLANNING", raw_msg.message, false);
    return;
  }

  // 当前没有活动任务时，忽略除 PLANNING 外的事件
  if (!active_task_ctx_.active) {
    RCLCPP_WARN(
      this->get_logger(),
      "[task_id=%s] 当前无活动任务，忽略状态：%s",
      task_id.c_str(),
      state_str.c_str());
    return;
  }

  // 只处理当前活动任务对应的状态
  if (task_id != active_task_ctx_.task_id) {
    RCLCPP_WARN(
      this->get_logger(),
      "[task_id=%s] 非当前活动任务（当前活动任务=%s），忽略该状态",
      task_id.c_str(),
      active_task_ctx_.task_id.c_str());
    return;
  }

  active_task_ctx_.last_event_time = this->now();

  // 尝试状态流转
  if (!state_machine_.transition_to(target_state)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 非法状态流转：%s -> %s",
      task_id.c_str(),
      state_machine_.current_state_string().c_str(),
      state_str.c_str());

    publish_system_state(task_id, "ERROR", "非法状态流转", true);

    // 非法状态流转时，进入 ERROR 并清理
    if (state_machine_.transition_to(StateMachine::State::ERROR)) {
      publish_system_state(task_id, "ERROR", "状态机已进入 ERROR", true);
    }

    finish_active_task();

    if (state_machine_.transition_to(StateMachine::State::IDLE)) {
      publish_system_state("", "IDLE", "系统恢复空闲", false);
    }
    return;
  }

  // 正常发布正式状态
  publish_system_state(task_id, state_str, raw_msg.message, raw_msg.is_error);

  // 若任务结束，清理上下文并恢复 IDLE
  if (target_state == StateMachine::State::FINISHED ||
      target_state == StateMachine::State::CANCELLED ||
      target_state == StateMachine::State::ERROR) {
    finish_active_task();

    if (state_machine_.transition_to(StateMachine::State::IDLE)) {
      publish_system_state("", "IDLE", "系统恢复空闲", false);
    }
  }
}

void SystemManagerNode::publish_system_state(
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
    "[system] task_id=%s, state=%s, message=%s, is_error=%s",
    task_id.c_str(),
    state.c_str(),
    message.c_str(),
    is_error ? "true" : "false");
}

} // namespace system_manager_pkg

// 如果需要组件化加载节点，则取消下面的注释，并在 CMakeLists.txt 中添加相关配置
// 组件化加载不需要main函数，直接由 rclcpp_components 管理
// 如果不需要组件化加载，则可以在 main.cpp 中直接创建并运行 MotionApiNode 实例

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(system_manager_pkg::SystemManagerNode)