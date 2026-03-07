#include "system_manager_pkg/system_manager_node.hpp"

#include <utility>

#include "robot_common_pkg/constants.hpp"

SystemManagerNode::SystemManagerNode(const rclcpp::NodeOptions & options)
: Node("system_manager_node", options)
{
  // =========================
  // 1. 声明参数
  // =========================
  this->declare_parameter<std::string>("system_state_topic", "/system_state");
  this->declare_parameter<std::string>("motion_event_topic", "/motion_event");
  this->declare_parameter<std::string>("reset_service_name", "/reset_system");

  // =========================
  // 2. 读取参数
  // =========================
  system_state_topic_ = this->get_parameter("system_state_topic").as_string();
  motion_event_topic_ = this->get_parameter("motion_event_topic").as_string();
  reset_service_name_ = this->get_parameter("reset_service_name").as_string();

  // =========================
  // 3. 初始化订阅器
  // =========================
  system_state_sub_ =
    this->create_subscription<robot_motion_msgs::msg::SystemState>(
    system_state_topic_,
    10,
    std::bind(&SystemManagerNode::system_state_callback, this, std::placeholders::_1));

  motion_event_sub_ =
    this->create_subscription<robot_motion_msgs::msg::MotionEvent>(
    motion_event_topic_,
    10,
    std::bind(&SystemManagerNode::motion_event_callback, this, std::placeholders::_1));

  // =========================
  // 4. 初始化服务
  // =========================
  reset_system_srv_ =
    this->create_service<robot_motion_msgs::srv::ResetSystem>(
    reset_service_name_,
    std::bind(
      &SystemManagerNode::handle_reset_system,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "SystemManagerNode started.");
  RCLCPP_INFO(this->get_logger(), "System state topic: %s", system_state_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Motion event topic: %s", motion_event_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Reset service: %s", reset_service_name_.c_str());
}

void SystemManagerNode::system_state_callback(
  const robot_motion_msgs::msg::SystemState::SharedPtr msg)
{
  // 更新当前任务 ID（如果消息中带了）
  if (!msg->task_id.empty()) {
    current_task_id_ = msg->task_id;
  }

  // 更新状态机
  state_machine_.set_state(msg->state);

  // 打印状态变化
  RCLCPP_INFO(
    this->get_logger(),
    "[SystemState] task_id=%s state=%s is_error=%s message=%s",
    msg->task_id.c_str(),
    msg->state.c_str(),
    msg->is_error ? "true" : "false",
    msg->message.c_str());
}

void SystemManagerNode::motion_event_callback(
  const robot_motion_msgs::msg::MotionEvent::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "[MotionEvent] task_id=%s module=%s event=%s detail=%s",
    msg->task_id.c_str(),
    msg->module_name.c_str(),
    msg->event_name.c_str(),
    msg->detail.c_str());
}

void SystemManagerNode::handle_reset_system(
  const std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Request> request,
  std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Response> response)
{
  // 当前阶段 2 的最小实现：
  // 只做状态机和任务号的本地重置
  (void)request;

  current_task_id_.clear();
  state_machine_.reset();

  response->success = true;
  response->message = "system manager local state has been reset";

  RCLCPP_WARN(
    this->get_logger(),
    "System reset requested. Current state has been reset to: %s",
    state_machine_.get_state().c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemManagerNode>());
  rclcpp::shutdown();
  return 0;
}