#ifndef SYSTEM_MANAGER_PKG__SYSTEM_MANAGER_NODE_HPP_
#define SYSTEM_MANAGER_PKG__SYSTEM_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_motion_msgs/msg/system_state.hpp"
#include "robot_motion_msgs/msg/motion_event.hpp"
#include "robot_motion_msgs/srv/reset_system.hpp"

#include "system_manager_pkg/state_machine.hpp"

class SystemManagerNode : public rclcpp::Node
{
public:
  explicit SystemManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 处理系统状态消息
  void system_state_callback(
    const robot_motion_msgs::msg::SystemState::SharedPtr msg);

  // 处理模块事件消息
  void motion_event_callback(
    const robot_motion_msgs::msg::MotionEvent::SharedPtr msg);

  // 处理复位服务
  void handle_reset_system(
    const std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Request> request,
    std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Response> response);

private:
  // 参数
  std::string system_state_topic_;
  std::string motion_event_topic_;
  std::string reset_service_name_;

  // 最近一次任务 id
  std::string current_task_id_;

  // 状态机
  StateMachine state_machine_;

  // 订阅器
  rclcpp::Subscription<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_sub_;
  rclcpp::Subscription<robot_motion_msgs::msg::MotionEvent>::SharedPtr motion_event_sub_;

  // 服务
  rclcpp::Service<robot_motion_msgs::srv::ResetSystem>::SharedPtr reset_system_srv_;
};

#endif  // SYSTEM_MANAGER_PKG__SYSTEM_MANAGER_NODE_HPP_