#ifndef SYSTEM_MANAGER_PKG__SYSTEM_MANAGER_NODE_HPP_
#define SYSTEM_MANAGER_PKG__SYSTEM_MANAGER_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_motion_msgs/msg/system_state.hpp"
#include "robot_motion_msgs/srv/reset_system.hpp"

#include "system_manager_pkg/state_machine.hpp"

namespace system_manager_pkg
{

class SystemManagerNode : public rclcpp::Node
{
public:
  explicit SystemManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // =========================
  // 当前任务上下文
  // =========================
  struct ActiveTaskContext
  {
    bool active {false};               // 当前是否存在活动任务
    std::string task_id;               // 当前任务 ID
    rclcpp::Time start_time;           // 当前任务开始时间
    rclcpp::Time last_event_time;      // 最近一次事件时间
  };

  // =========================
  // 回调函数
  // =========================

  // 接收业务节点上报的原始系统状态
  void on_system_state_raw(const robot_motion_msgs::msg::SystemState::SharedPtr msg);

  // reset 服务
  void on_reset_system(
    const std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Request> request,
    std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Response> response);

  // 定时检查任务超时
  void on_watchdog_timer();

  // =========================
  // 业务辅助函数
  // =========================

  // 初始化系统状态
  void init_state_machine();

  // 是否允许接受新的任务
  bool can_accept_new_task() const;

  // 启动新任务
  void start_new_task(const std::string & task_id);

  // 结束当前任务
  void finish_active_task();

  // 清空当前任务上下文
  void reset_active_task_context();

  // 处理一条原始状态上报
  void handle_raw_state(
    const robot_motion_msgs::msg::SystemState & raw_msg);

  // 向外正式发布系统状态
  void publish_system_state(
    const std::string & task_id,
    const std::string & state,
    const std::string & message,
    bool is_error);

private:
  // =========================
  // ROS 通信对象
  // =========================

  // 订阅原始状态
  rclcpp::Subscription<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_raw_sub_;

  // 对外发布正式状态
  rclcpp::Publisher<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_pub_;

  // reset 服务
  rclcpp::Service<robot_motion_msgs::srv::ResetSystem>::SharedPtr reset_service_;

  // 监控定时器
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // =========================
  // 配置参数
  // =========================

  double task_timeout_sec_ {20.0};     // 任务整体超时时间
  double watchdog_rate_hz_ {2.0};      // 看门狗检查频率

  // =========================
  // 状态机与任务上下文
  // =========================

  StateMachine state_machine_;
  ActiveTaskContext active_task_ctx_;
};

}  // namespace system_manager_pkg

#endif  // SYSTEM_MANAGER_PKG__SYSTEM_MANAGER_NODE_HPP_