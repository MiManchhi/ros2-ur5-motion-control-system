#ifndef SYSTEM_MANAGER_PKG__SYSTEM_MANAGER_NODE_HPP_
#define SYSTEM_MANAGER_PKG__SYSTEM_MANAGER_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_motion_msgs/msg/motion_event.hpp"
#include "robot_motion_msgs/msg/task_state.hpp"
#include "robot_motion_msgs/msg/system_state.hpp"
#include "robot_motion_msgs/srv/reset_system.hpp"

#include "system_manager_pkg/task_state_machine.hpp"

namespace system_manager_pkg
{

class SystemManagerNode : public rclcpp::Node
{
public:
  explicit SystemManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ============================================================
  // 当前活动任务上下文
  // system_manager_node 负责维护当前被正式管理的任务信息
  // ============================================================
  struct ActiveTaskContext
  {
    bool active {false};                 // 当前是否存在活动任务
    std::string task_id;                 // 当前活动任务 id
    std::string state;                   // 当前活动任务状态（字符串形式）
    std::string source_module;           // 最近一次更新来源模块
    rclcpp::Time start_time;             // 任务开始时间
    rclcpp::Time last_event_time;        // 最近一次事件时间
    float progress {0.0F};               // 当前进度
    double current_error {0.0};          // 当前误差
  };

private:
  // 接收各业务节点上报的任务事件
  void on_motion_event(const robot_motion_msgs::msg::MotionEvent::SharedPtr msg);

  // reset 服务回调
  void on_reset_system(
    const std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Request> request,
    std::shared_ptr<robot_motion_msgs::srv::ResetSystem::Response> response);

  // watchdog 定时器回调
  void on_watchdog_timer();

  // 初始化系统状态
  void init_system_state();

  // 判断当前系统是否可接受新任务
  bool can_accept_new_task() const;

  // 启动一个新任务的正式上下文
  void start_new_task(
    const std::string & task_id,
    const std::string & initial_state,
    const std::string & source_module,
    float progress,
    double current_error);

  // 更新当前活动任务上下文
  void update_active_task(
    const std::string & state,
    const std::string & source_module,
    float progress,
    double current_error);

  // 完成并清理当前任务上下文
  void finish_active_task();

  // 彻底清空任务上下文
  void reset_active_task_context();

  // 处理单条事件
  void handle_motion_event(const robot_motion_msgs::msg::MotionEvent & event_msg);

  // 由事件计算目标任务状态
  std::string resolve_target_task_state(const robot_motion_msgs::msg::MotionEvent & event_msg) const;

  // 发布正式任务状态
  void publish_task_state(
    const std::string & task_id,
    const std::string & state,
    const std::string & source_module,
    const std::string & message,
    float progress,
    double current_error,
    bool is_terminal,
    bool is_error);

  // 发布正式系统状态
  void publish_system_state(
    const std::string & active_task_id,
    const std::string & state,
    const std::string & message,
    bool is_error);

  // 根据当前活动任务情况派生系统状态
  void refresh_system_state();

private:
  // 订阅任务事件
  rclcpp::Subscription<robot_motion_msgs::msg::MotionEvent>::SharedPtr motion_event_sub_;

  // 发布正式任务状态
  rclcpp::Publisher<robot_motion_msgs::msg::TaskState>::SharedPtr task_state_pub_;

  // 发布正式系统状态
  rclcpp::Publisher<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_pub_;

  // reset 服务
  rclcpp::Service<robot_motion_msgs::srv::ResetSystem>::SharedPtr reset_service_;

  // watchdog 定时器
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // 参数：整体任务超时时间
  double task_timeout_sec_ {20.0};

  // 参数：watchdog 检查频率
  double watchdog_rate_hz_ {2.0};

  // 参数：是否在失败后自动恢复到 idle
  bool auto_reset_on_failure_ {true};

  // 当前任务状态机
  TaskStateMachine task_state_machine_;

  // 当前活动任务上下文
  ActiveTaskContext active_task_ctx_;
};

}  // namespace system_manager_pkg

#endif  // SYSTEM_MANAGER_PKG__SYSTEM_MANAGER_NODE_HPP_