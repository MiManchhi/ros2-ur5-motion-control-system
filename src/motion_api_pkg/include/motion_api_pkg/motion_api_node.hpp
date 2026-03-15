#ifndef MOTION_API_PKG__MOTION_API_NODE_HPP_
#define MOTION_API_PKG__MOTION_API_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_motion_msgs/action/move_joints.hpp"
#include "robot_motion_msgs/msg/motion_command.hpp"
#include "robot_motion_msgs/msg/motion_event.hpp"
#include "robot_motion_msgs/msg/task_state.hpp"

namespace motion_api_pkg
{

class MotionApiNode : public rclcpp::Node
{
public:
  using MoveJoints = robot_motion_msgs::action::MoveJoints;
  using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<MoveJoints>;

  explicit MotionApiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ============================================================
  // 当前活动任务上下文
  // 用于在 action 生命周期内记录当前任务的执行信息
  // ============================================================
  struct ActiveGoalContext
  {
    bool active {false};                          // 当前是否存在活动任务
    bool cancel_requested {false};               // 是否收到取消请求
    std::string task_id;                         // 当前任务 id
    std::shared_ptr<GoalHandleMoveJoints> goal_handle;  // 当前 action goal handle
    double latest_progress {0.0};               // 最近一次进度
    double latest_error {0.0};                  // 最近一次误差
    std::string latest_task_state;              // 最近一次任务状态
  };

  // Action 回调：收到 goal 请求
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJoints::Goal> goal);

  // Action 回调：收到 cancel 请求
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  // Action 回调：正式接受任务
  void handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  // 订阅正式任务状态
  void on_task_state(const robot_motion_msgs::msg::TaskState::SharedPtr msg);

  // 校验任务输入
  bool validate_goal(const MoveJoints::Goal & goal, std::string & reason);

  // 生成 task_id
  std::string generate_task_id();

  // 任务执行主流程
  void execute_goal(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  // 发布内部运动命令
  void publish_motion_command(
    const std::string & task_id,
    const MoveJoints::Goal & goal);

  // 发布任务事件
  void publish_motion_event(
    const std::string & task_id,
    const std::string & event_name,
    const std::string & related_state,
    const std::string & detail,
    float progress,
    double current_error,
    bool is_error);

  // 发布 Action feedback
  void publish_feedback(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
    const std::string & task_id,
    const std::string & current_state,
    double progress,
    double current_error);

  // 任务成功结束
  void finish_goal_success(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
    const std::string & task_id,
    const std::string & message,
    double final_error);

  // 任务失败结束
  void finish_goal_abort(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
    const std::string & task_id,
    const std::string & message,
    double final_error);

  // 任务取消结束
  void finish_goal_cancel(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
    const std::string & task_id,
    const std::string & message,
    double final_error);

  // 清理活动任务上下文
  void reset_active_goal_context();

private:
  // Action Server
  rclcpp_action::Server<MoveJoints>::SharedPtr action_server_;

  // 发布内部运动命令到规划层
  rclcpp::Publisher<robot_motion_msgs::msg::MotionCommand>::SharedPtr motion_cmd_pub_;

  // 发布任务事件到 system_manager_node
  rclcpp::Publisher<robot_motion_msgs::msg::MotionEvent>::SharedPtr motion_event_pub_;

  // 订阅正式任务状态
  rclcpp::Subscription<robot_motion_msgs::msg::TaskState>::SharedPtr task_state_sub_;

  // 参数
  double default_speed_scale_ {1.0};
  double default_timeout_sec_ {10.0};
  bool enable_preempt_ {false};

  // 当前活动任务上下文
  ActiveGoalContext active_goal_ctx_;

  // task_id 计数器
  uint64_t task_counter_ {0};

  // 互斥锁，防止 action 线程和订阅线程并发修改上下文
  std::mutex goal_mutex_;
};

}  // namespace motion_api_pkg

#endif  // MOTION_API_PKG__MOTION_API_NODE_HPP_