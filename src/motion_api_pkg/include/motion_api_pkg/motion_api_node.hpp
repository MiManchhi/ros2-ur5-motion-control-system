#ifndef MOTION_API_PKG__MOTION_API_NODE_HPP_
#define MOTION_API_PKG__MOTION_API_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_motion_msgs/action/move_joints.hpp"
#include "robot_motion_msgs/msg/motion_command.hpp"
#include "robot_motion_msgs/msg/system_state.hpp"

namespace motion_api_pkg
{

class MotionApiNode : public rclcpp::Node
{
public:
  // =========================
  // 类型别名
  // =========================
  using MoveJoints = robot_motion_msgs::action::MoveJoints;
  using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<MoveJoints>;

  explicit MotionApiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // =========================
  // 内部结构体定义
  // =========================

  // 当前活动任务上下文
  // 将“当前任务”的相关状态聚合在一起，便于统一维护和清理
  struct ActiveGoalContext
  {
    bool active {false};                               // 当前是否存在活动任务
    std::string task_id;                               // 当前任务 ID
    std::shared_ptr<GoalHandleMoveJoints> goal_handle; // 当前活动任务句柄
    double latest_progress {0.0};                      // 最近一次任务进度
    double latest_error {0.0};                         // 最近一次任务误差
  };

  // =========================
  // Action Server 回调
  // =========================

  // 收到新的 goal 请求时调用
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJoints::Goal> goal);

  // 收到取消请求时调用
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  // goal 被接受后调用
  void handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  // =========================
  // Topic 订阅回调
  // =========================

  // 接收系统状态更新
  void on_system_state(const robot_motion_msgs::msg::SystemState::SharedPtr msg);

  // =========================
  // 业务辅助函数
  // =========================

  // 校验 goal 是否合法
  bool validate_goal(const MoveJoints::Goal & goal, std::string & reason);

  // 生成唯一任务 ID
  std::string generate_task_id();

  // 真正执行任务的主流程
  void execute_goal(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  // 向 planner_node 发布运动命令
  void publish_motion_command(
    const std::string & task_id,
    const MoveJoints::Goal & goal);

  // 向 action 客户端发布 feedback
  void publish_feedback(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
    const std::string & task_id,
    const std::string & current_state,
    double progress,
    double current_error);

  // 结束任务：成功
  void finish_goal_success(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
    const std::string & task_id,
    const std::string & message,
    double final_error);

  // 结束任务：失败
  void finish_goal_abort(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
    const std::string & task_id,
    const std::string & message,
    double final_error);

  // 结束任务：取消
  void finish_goal_cancel(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
    const std::string & task_id,
    const std::string & message,
    double final_error);

  // 清空当前活动任务上下文
  void reset_active_goal_context();

private:
  // =========================
  // ROS 通信对象
  // =========================

  // Action 服务端：对外提供 /move_joints
  rclcpp_action::Server<MoveJoints>::SharedPtr action_server_;

  // 向规划节点发布运动任务
  rclcpp::Publisher<robot_motion_msgs::msg::MotionCommand>::SharedPtr motion_cmd_pub_;

  // 订阅系统状态
  rclcpp::Subscription<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_sub_;

  // =========================
  // 配置参数
  // =========================

  double default_speed_scale_ {1.0};   // 默认速度比例
  double default_timeout_sec_ {10.0};  // 默认超时时间
  bool enable_preempt_ {false};        // 是否允许任务抢占（当前版本未真正实现）

  // =========================
  // 当前任务上下文
  // =========================

  ActiveGoalContext active_goal_ctx_;

  // =========================
  // 节点运行缓存状态
  // =========================

  std::string latest_system_state_ {"IDLE"};  // 最近一次接收到的系统状态

  // =========================
  // 工具型成员变量
  // =========================

  uint64_t task_counter_ {0};  // 用于辅助生成唯一 task_id
};

}  // namespace motion_api_pkg

#endif  // MOTION_API_PKG__MOTION_API_NODE_HPP_