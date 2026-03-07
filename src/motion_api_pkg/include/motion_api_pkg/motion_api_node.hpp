#ifndef MOTION_API_PKG__MOTION_API_NODE_HPP_
#define MOTION_API_PKG__MOTION_API_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_motion_msgs/action/move_joints.hpp"
#include "robot_motion_msgs/msg/motion_command.hpp"
#include "robot_motion_msgs/msg/system_state.hpp"

class MotionApiNode : public rclcpp::Node
{
public:
  using MoveJoints = robot_motion_msgs::action::MoveJoints;
  using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<MoveJoints>;

  // 构造函数，支持 ROS2 标准 NodeOptions
  explicit MotionApiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Action 目标到达时的处理函数：
  // 这里只做“是否接收该任务”的判定，不做耗时逻辑
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJoints::Goal> goal);

  // Action 取消请求处理函数
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  // Action 目标被接受后，会在独立线程中执行 execute
  void handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  // Action 真正执行逻辑：
  // 阶段 2 中，这里负责：
  // 1. 生成 task_id
  // 2. 发布 MotionCommand 给 planner
  // 3. 发布 system_state
  // 4. 返回简化结果
  void execute(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);

  // 校验目标是否合法
  bool validate_goal(
    const std::shared_ptr<const MoveJoints::Goal> & goal,
    std::string & reason) const;

  // 生成系统内部唯一任务 ID
  std::string generate_task_id();

  // 发布系统状态消息
  void publish_system_state(
    const std::string & task_id,
    const std::string & state,
    const std::string & message,
    bool is_error);

  // 发布内部运动命令给 planner
  void publish_motion_command(
    const std::string & task_id,
    const std::shared_ptr<const MoveJoints::Goal> & goal);

private:
  // ===== 参数 =====

  // 对外 action 名称，例如 /move_joints
  std::string action_name_;

  // 系统状态 topic，例如 /system_state
  std::string system_state_topic_;

  // 向规划器发送命令的 topic，例如 /motion_command
  std::string motion_command_topic_;

  // 默认超时时间
  double default_timeout_sec_;

  // feedback 发布周期
  double feedback_period_sec_;

  // 速度缩放最小/最大值
  double min_speed_scale_;
  double max_speed_scale_;

  // 预期关节数，例如 UR5 为 6
  int expected_joint_count_;

  // 任务计数器，用于辅助生成唯一 task_id
  std::atomic<uint64_t> task_counter_;

  // ===== 发布器 / Action Server =====

  // 系统状态发布器
  rclcpp::Publisher<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_pub_;

  // 内部运动命令发布器
  rclcpp::Publisher<robot_motion_msgs::msg::MotionCommand>::SharedPtr motion_command_pub_;

  // 对外动作服务器
  rclcpp_action::Server<MoveJoints>::SharedPtr action_server_;
};

#endif  // MOTION_API_PKG__MOTION_API_NODE_HPP_