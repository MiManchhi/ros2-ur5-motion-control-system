#ifndef PLANNER_PKG__PLANNER_NODE_HPP_
#define PLANNER_PKG__PLANNER_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "planner_pkg/simple_joint_planner.hpp"
#include "robot_motion_msgs/msg/motion_command.hpp"
#include "robot_motion_msgs/msg/motion_event.hpp"
#include "robot_motion_msgs/msg/planned_trajectory.hpp"

namespace planner_pkg
{

class PlannerNode : public rclcpp::Node
{
public:
  explicit PlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 接收上游 motion_api 发来的内部运动命令
  void on_motion_command(const robot_motion_msgs::msg::MotionCommand::SharedPtr msg);

  // 接收底层 joint_states
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 按目标关节顺序从 joint_state 中提取当前位置
  bool extract_current_positions(
    const std::vector<std::string> & target_joint_names,
    const sensor_msgs::msg::JointState & joint_state,
    std::vector<double> & reordered_positions) const;

  // 发布规划结果给控制层
  void publish_planned_trajectory(
    const std::string & task_id,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double timeout_sec);

  // 发布任务事件给 system_manager
  void publish_motion_event(
    const std::string & task_id,
    const std::string & event_name,
    const std::string & related_state,
    const std::string & detail,
    float progress,
    double current_error,
    bool is_error);

private:
  // 订阅内部运动命令
  rclcpp::Subscription<robot_motion_msgs::msg::MotionCommand>::SharedPtr motion_cmd_sub_;

  // 订阅 joint_states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // 发布规划轨迹
  rclcpp::Publisher<robot_motion_msgs::msg::PlannedTrajectory>::SharedPtr planned_traj_pub_;

  // 发布任务事件
  rclcpp::Publisher<robot_motion_msgs::msg::MotionEvent>::SharedPtr motion_event_pub_;

  // 简单规划器
  SimpleJointPlanner planner_;

  // 最近一次关节状态缓存
  sensor_msgs::msg::JointState latest_joint_state_;
  bool has_joint_state_ {false};

  // 规划参数
  int traj_points_ {50};
  double plan_duration_sec_ {5.0};
};

}  // namespace planner_pkg

#endif  // PLANNER_PKG__PLANNER_NODE_HPP_