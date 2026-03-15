#ifndef PLANNER_PKG__PLANNER_NODE_HPP_
#define PLANNER_PKG__PLANNER_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "robot_motion_msgs/msg/motion_command.hpp"
#include "robot_motion_msgs/msg/planned_trajectory.hpp"
#include "robot_motion_msgs/msg/system_state.hpp"

#include "planner_pkg/simple_joint_planner.hpp"

namespace planner_pkg
{

class PlannerNode : public rclcpp::Node
{
public:
  explicit PlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // =========================
  // Topic 回调函数
  // =========================

  // 接收机器人当前关节状态
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 接收来自 motion_api_node 的运动命令
  void on_motion_command(const robot_motion_msgs::msg::MotionCommand::SharedPtr msg);

  // =========================
  // 业务辅助函数
  // =========================

  // 根据 motion_command 构造规划请求
  bool build_plan_request(
    const robot_motion_msgs::msg::MotionCommand & msg,
    SimpleJointPlanner::PlanRequest & request,
    std::string & error_msg);

  // 根据目标关节顺序，从 joint_states 中提取当前关节值
  bool reorder_current_positions(
    const sensor_msgs::msg::JointState & joint_state,
    const std::vector<std::string> & target_joint_names,
    std::vector<double> & reordered_positions);

  // 发布规划结果
  void publish_planned_trajectory(
    const std::string & task_id,
    const trajectory_msgs::msg::JointTrajectory & trajectory);

  // 发布系统状态
  void publish_system_state(
    const std::string & task_id,
    const std::string & state,
    const std::string & message,
    bool is_error);

private:
  // =========================
  // ROS 通信对象
  // =========================

  // 订阅上游运动命令
  rclcpp::Subscription<robot_motion_msgs::msg::MotionCommand>::SharedPtr motion_cmd_sub_;

  // 订阅机器人关节状态
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // 发布规划轨迹
  rclcpp::Publisher<robot_motion_msgs::msg::PlannedTrajectory>::SharedPtr planned_traj_pub_;

  // 发布系统状态
  rclcpp::Publisher<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_pub_;

  // =========================
  // 配置参数
  // =========================

  int traj_points_ {50};             // 轨迹离散点数
  double plan_duration_sec_ {2.0};   // 规划总时长

  // =========================
  // 节点内部成员
  // =========================

  SimpleJointPlanner planner_;                // 简单关节规划器
  sensor_msgs::msg::JointState latest_joint_state_; // 最近一次关节状态
  bool has_joint_state_ {false};              // 是否已收到 joint_states
};

}  // namespace planner_pkg

#endif  // PLANNER_PKG__PLANNER_NODE_HPP_