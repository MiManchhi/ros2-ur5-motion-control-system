#ifndef CONTROLLER_PKG__CONTROLLER_NODE_HPP_
#define CONTROLLER_PKG__CONTROLLER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "controller_pkg/trajectory_executor.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 接收规划器输出轨迹后的回调
  void planned_traj_callback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

private:
  // 输入输出 topic 名称
  std::string planned_traj_topic_;
  std::string joint_cmd_topic_;

  // 轨迹执行器（阶段2先做简单透传）
  TrajectoryExecutor executor_;

  // 订阅规划轨迹
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr planned_traj_sub_;

  // 发布控制命令
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub_;
};

#endif  // CONTROLLER_PKG__CONTROLLER_NODE_HPP_