#ifndef ROBOT_INTERFACE_PKG__ROBOT_INTERFACE_NODE_HPP_
#define ROBOT_INTERFACE_PKG__ROBOT_INTERFACE_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "robot_interface_pkg/ur_gazebo_backend.hpp"

class RobotInterfaceNode : public rclcpp::Node
{
public:
  explicit RobotInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 接收内部 joint_cmd 后的处理函数
  void joint_cmd_callback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

private:
  // 输入 topic：系统内部统一控制命令
  std::string joint_cmd_topic_;

  // 输出 topic：仿真控制器命令 topic
  std::string sim_command_topic_;

  // 仿真后端适配器
  UrGazeboBackend backend_;

  // 订阅内部 joint_cmd
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_sub_;

  // 发布到 ros2_control / Gazebo 的控制命令
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr sim_command_pub_;
};

#endif  // ROBOT_INTERFACE_PKG__ROBOT_INTERFACE_NODE_HPP_