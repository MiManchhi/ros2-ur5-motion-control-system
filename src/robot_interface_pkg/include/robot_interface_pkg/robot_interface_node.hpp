#ifndef ROBOT_INTERFACE_PKG__ROBOT_INTERFACE_NODE_HPP_
#define ROBOT_INTERFACE_PKG__ROBOT_INTERFACE_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "robot_motion_msgs/msg/motion_command.hpp"
#include "robot_motion_msgs/msg/system_state.hpp"

#include "robot_interface_pkg/ur_gazebo_backend.hpp"

namespace robot_interface_pkg
{

class RobotInterfaceNode : public rclcpp::Node
{
public:
  explicit RobotInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 接收上层控制命令
  void on_joint_command(const robot_motion_msgs::msg::MotionCommand::SharedPtr msg);

  // 发布系统状态
  void publish_system_state(
    const std::string & task_id,
    const std::string & state,
    const std::string & message,
    bool is_error);

private:
  // 订阅上层统一控制命令
  rclcpp::Subscription<robot_motion_msgs::msg::MotionCommand>::SharedPtr joint_cmd_sub_;

  // 发布到底层 Gazebo / ros2_control 控制器
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr controller_cmd_pub_;

  // 发布系统状态
  rclcpp::Publisher<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_pub_;

  // 后端适配器
  UrGazeboBackend backend_;
};

}  // namespace robot_interface_pkg

#endif  // ROBOT_INTERFACE_PKG__ROBOT_INTERFACE_NODE_HPP_