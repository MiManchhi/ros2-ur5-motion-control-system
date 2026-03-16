#ifndef ROBOT_INTERFACE_PKG__ROBOT_INTERFACE_NODE_HPP_
#define ROBOT_INTERFACE_PKG__ROBOT_INTERFACE_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "robot_motion_msgs/msg/motion_command.hpp"
#include "robot_motion_msgs/msg/motion_event.hpp"

#include "robot_interface_pkg/ur_gazebo_backend.hpp"

namespace robot_interface_pkg
{

class RobotInterfaceNode : public rclcpp::Node
{
public:
  explicit RobotInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 接收控制层发来的内部统一控制命令
  void on_joint_command(const robot_motion_msgs::msg::MotionCommand::SharedPtr msg);

  // 发布任务事件到 system_manager_node
  void publish_motion_event(
    const std::string & task_id,
    const std::string & event_name,
    const std::string & related_state,
    const std::string & detail,
    float progress,
    double current_error,
    bool is_error);

private:
  // 订阅控制层输出的关节控制命令
  rclcpp::Subscription<robot_motion_msgs::msg::MotionCommand>::SharedPtr joint_cmd_sub_;

  // 发布到底层控制器的话题
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr controller_cmd_pub_;

  // 发布任务事件
  rclcpp::Publisher<robot_motion_msgs::msg::MotionEvent>::SharedPtr motion_event_pub_;

  // Gazebo / ros2_control 后端适配器
  UrGazeboBackend backend_;
};

}  // namespace robot_interface_pkg

#endif  // ROBOT_INTERFACE_PKG__ROBOT_INTERFACE_NODE_HPP_