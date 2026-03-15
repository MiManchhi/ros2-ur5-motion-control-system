#ifndef CONTROLLER_PKG__CONTROLLER_NODE_HPP_
#define CONTROLLER_PKG__CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "robot_motion_msgs/msg/motion_command.hpp"
#include "robot_motion_msgs/msg/planned_trajectory.hpp"
#include "robot_motion_msgs/msg/system_state.hpp"

#include "controller_pkg/trajectory_executor.hpp"

namespace controller_pkg
{

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 接收规划轨迹
  void on_planned_trajectory(const robot_motion_msgs::msg::PlannedTrajectory::SharedPtr msg);

  // 接收当前关节状态
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 控制定时器回调
  void on_control_timer();

  // 发布关节控制命令
  void publish_joint_command(
    const std::string & task_id,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions);

  // 发布系统状态
  void publish_system_state(
    const std::string & task_id,
    const std::string & state,
    const std::string & message,
    bool is_error);

private:
  rclcpp::Subscription<robot_motion_msgs::msg::PlannedTrajectory>::SharedPtr planned_traj_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<robot_motion_msgs::msg::MotionCommand>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<robot_motion_msgs::msg::SystemState>::SharedPtr system_state_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double control_rate_hz_ {50.0}; // 控制频率

  TrajectoryExecutor executor_;   // 轨迹执行器
};

}  // namespace controller_pkg

#endif  // CONTROLLER_PKG__CONTROLLER_NODE_HPP_