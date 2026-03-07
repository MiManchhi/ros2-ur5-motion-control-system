#ifndef ROBOT_INTERFACE_PKG__UR_GAZEBO_BACKEND_HPP_
#define ROBOT_INTERFACE_PKG__UR_GAZEBO_BACKEND_HPP_

#include "trajectory_msgs/msg/joint_trajectory.hpp"

class UrGazeboBackend
{
public:
  UrGazeboBackend() = default;

  // 将系统内部 joint_cmd 转换为 Gazebo/ros2_control 可接受的控制命令
  // 阶段2中两边都使用 JointTrajectory，因此先直接透传
  trajectory_msgs::msg::JointTrajectory convert_to_sim_command(
    const trajectory_msgs::msg::JointTrajectory & joint_cmd) const;
};

#endif  // ROBOT_INTERFACE_PKG__UR_GAZEBO_BACKEND_HPP_