#include "robot_interface_pkg/ur_gazebo_backend.hpp"

trajectory_msgs::msg::JointTrajectory UrGazeboBackend::convert_to_sim_command(
  const trajectory_msgs::msg::JointTrajectory & joint_cmd) const
{
  // 阶段2中先直接透传
  // 后续阶段3/4中，这里可以加入：
  // 1. 关节名重映射
  // 2. 速度/加速度补全
  // 3. 安全限幅
  // 4. 真机驱动适配
  return joint_cmd;
}