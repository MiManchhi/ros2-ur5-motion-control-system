#include "controller_pkg/trajectory_executor.hpp"

trajectory_msgs::msg::JointTrajectory TrajectoryExecutor::convert_to_command(
  const trajectory_msgs::msg::JointTrajectory & planned_traj) const
{
  // 阶段2中先直接透传
  return planned_traj;
}