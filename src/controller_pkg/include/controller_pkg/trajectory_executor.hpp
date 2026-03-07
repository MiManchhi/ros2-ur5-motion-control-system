#ifndef CONTROLLER_PKG__TRAJECTORY_EXECUTOR_HPP_
#define CONTROLLER_PKG__TRAJECTORY_EXECUTOR_HPP_

#include "trajectory_msgs/msg/joint_trajectory.hpp"

class TrajectoryExecutor
{
public:
  TrajectoryExecutor() = default;

  // 阶段2最小实现：
  // 直接返回输入轨迹，后续阶段3再扩展为真正的轨迹执行/重定时/分段输出逻辑
  trajectory_msgs::msg::JointTrajectory convert_to_command(
    const trajectory_msgs::msg::JointTrajectory & planned_traj) const;
};

#endif  // CONTROLLER_PKG__TRAJECTORY_EXECUTOR_HPP_