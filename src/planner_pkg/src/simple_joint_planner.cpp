#include "planner_pkg/simple_joint_planner.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

trajectory_msgs::msg::JointTrajectory SimpleJointPlanner::plan(
    const std::string & task_id,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & target_positions,
    double duration_sec,
    int interpolation_points) const
{
  (void)task_id;

  if (joint_names.empty()) {
    throw std::runtime_error("joint_names is empty");
  }

  if (joint_names.size() != target_positions.size()) {
    throw std::runtime_error("joint_names size does not match target_positions size");
  }

  if (duration_sec <= 0.0) {
    throw std::runtime_error("duration_sec must be > 0");
  }

  if (interpolation_points < 2) {
    throw std::runtime_error("interpolation_points must be >= 2");
  }

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joint_names;
  traj.points.reserve(static_cast<size_t>(interpolation_points));

  for (int i = 0; i < interpolation_points; ++i) {
    const double ratio =
      static_cast<double>(i + 1) / static_cast<double>(interpolation_points);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(target_positions.size());
    point.velocities.resize(target_positions.size(), 0.0);
    point.accelerations.resize(target_positions.size(), 0.0);

    for (const auto & target : target_positions) {
      point.positions.push_back(target * ratio);
    }

    const double time_sec = duration_sec * ratio;
    point.time_from_start.sec = static_cast<int32_t>(time_sec);
    point.time_from_start.nanosec =
      static_cast<uint32_t>((time_sec - static_cast<double>(point.time_from_start.sec)) * 1e9);

    traj.points.push_back(point);
  }

  return traj;
}