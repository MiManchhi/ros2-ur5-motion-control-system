#ifndef PLANNER_PKG__SIMPLE_JOINT_PLANNER_HPP_
#define PLANNER_PKG__SIMPLE_JOINT_PLANNER_HPP_

#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

class SimpleJointPlanner
{
public:
    SimpleJointPlanner() = default;

    trajectory_msgs::msg::JointTrajectory plan(
    const std::string & task_id,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & target_positions,
    double duration_sec,
    int interpolation_points) const;
};

#endif  // PLANNER_PKG__SIMPLE_JOINT_PLANNER_HPP_