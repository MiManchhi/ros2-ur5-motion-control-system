#ifndef PLANNER_PKG__PLANNER_NODE_HPP_
#define PLANNER_PKG__PLANNER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_motion_msgs/msg/motion_command.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "planner_pkg/simple_joint_planner.hpp"

class PlannerNode : public rclcpp::Node
{
public:
    explicit PlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void motion_command_callback(const robot_motion_msgs::msg::MotionCommand::SharedPtr msg);

private:
    std::string input_topic_;
    std::string output_topic_;
    double default_duration_sec_;
    int interpolation_points_;

    SimpleJointPlanner planner_;

    rclcpp::Subscription<robot_motion_msgs::msg::MotionCommand>::SharedPtr motion_command_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr planned_traj_pub_;
};

#endif  // PLANNER_PKG__PLANNER_NODE_HPP_