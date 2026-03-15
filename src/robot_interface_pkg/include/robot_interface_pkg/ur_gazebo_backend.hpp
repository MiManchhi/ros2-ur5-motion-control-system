#ifndef ROBOT_INTERFACE_PKG__UR_GAZEBO_BACKEND_HPP_
#define ROBOT_INTERFACE_PKG__UR_GAZEBO_BACKEND_HPP_

#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "robot_motion_msgs/msg/motion_command.hpp"

namespace robot_interface_pkg
{

// UR Gazebo 后端适配器
// 负责将系统内部统一控制命令 MotionCommand
// 转换为 Gazebo / ros2_control 可接受的 JointTrajectory
class UrGazeboBackend
{
public:
  // 后端配置
  struct Config
  {
    std::string controller_topic {"/joint_trajectory_controller/joint_trajectory"}; // 底层控制器话题
    double point_time_from_start_sec {0.1};  // 单个命令点默认到达时间
  };

public:
  UrGazeboBackend() = default;
  ~UrGazeboBackend() = default;

  void set_config(const Config & config);

  // 将内部统一控制命令转换为 JointTrajectory
  bool convert_command(
    const robot_motion_msgs::msg::MotionCommand & cmd,
    trajectory_msgs::msg::JointTrajectory & traj,
    std::string & error_msg) const;

  // 获取底层控制器话题
  const std::string & controller_topic() const;

private:
  Config config_;
};

}  // namespace robot_interface_pkg

#endif  // ROBOT_INTERFACE_PKG__UR_GAZEBO_BACKEND_HPP_