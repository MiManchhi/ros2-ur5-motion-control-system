#ifndef CONTROLLER_PKG__CONTROLLER_NODE_HPP_
#define CONTROLLER_PKG__CONTROLLER_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "robot_motion_msgs/msg/motion_command.hpp"
#include "robot_motion_msgs/msg/motion_event.hpp"
#include "robot_motion_msgs/msg/planned_trajectory.hpp"
#include "robot_motion_msgs/msg/task_state.hpp"

#include "controller_pkg/trajectory_executor.hpp"

namespace controller_pkg
{

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 收到规划层输出的轨迹
  void on_planned_trajectory(
    const robot_motion_msgs::msg::PlannedTrajectory::SharedPtr msg);

  // 收到底层 joint_states
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 收到正式任务状态
  void on_task_state(const robot_motion_msgs::msg::TaskState::SharedPtr msg);

  // 控制定时器回调
  void on_control_timer();

  // 发布内部关节控制命令
  void publish_joint_command(
    const std::string & task_id,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions);

  // 发布任务事件
  void publish_motion_event(
    const std::string & task_id,
    const std::string & event_name,
    const std::string & related_state,
    const std::string & detail,
    float progress,
    double current_error,
    bool is_error);

  // 计算当前任务进度
  float compute_progress() const;

private:
  // 订阅规划轨迹
  rclcpp::Subscription<robot_motion_msgs::msg::PlannedTrajectory>::SharedPtr planned_traj_sub_;

  // 订阅底层关节状态
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // 订阅正式任务状态
  rclcpp::Subscription<robot_motion_msgs::msg::TaskState>::SharedPtr task_state_sub_;

  // 发布控制命令
  rclcpp::Publisher<robot_motion_msgs::msg::MotionCommand>::SharedPtr joint_cmd_pub_;

  // 发布任务事件
  rclcpp::Publisher<robot_motion_msgs::msg::MotionEvent>::SharedPtr motion_event_pub_;

  // 控制定时器
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 控制频率
  double control_rate_hz_ {50.0};

  // 轨迹执行器
  TrajectoryExecutor executor_;
};

}  // namespace controller_pkg

#endif  // CONTROLLER_PKG__CONTROLLER_NODE_HPP_