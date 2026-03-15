#include "planner_pkg/planner_node.hpp"

#include <unordered_map>

namespace planner_pkg
{

PlannerNode::PlannerNode(const rclcpp::NodeOptions & options)
: Node("planner_node", options)
{
  // =========================
  // 声明并读取参数
  // =========================
  this->declare_parameter<int>("traj_points", 50);
  this->declare_parameter<double>("plan_duration_sec", 2.0);

  this->get_parameter("traj_points", traj_points_);
  this->get_parameter("plan_duration_sec", plan_duration_sec_);

  // 对参数进行基本保护
  if (traj_points_ <= 1) {
    traj_points_ = 2;
  }

  if (plan_duration_sec_ <= 0.0) {
    plan_duration_sec_ = 2.0;
  }

  // 将参数同步给内部简单规划器
  planner_.set_traj_points(traj_points_);
  planner_.set_plan_duration(plan_duration_sec_);

  // =========================
  // 创建订阅器
  // =========================

  // 订阅运动命令
  motion_cmd_sub_ = this->create_subscription<robot_motion_msgs::msg::MotionCommand>(
    "/motion_command",
    10,
    std::bind(&PlannerNode::on_motion_command, this, std::placeholders::_1));

  // 订阅机器人当前关节状态
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    50,
    std::bind(&PlannerNode::on_joint_state, this, std::placeholders::_1));

  // =========================
  // 创建发布器
  // =========================

  // 发布规划结果
  planned_traj_pub_ = this->create_publisher<robot_motion_msgs::msg::PlannedTrajectory>(
    "/planned_traj", 10);

  // 发布系统状态
  system_state_pub_ = this->create_publisher<robot_motion_msgs::msg::SystemState>(
    "/system_state_raw", 10);

  RCLCPP_INFO(this->get_logger(), "planner_node 已启动");
  RCLCPP_INFO(
    this->get_logger(),
    "参数：traj_points=%d, plan_duration_sec=%.2f",
    traj_points_,
    plan_duration_sec_);
}

void PlannerNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 缓存最近一次机器人关节状态，供规划时作为轨迹起点使用
  latest_joint_state_ = *msg;
  has_joint_state_ = true;
}

void PlannerNode::on_motion_command(const robot_motion_msgs::msg::MotionCommand::SharedPtr msg)
{
  const std::string & task_id = msg->task_id;

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 收到 /motion_command，开始规划",
    task_id.c_str());

  // 先发布“规划中”状态
  publish_system_state(task_id, "PLANNING", "开始轨迹规划", false);

  // 构造规划请求
  SimpleJointPlanner::PlanRequest request;
  std::string error_msg;
  if (!build_plan_request(*msg, request, error_msg)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 构造规划请求失败：%s",
      task_id.c_str(),
      error_msg.c_str());

    publish_system_state(task_id, "ERROR", error_msg, true);
    return;
  }

  // 调用内部简单规划器执行规划
  SimpleJointPlanner::PlanResult result = planner_.plan(request);
  if (!result.success) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 轨迹规划失败：%s",
      task_id.c_str(),
      result.error_msg.c_str());

    publish_system_state(task_id, "ERROR", result.error_msg, true);
    return;
  }

  // 发布规划结果
  publish_planned_trajectory(task_id, result.trajectory);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 已发布 /planned_traj，轨迹点数=%zu",
    task_id.c_str(),
    result.trajectory.points.size());

  // 当前版本先简化为：规划成功后发布 RUNNING
  // 更规范的做法是后续由 controller_node 在真正开始执行时发布 RUNNING
  // publish_system_state(task_id, "RUNNING", "轨迹规划完成，准备执行", false);
}

bool PlannerNode::build_plan_request(
  const robot_motion_msgs::msg::MotionCommand & msg,
  SimpleJointPlanner::PlanRequest & request,
  std::string & error_msg)
{
  // 必须先收到 joint_states，才能知道当前起始位置
  if (!has_joint_state_) {
    error_msg = "尚未收到 /joint_states，无法规划";
    return false;
  }

  // 基本输入校验
  if (msg.task_id.empty()) {
    error_msg = "task_id 为空";
    return false;
  }

  if (msg.joint_names.empty()) {
    error_msg = "joint_names 为空";
    return false;
  }

  if (msg.positions.empty()) {
    error_msg = "positions 为空";
    return false;
  }

  if (msg.joint_names.size() != msg.positions.size()) {
    error_msg = "joint_names 与 positions 数量不一致";
    return false;
  }

  // 按目标关节顺序提取当前关节位置
  std::vector<double> current_positions;
  if (!reorder_current_positions(latest_joint_state_, msg.joint_names, current_positions)) {
    error_msg = "joint_states 中缺少目标关节，无法规划";
    return false;
  }

  // 填充规划输入
  request.task_id = msg.task_id;
  request.joint_names = msg.joint_names;
  request.current_positions = current_positions;
  request.target_positions = msg.positions;

  // 当前 MotionCommand.msg 中尚未定义 speed_scale 和 timeout_sec
  // 这里先保留默认值，后续如扩展消息字段再接入
  request.speed_scale = 1.0;
  request.timeout_sec = 10.0;

  return true;
}

bool PlannerNode::reorder_current_positions(
  const sensor_msgs::msg::JointState & joint_state,
  const std::vector<std::string> & target_joint_names,
  std::vector<double> & reordered_positions)
{
  // name 和 position 必须一一对应
  if (joint_state.name.size() != joint_state.position.size()) {
    return false;
  }

  // 构造“关节名 -> 关节值”映射
  std::unordered_map<std::string, double> joint_map;
  joint_map.reserve(joint_state.name.size());

  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    joint_map[joint_state.name[i]] = joint_state.position[i];
  }

  reordered_positions.clear();
  reordered_positions.reserve(target_joint_names.size());

  // 按目标关节顺序提取当前位置
  for (const auto & joint_name : target_joint_names) {
    auto it = joint_map.find(joint_name);
    if (it == joint_map.end()) {
      return false;
    }
    reordered_positions.push_back(it->second);
  }

  return true;
}

void PlannerNode::publish_planned_trajectory(
  const std::string & task_id,
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  robot_motion_msgs::msg::PlannedTrajectory msg;
  msg.task_id = task_id;
  msg.trajectory = trajectory;
  msg.stamp = this->now();

  planned_traj_pub_->publish(msg);
}

void PlannerNode::publish_system_state(
  const std::string & task_id,
  const std::string & state,
  const std::string & message,
  bool is_error)
{
  robot_motion_msgs::msg::SystemState msg;
  msg.task_id = task_id;
  msg.state = state;
  msg.message = message;
  msg.is_error = is_error;
  msg.stamp = this->now();

  system_state_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 发布 /system_state：state=%s, message=%s, is_error=%s",
    task_id.c_str(),
    state.c_str(),
    message.c_str(),
    is_error ? "true" : "false");
}

} // namespace planner_pkg

// 如果需要组件化加载节点，则取消下面的注释，并在 CMakeLists.txt 中添加相关配置
// 组件化加载不需要main函数，直接由 rclcpp_components 管理
// 如果不需要组件化加载，则可以在 main.cpp 中直接创建并运行 MotionApiNode 实例

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(planner_pkg::PlannerNode)