#include "planner_pkg/planner_node.hpp"

#include <unordered_map>

#include "robot_common_pkg/constants.hpp"

namespace planner_pkg
{

namespace c = robot_common_pkg::constants;

PlannerNode::PlannerNode(const rclcpp::NodeOptions & options)
: Node("planner_node", options)
{
  // =========================
  // 声明并读取参数
  // =========================
  this->declare_parameter<int>("traj_points", 50);
  this->declare_parameter<double>("plan_duration_sec", 5.0);

  this->get_parameter("traj_points", traj_points_);
  this->get_parameter("plan_duration_sec", plan_duration_sec_);

  if (traj_points_ <= 1) {
    traj_points_ = 50;
  }

  if (plan_duration_sec_ <= 0.0) {
    plan_duration_sec_ = 5.0;
  }

  // =========================
  // 创建订阅器
  // =========================

  // 接收 motion_api_node 下发的内部运动任务
  motion_cmd_sub_ =
    this->create_subscription<robot_motion_msgs::msg::MotionCommand>(
      "/motion_command",
      10,
      std::bind(&PlannerNode::on_motion_command, this, std::placeholders::_1));

  // 接收底层 joint_states
  joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      50,
      std::bind(&PlannerNode::on_joint_state, this, std::placeholders::_1));

  // =========================
  // 创建发布器
  // =========================

  // 向控制层发布规划轨迹
  planned_traj_pub_ =
    this->create_publisher<robot_motion_msgs::msg::PlannedTrajectory>(
      "/planned_traj",
      10);

  // 向 system_manager_node 发布任务事件
  motion_event_pub_ =
    this->create_publisher<robot_motion_msgs::msg::MotionEvent>(
      "/motion_event",
      20);

  RCLCPP_INFO(this->get_logger(), "planner_node 已启动");
  RCLCPP_INFO(
    this->get_logger(),
    "参数：traj_points=%d, plan_duration_sec=%.2f",
    traj_points_,
    plan_duration_sec_);
}

void PlannerNode::on_motion_command(
  const robot_motion_msgs::msg::MotionCommand::SharedPtr msg)
{
  const std::string & task_id = msg->task_id;

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 收到 /motion_command，准备进行轨迹规划",
    task_id.c_str());

  // 先发布“开始规划”事件
  publish_motion_event(
    task_id,
    c::event::kPlanningStarted,
    c::task_state::kPlanning,
    "开始轨迹规划",
    0.10F,
    0.0,
    false);

  // 当前还没有 joint_states，无法规划
  if (!has_joint_state_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 尚未收到 /joint_states，无法规划",
      task_id.c_str());

    publish_motion_event(
      task_id,
      c::event::kPlanningFailed,
      c::task_state::kFailed,
      "尚未收到 /joint_states，无法规划",
      0.10F,
      0.0,
      true);
    return;
  }

  // 从 joint_states 中提取当前关节位置，顺序与目标关节保持一致
  std::vector<double> current_positions;
  if (!extract_current_positions(msg->joint_names, latest_joint_state_, current_positions)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 无法根据目标关节顺序提取当前位置",
      task_id.c_str());

    publish_motion_event(
      task_id,
      c::event::kPlanningFailed,
      c::task_state::kFailed,
      "无法根据目标关节顺序提取当前位置",
      0.10F,
      0.0,
      true);
    return;
  }

  // 先将规划参数写入规划器内部
  // 当前 SimpleJointPlanner 使用类内部参数 traj_points_ / plan_duration_sec_
  planner_.set_traj_points(traj_points_);
  planner_.set_plan_duration(plan_duration_sec_);

  // 构造规划请求
  SimpleJointPlanner::PlanRequest request;
  request.task_id = task_id;
  request.joint_names = msg->joint_names;
  request.current_positions = current_positions;
  request.target_positions = msg->positions;

  // 当前 MotionCommand 里还没有 speed_scale / timeout_sec 透传字段
  // 这里先采用默认值占位，后续可按需要扩展 msg 定义
  request.speed_scale = 1.0;
  request.timeout_sec = 0.0;

  // 调用规划器
  const auto result = planner_.plan(request);

  if (!result.success) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[task_id=%s] 轨迹规划失败：%s",
      task_id.c_str(),
      result.error_msg.c_str());

    publish_motion_event(
      task_id,
      c::event::kPlanningFailed,
      c::task_state::kFailed,
      result.error_msg,
      0.10F,
      0.0,
      true);
    return;
  }

  // 发布规划结果
  publish_planned_trajectory(task_id, result.trajectory);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 轨迹规划成功，轨迹点数=%zu",
    task_id.c_str(),
    result.trajectory.points.size());

  // 发布“规划完成”事件
  publish_motion_event(
    task_id,
    c::event::kPlanningDone,
    c::task_state::kPlanning,
    "轨迹规划完成，等待控制层启动执行",
    0.20F,
    0.0,
    false);
}

void PlannerNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  latest_joint_state_ = *msg;
  has_joint_state_ = true;
}

bool PlannerNode::extract_current_positions(
  const std::vector<std::string> & target_joint_names,
  const sensor_msgs::msg::JointState & joint_state,
  std::vector<double> & reordered_positions) const
{
  // joint_state.name 和 joint_state.position 数量不一致，数据非法
  if (joint_state.name.size() != joint_state.position.size()) {
    return false;
  }

  // 构造 name -> position 的映射
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

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 已发布 /planned_traj，轨迹点数=%zu",
    task_id.c_str(),
    trajectory.points.size());
}

void PlannerNode::publish_motion_event(
  const std::string & task_id,
  const std::string & event_name,
  const std::string & related_state,
  const std::string & detail,
  float progress,
  double current_error,
  bool is_error)
{
  robot_motion_msgs::msg::MotionEvent msg;
  msg.task_id = task_id;
  msg.module_name = c::module::kPlanner;
  msg.event_name = event_name;
  msg.related_state = related_state;
  msg.detail = detail;
  msg.progress = progress;
  msg.current_error = current_error;
  msg.is_error = is_error;
  msg.stamp = this->now();

  motion_event_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[motion_event] task_id=%s, module=%s, event=%s, related_state=%s, progress=%.3f, error=%.6f, is_error=%s, detail=%s",
    task_id.c_str(),
    c::module::kPlanner,
    event_name.c_str(),
    related_state.c_str(),
    progress,
    current_error,
    is_error ? "true" : "false",
    detail.c_str());
}

}  // namespace planner_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planner_pkg::PlannerNode)