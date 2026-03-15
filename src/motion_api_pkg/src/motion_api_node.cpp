#include "motion_api_pkg/motion_api_node.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace motion_api_pkg
{

MotionApiNode::MotionApiNode(const rclcpp::NodeOptions & options)
: Node("motion_api_node", options)
{
  // =========================
  // 声明并读取参数
  // =========================
  this->declare_parameter<double>("default_speed_scale", 1.0);
  this->declare_parameter<double>("default_timeout_sec", 10.0);
  this->declare_parameter<bool>("enable_preempt", false);

  this->get_parameter("default_speed_scale", default_speed_scale_);
  this->get_parameter("default_timeout_sec", default_timeout_sec_);
  this->get_parameter("enable_preempt", enable_preempt_);

  // =========================
  // 创建发布器
  // =========================
  // 向 planner_node 发布运动命令
  motion_cmd_pub_ = this->create_publisher<robot_motion_msgs::msg::MotionCommand>(
    "/motion_command", 10);

  // =========================
  // 创建订阅器
  // =========================
  // 接收系统状态
  system_state_sub_ = this->create_subscription<robot_motion_msgs::msg::SystemState>(
    "/system_state",
    10,
    std::bind(&MotionApiNode::on_system_state, this, std::placeholders::_1));

  // =========================
  // 创建 Action Server
  // =========================
  action_server_ = rclcpp_action::create_server<MoveJoints>(
    this,
    "/move_joints",
    std::bind(&MotionApiNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionApiNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&MotionApiNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "motion_api_node 已启动");
  RCLCPP_INFO(
    this->get_logger(),
    "参数：default_speed_scale=%.2f, default_timeout_sec=%.2f, enable_preempt=%s",
    default_speed_scale_,
    default_timeout_sec_,
    enable_preempt_ ? "true" : "false");
}

rclcpp_action::GoalResponse MotionApiNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveJoints::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(this->get_logger(), "收到 /move_joints 新任务请求");

  // 先校验任务参数合法性
  std::string reason;
  if (!validate_goal(*goal, reason)) {
    RCLCPP_WARN(this->get_logger(), "任务校验失败：%s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // 当前版本采用“单任务串行执行”策略
  // 如果当前已经有活动任务，则拒绝新任务
  if (active_goal_ctx_.active) {
    if (!enable_preempt_) {
      RCLCPP_WARN(
        this->get_logger(),
        "当前已有活动任务 task_id=%s，拒绝新任务",
        active_goal_ctx_.task_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    // 这里先保留抢占扩展点，但当前版本不真正实现
    RCLCPP_WARN(
      this->get_logger(),
      "enable_preempt=true，但当前版本尚未实现抢占逻辑，仍然拒绝新任务");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // 接受并开始执行
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionApiNode::handle_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  (void)goal_handle;

  RCLCPP_WARN(this->get_logger(), "收到任务取消请求");

  // 当前版本先直接允许取消
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionApiNode::handle_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  // 注意：不要在该回调中做阻塞式耗时操作
  // 因此这里开一个线程去执行任务主流程
  std::thread(
    std::bind(&MotionApiNode::execute_goal, this, std::placeholders::_1),
    goal_handle).detach();
}

void MotionApiNode::on_system_state(const robot_motion_msgs::msg::SystemState::SharedPtr msg)
{
  latest_system_state_ = msg->state;

  // 当前没有活动任务，则不处理
  if (!active_goal_ctx_.active || !active_goal_ctx_.goal_handle) {
    return;
  }

  // 只处理“当前活动任务”的系统状态
  if (msg->task_id != active_goal_ctx_.task_id) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 收到系统状态：state=%s, message=%s, is_error=%s",
    msg->task_id.c_str(),
    msg->state.c_str(),
    msg->message.c_str(),
    msg->is_error ? "true" : "false");

  // 根据系统状态向 action 客户端发送 feedback 或结束任务
  if (msg->state == "PLANNING") {
    active_goal_ctx_.latest_progress = 0.1;

    publish_feedback(
      active_goal_ctx_.goal_handle,
      active_goal_ctx_.task_id,
      "PLANNING",
      active_goal_ctx_.latest_progress,
      active_goal_ctx_.latest_error);

  } else if (msg->state == "RUNNING") {
    // 当前先简化处理：进入 RUNNING 后，将进度更新到至少 0.5
    // 后续 controller_node 完善后，可根据真实执行进度进行更新
    active_goal_ctx_.latest_progress = std::max(active_goal_ctx_.latest_progress, 0.5);

    publish_feedback(
      active_goal_ctx_.goal_handle,
      active_goal_ctx_.task_id,
      "RUNNING",
      active_goal_ctx_.latest_progress,
      active_goal_ctx_.latest_error);

  } else if (msg->state == "FINISHED") {
    finish_goal_success(
      active_goal_ctx_.goal_handle,
      active_goal_ctx_.task_id,
      msg->message,
      active_goal_ctx_.latest_error);

  } else if (msg->state == "ERROR") {
    finish_goal_abort(
      active_goal_ctx_.goal_handle,
      active_goal_ctx_.task_id,
      msg->message,
      active_goal_ctx_.latest_error);

  } else if (msg->state == "CANCELLED") {
    finish_goal_cancel(
      active_goal_ctx_.goal_handle,
      active_goal_ctx_.task_id,
      msg->message,
      active_goal_ctx_.latest_error);
  }
}

bool MotionApiNode::validate_goal(const MoveJoints::Goal & goal, std::string & reason)
{
  // joint_names 不能为空
  if (goal.joint_names.empty()) {
    reason = "joint_names 不能为空";
    return false;
  }

  // target_positions 不能为空
  if (goal.target_positions.empty()) {
    reason = "target_positions 不能为空";
    return false;
  }

  // 长度必须一致
  if (goal.joint_names.size() != goal.target_positions.size()) {
    reason = "joint_names 与 target_positions 长度不一致";
    return false;
  }

  // 当前按 UR5 六关节实现校验
  if (goal.joint_names.size() != 6) {
    reason = "当前实现要求关节数量为 6";
    return false;
  }

  // speed_scale 不能为负
  if (goal.speed_scale < 0.0) {
    reason = "speed_scale 不能小于 0";
    return false;
  }

  // timeout_sec 不能为负
  if (goal.timeout_sec < 0.0) {
    reason = "timeout_sec 不能小于 0";
    return false;
  }

  return true;
}

std::string MotionApiNode::generate_task_id()
{
  // task_id 形式：
  // task_YYYYMMDD_HHMMSS_001
  auto now = this->now();
  auto sec = now.seconds();

  std::time_t time_sec = static_cast<std::time_t>(sec);
  std::tm tm_time {};

#ifdef _WIN32
  localtime_s(&tm_time, &time_sec);
#else
  localtime_r(&time_sec, &tm_time);
#endif

  std::ostringstream oss;
  oss << "task_"
      << std::put_time(&tm_time, "%Y%m%d_%H%M%S")
      << "_"
      << std::setw(3) << std::setfill('0') << (++task_counter_);

  return oss.str();
}

void MotionApiNode::execute_goal(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  // 生成本次任务唯一 ID
  const std::string task_id = generate_task_id();

  // 初始化当前活动任务上下文
  active_goal_ctx_.active = true;
  active_goal_ctx_.task_id = task_id;
  active_goal_ctx_.goal_handle = goal_handle;
  active_goal_ctx_.latest_progress = 0.0;
  active_goal_ctx_.latest_error = 0.0;

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 开始执行任务，task_name=%s",
    task_id.c_str(),
    goal->task_name.c_str());

  // 先向客户端返回一个初始 feedback
  publish_feedback(goal_handle, task_id, "ACCEPTED", 0.0, 0.0);

  // 将任务转成 MotionCommand 发给 planner_node
  publish_motion_command(task_id, *goal);

  // 如果 goal 中未显式给 timeout_sec，则使用默认超时时间
  const double timeout_sec =
    (goal->timeout_sec > 0.0) ? goal->timeout_sec : default_timeout_sec_;

  auto start_time = this->now();
  rclcpp::Rate rate(10.0);

  // 简单轮询等待系统状态推进
  while (rclcpp::ok()) {
    // 如果客户端在执行过程中发起取消
    if (goal_handle->is_canceling()) {
      finish_goal_cancel(goal_handle, task_id, "任务被客户端取消", active_goal_ctx_.latest_error);
      return;
    }

    // 如果当前任务已经被结束函数清理，则退出线程
    if (!active_goal_ctx_.active || active_goal_ctx_.task_id != task_id) {
      return;
    }

    // 超时保护
    const double elapsed = (this->now() - start_time).seconds();
    if (elapsed > timeout_sec) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[task_id=%s] 任务执行超时，timeout=%.2f 秒",
        task_id.c_str(),
        timeout_sec);

      finish_goal_abort(goal_handle, task_id, "任务执行超时", active_goal_ctx_.latest_error);
      return;
    }

    rate.sleep();
  }
}

void MotionApiNode::publish_motion_command(
  const std::string & task_id,
  const MoveJoints::Goal & goal)
{
  robot_motion_msgs::msg::MotionCommand cmd_msg;
  cmd_msg.task_id = task_id;
  cmd_msg.joint_names = goal.joint_names;
  cmd_msg.positions = goal.target_positions;
  cmd_msg.stamp = this->now();

  motion_cmd_pub_->publish(cmd_msg);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 已发布 /motion_command，关节数=%zu",
    task_id.c_str(),
    cmd_msg.joint_names.size());
}

void MotionApiNode::publish_feedback(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
  const std::string & task_id,
  const std::string & current_state,
  double progress,
  double current_error)
{
  auto feedback = std::make_shared<MoveJoints::Feedback>();
  feedback->task_id = task_id;
  feedback->current_state = current_state;
  feedback->progress = progress;
  feedback->current_error = current_error;

  goal_handle->publish_feedback(feedback);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 反馈：state=%s, progress=%.2f, error=%.6f",
    task_id.c_str(),
    current_state.c_str(),
    progress,
    current_error);
}

void MotionApiNode::finish_goal_success(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
  const std::string & task_id,
  const std::string & message,
  double final_error)
{
  auto result = std::make_shared<MoveJoints::Result>();
  result->success = true;
  result->task_id = task_id;
  result->message = message;
  result->final_error = final_error;

  goal_handle->succeed(result);

  RCLCPP_INFO(
    this->get_logger(),
    "[task_id=%s] 任务成功结束：%s，final_error=%.6f",
    task_id.c_str(),
    message.c_str(),
    final_error);

  reset_active_goal_context();
}

void MotionApiNode::finish_goal_abort(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
  const std::string & task_id,
  const std::string & message,
  double final_error)
{
  auto result = std::make_shared<MoveJoints::Result>();
  result->success = false;
  result->task_id = task_id;
  result->message = message;
  result->final_error = final_error;

  goal_handle->abort(result);

  RCLCPP_ERROR(
    this->get_logger(),
    "[task_id=%s] 任务失败结束：%s，final_error=%.6f",
    task_id.c_str(),
    message.c_str(),
    final_error);

  reset_active_goal_context();
}

void MotionApiNode::finish_goal_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle,
  const std::string & task_id,
  const std::string & message,
  double final_error)
{
  auto result = std::make_shared<MoveJoints::Result>();
  result->success = false;
  result->task_id = task_id;
  result->message = message;
  result->final_error = final_error;

  goal_handle->canceled(result);

  RCLCPP_WARN(
    this->get_logger(),
    "[task_id=%s] 任务取消结束：%s，final_error=%.6f",
    task_id.c_str(),
    message.c_str(),
    final_error);

  reset_active_goal_context();
}

void MotionApiNode::reset_active_goal_context()
{
  active_goal_ctx_.active = false;
  active_goal_ctx_.task_id.clear();
  active_goal_ctx_.goal_handle.reset();
  active_goal_ctx_.latest_progress = 0.0;
  active_goal_ctx_.latest_error = 0.0;
}

}  // namespace motion_api_pkg

// 如果需要组件化加载节点，则取消下面的注释，并在 CMakeLists.txt 中添加相关配置
// 组件化加载不需要main函数，直接由 rclcpp_components 管理
// 如果不需要组件化加载，则可以在 main.cpp 中直接创建并运行 MotionApiNode 实例

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(motion_api_pkg::MotionApiNode)