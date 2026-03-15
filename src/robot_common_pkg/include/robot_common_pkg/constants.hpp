#ifndef ROBOT_COMMON_PKG__CONSTANTS_HPP_
#define ROBOT_COMMON_PKG__CONSTANTS_HPP_

#include <string>

namespace robot_common_pkg::constants
{

// ============================================================
// 任务状态常量
// 这些状态与某个 task_id 绑定，用于描述任务生命周期
// ============================================================
namespace task_state
{
inline constexpr const char * kAccepted  = "accepted";
inline constexpr const char * kPlanning  = "planning";
inline constexpr const char * kExecuting = "executing";
inline constexpr const char * kCompleted = "completed";
inline constexpr const char * kCanceled  = "canceled";
inline constexpr const char * kFailed    = "failed";
inline constexpr const char * kRejected  = "rejected";
}  // namespace task_state

// ============================================================
// 系统状态常量
// 这些状态描述的是整个系统，而不是某个具体任务
// ============================================================
namespace system_state
{
inline constexpr const char * kInit      = "init";
inline constexpr const char * kIdle      = "idle";
inline constexpr const char * kBusy      = "busy";
inline constexpr const char * kResetting = "resetting";
inline constexpr const char * kError     = "error";
}  // namespace system_state

// ============================================================
// 事件名称常量
// 事件表示“发生了什么”
// 由各业务节点上报，供 system_manager_node 收敛为正式 task_state
// ============================================================
namespace event
{
inline constexpr const char * kTaskReceived     = "task_received";
inline constexpr const char * kGoalRejected     = "goal_rejected";
inline constexpr const char * kPlanningStarted  = "planning_started";
inline constexpr const char * kPlanningDone     = "planning_done";
inline constexpr const char * kPlanningFailed   = "planning_failed";
inline constexpr const char * kExecutionStarted = "execution_started";
inline constexpr const char * kExecutionDone    = "execution_done";
inline constexpr const char * kExecutionFailed  = "execution_failed";
inline constexpr const char * kTaskCanceled     = "task_canceled";
inline constexpr const char * kSystemReset      = "system_reset";
}  // namespace event

// ============================================================
// 模块名称常量
// 统一各节点的模块命名，避免日志、事件和状态中出现多套写法
// ============================================================
namespace module
{
inline constexpr const char * kMotionApi      = "motion_api";
inline constexpr const char * kPlanner        = "planner";
inline constexpr const char * kController     = "controller";
inline constexpr const char * kRobotInterface = "robot_interface";
inline constexpr const char * kSystemManager  = "system_manager";
}  // namespace module

// ============================================================
// 任务状态辅助函数
// ============================================================

// 判断某个任务状态是否为终态
inline bool is_task_terminal_state(const std::string & state)
{
  return state == task_state::kCompleted ||
         state == task_state::kCanceled ||
         state == task_state::kFailed ||
         state == task_state::kRejected;
}

// 判断某个任务状态是否仍然处于活动阶段
inline bool is_task_active_state(const std::string & state)
{
  return state == task_state::kAccepted ||
         state == task_state::kPlanning ||
         state == task_state::kExecuting;
}

// 判断某个任务状态是否属于异常态
inline bool is_task_error_state(const std::string & state)
{
  return state == task_state::kFailed ||
         state == task_state::kRejected;
}

// ============================================================
// 系统状态辅助函数
// ============================================================

// 判断系统当前是否繁忙
inline bool is_system_busy_state(const std::string & state)
{
  return state == system_state::kBusy;
}

// 判断系统当前是否为空闲
inline bool is_system_idle_state(const std::string & state)
{
  return state == system_state::kIdle;
}

// 判断系统当前是否处于可恢复异常态
inline bool is_system_error_state(const std::string & state)
{
  return state == system_state::kError;
}

// ============================================================
// 事件与目标任务状态映射辅助函数
// 便于 system_manager_node 根据事件快速推导目标状态
// ============================================================

// 根据事件名称推导对应的任务状态
// 若无对应状态，则返回空字符串
inline std::string event_to_task_state(const std::string & event_name)
{
  if (event_name == event::kTaskReceived) {
    return task_state::kAccepted;
  }
  if (event_name == event::kPlanningStarted) {
    return task_state::kPlanning;
  }
  if (event_name == event::kExecutionStarted) {
    return task_state::kExecuting;
  }
  if (event_name == event::kExecutionDone) {
    return task_state::kCompleted;
  }
  if (event_name == event::kPlanningFailed ||
      event_name == event::kExecutionFailed) {
    return task_state::kFailed;
  }
  if (event_name == event::kTaskCanceled) {
    return task_state::kCanceled;
  }
  if (event_name == event::kGoalRejected) {
    return task_state::kRejected;
  }

  return "";
}

}  // namespace robot_common_pkg::constants

#endif  // ROBOT_COMMON_PKG__CONSTANTS_HPP_