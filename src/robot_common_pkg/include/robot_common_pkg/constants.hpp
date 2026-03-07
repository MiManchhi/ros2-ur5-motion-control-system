#ifndef ROBOT_COMMON_PKG__CONSTANTS_HPP_
#define ROBOT_COMMON_PKG__CONSTANTS_HPP_

namespace robot_common_pkg::constants
{

namespace state
{
inline constexpr const char * kIdle      = "idle";
inline constexpr const char * kAccepted  = "accepted";
inline constexpr const char * kRejected  = "rejected";
inline constexpr const char * kPlanning  = "planning";
inline constexpr const char * kExecuting = "executing";
inline constexpr const char * kCompleted = "completed";
inline constexpr const char * kCanceled  = "canceled";
inline constexpr const char * kFailed    = "failed";
inline constexpr const char * kError     = "error";
inline constexpr const char * kResetting = "resetting";
}  // namespace state

namespace event
{
inline constexpr const char * kTaskReceived     = "task_received";
inline constexpr const char * kGoalRejected     = "goal_rejected";
inline constexpr const char * kPlanningStarted  = "planning_started";
inline constexpr const char * kPlanningDone     = "planning_done";
inline constexpr const char * kExecutionStarted = "execution_started";
inline constexpr const char * kExecutionDone    = "execution_done";
inline constexpr const char * kExecutionFailed  = "execution_failed";
inline constexpr const char * kTaskCanceled     = "task_canceled";
inline constexpr const char * kSystemReset      = "system_reset";
}  // namespace event

namespace module
{
inline constexpr const char * kMotionApi      = "motion_api";
inline constexpr const char * kPlanner        = "planner";
inline constexpr const char * kController     = "controller";
inline constexpr const char * kRobotInterface = "robot_interface";
inline constexpr const char * kSystemManager  = "system_manager";
}  // namespace module

}  // namespace robot_common_pkg::constants

#endif  // ROBOT_COMMON_PKG__CONSTANTS_HPP_