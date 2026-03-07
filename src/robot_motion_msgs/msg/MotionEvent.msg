# 所属任务唯一标识
string task_id
# 事件来源模块名称，如 motion_api / planner / controller / interface
string module_name
# 事件名称，如 task_received / planning_done / execution_start / execution_failed
string event_name
# 事件详细说明，可记录错误原因或补充信息
string detail
# 事件发生时间
builtin_interfaces/Time stamp