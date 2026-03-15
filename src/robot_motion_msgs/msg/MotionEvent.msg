# ==========================================
# MotionEvent.msg
# 任务事件消息
# 用于业务节点向 system_manager_node 上报“发生了什么”
# 它描述的是事件，而不是正式状态
# ==========================================

# 任务唯一标识
# 对于系统级事件（如 system_reset）可为空
string task_id

# 事件来源模块
# 建议取值：
# motion_api / planner / controller / robot_interface / system_manager
string module_name

# 事件名称
# 固定取值：
# task_received
# goal_rejected
# planning_started
# planning_done
# planning_failed
# execution_started
# execution_done
# execution_failed
# task_canceled
# system_reset
string event_name

# 该事件对应或期望推进到的任务状态
# 例如：
# task_received    -> accepted
# planning_started -> planning
# execution_started-> executing
# execution_done   -> completed
# planning_failed  -> failed
# task_canceled    -> canceled
string related_state

# 事件细节说明
# 比 message 更偏“发生了什么”
string detail

# 事件发生时的进度
# 非必须，但有利于 system_manager 统一生成 task_state
float32 progress

# 事件发生时的当前误差
# 非执行阶段可为 0
float64 current_error

# 是否为异常事件
bool is_error

# 时间戳
builtin_interfaces/Time stamp