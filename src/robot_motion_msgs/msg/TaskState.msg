# ================================
# TaskState.msg
# 任务正式状态消息
# 用于对外广播某个 task_id 当前所处的生命周期阶段
# ================================

# 任务唯一标识
string task_id

# 当前任务状态
# 固定取值：
# accepted / planning / executing / completed / canceled / failed / rejected
string state

# 产生或确认该状态的模块名称
# 固定取值：
# motion_api / planner / controller / robot_interface / system_manager
string source_module

# 状态说明信息
# 例如：
# "任务已接收"
# "开始轨迹规划"
# "控制器开始执行"
# "接口适配失败"
string message

# 任务当前进度，范围建议 [0.0, 1.0]
# accepted 阶段可为 0.0~0.05
# planning 阶段可为 0.1 左右
# executing 阶段由控制层真实更新
# completed 时应为 1.0
float32 progress

# 当前误差
# 一般由控制层提供，规划阶段可为 0
float64 current_error

# 是否为终态
# completed / canceled / failed / rejected 应为 true
bool is_terminal

# 是否为错误态
# failed / rejected 通常为 true
# canceled 一般为 false
bool is_error

# 时间戳
builtin_interfaces/Time stamp