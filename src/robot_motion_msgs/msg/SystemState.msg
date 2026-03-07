# 当前关联任务 ID；若为空可表示系统全局状态
string task_id
# 当前系统状态，如 idle / planning / executing / error / completed
string state
# 当前状态说明文本
string message
# 是否为错误状态
bool is_error
# 状态发布时间
builtin_interfaces/Time stamp