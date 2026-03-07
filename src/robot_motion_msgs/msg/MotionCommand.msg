# 所属任务唯一标识，用于和 action / system_state / event 对应
string task_id
# 本次命令涉及的关节名称
string[] joint_names
# 目标关节位置或命令位置，顺序与 joint_names 对应
float64[] positions
# 消息生成时间戳
builtin_interfaces/Time stamp