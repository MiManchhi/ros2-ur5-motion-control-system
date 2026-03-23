# 规划层输出的轨迹结果
#
# 字段说明：
# - task_id：任务唯一标识
# - trajectory：规划生成的关节轨迹
# - timeout_sec：任务级超时，传递给控制执行层
# - stamp：消息时间戳

string task_id
trajectory_msgs/JointTrajectory trajectory
float64 timeout_sec
builtin_interfaces/Time stamp