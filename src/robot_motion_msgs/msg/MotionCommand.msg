# 内部统一运动命令
# 用于 motion_api -> planner，以及 controller -> robot_interface 两段链路
#
# 字段说明：
# - task_id：任务唯一标识
# - joint_names：关节名称列表
# - positions：目标关节位置
# - speed_scale：任务速度缩放比例，0 表示由下游使用默认值
# - timeout_sec：任务级超时时间，0 表示由下游使用默认值
# - point_interval_sec：当前控制点建议执行节拍，主要由控制层发给接口层
# - stamp：消息时间戳

string task_id
string[] joint_names
float64[] positions
float64 speed_scale
float64 timeout_sec
float64 point_interval_sec
builtin_interfaces/Time stamp