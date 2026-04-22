# Topic 接口说明

## 1. 任务主链路

| Topic | Type | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|
| `/motion_command` | `robot_motion_msgs/msg/MotionCommand` | `motion_api_node` | `planner_node` | 内部任务命令（目标关节、速度、超时） |
| `/planned_traj` | `robot_motion_msgs/msg/PlannedTrajectory` | `planner_node` | `controller_node` | 规划结果轨迹 |
| `/joint_cmd` | `robot_motion_msgs/msg/MotionCommand` | `controller_node` | `robot_interface_node` | 控制层逐点命令 |
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | `robot_interface_node` | 底层控制器 | 接口层适配后的底层命令 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 底层控制器/仿真 | `planner_node`, `controller_node` | 关节反馈 |

---

## 2. 状态与事件链路

| Topic | Type | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|
| `/motion_event` | `robot_motion_msgs/msg/MotionEvent` | 业务节点 | `system_manager_node` | 事件总线 |
| `/task_state` | `robot_motion_msgs/msg/TaskState` | `system_manager_node` | `motion_api_node`, `controller_node` 等 | 正式任务状态 |
| `/system_state` | `robot_motion_msgs/msg/SystemState` | `system_manager_node` | 全系统/可视化 | 系统级状态 |

---

## 3. 当前实现备注

1. 当前 topic 名在代码中为硬编码常量（并非全部通过参数动态替换）。
2. 部分 package 的 YAML 里保留了 topic 参数字段，但并未被 node 主逻辑读取。
3. 现阶段建议将“代码真实行为”作为接口基准。
