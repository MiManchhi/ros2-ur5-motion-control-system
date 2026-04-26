# 基于 ROS2 的工业机器人运动控制系统架构设计与仿真验证

## 项目简介
本项目为本科毕业设计工程实现仓库，研究方向为基于 ROS2 的工业机器人运动控制系统架构设计与仿真验证。
研究对象为 UR5 工业机器人，开发环境基于 **Ubuntu 22.04 + ROS2 Humble + Gazebo**。

---

## 当前阶段
- 阶段 0：完成
- 阶段 1：完成
- 阶段 2：完成
- 阶段 3：进行中（后半段）

> 说明：根据当前代码实现，项目已进入“阶段 3 后半段”。
> 已具备的阶段 3 关键能力包括：
> 1) 控制层闭环执行与到位误差判定；
> 2) joint_states 反馈超时与执行超时保护；
> 3) 任务状态终态（`canceled` / `failed`）驱动控制器停机闭环（停止执行器并上报 `execution_done`）；
> 4) 事件流 + 正式状态流双层状态收敛与 Action 生命周期联动。

---

## 系统架构总览（按当前代码行为）

系统采用 ROS2 多节点分层架构，主链路如下：

```text
客户端
  -> /move_joints (Action)
motion_api_node
  -> /motion_command
planner_node
  -> /planned_traj
controller_node
  -> /joint_cmd
robot_interface_node
  -> /joint_trajectory_controller/joint_trajectory
底层控制器/仿真
  -> /joint_states
```

状态管理采用“事件流 + 正式状态流”双层机制：

```text
业务节点
  -> /motion_event
system_manager_node
  -> /task_state
  -> /system_state
```

---

## 工作空间结构
- `src/`：ROS2 功能包源码
- `docs/`：架构、接口、进度与实验文档
- `bags/`：实验数据
- `scripts/`：辅助脚本

---

## 主要功能包与职责
- `robot_motion_msgs`：统一定义 msg/action/srv 接口。
- `motion_api_pkg`：Action 入口、任务校验、`task_id` 生成与命令下发。
- `planner_pkg`：读取当前关节状态并进行轨迹规划。
- `controller_pkg`：轨迹执行、闭环判定、执行事件上报。
- `robot_interface_pkg`：上层命令到 Gazebo/ros2_control 命令适配。
- `system_manager_pkg`：任务状态机收敛、系统状态广播、watchdog/reset。
- `bringup_pkg`：统一启动入口与运行模式管理。
- `robot_common_pkg`：跨包共享常量与状态/事件映射工具。

---

## 关键接口（当前实现）

### Action
- `/move_joints` (`robot_motion_msgs/action/MoveJoints`)

### Topics（核心）
- `/motion_command` (`robot_motion_msgs/msg/MotionCommand`)
- `/planned_traj` (`robot_motion_msgs/msg/PlannedTrajectory`)
- `/joint_cmd` (`robot_motion_msgs/msg/MotionCommand`)
- `/motion_event` (`robot_motion_msgs/msg/MotionEvent`)
- `/task_state` (`robot_motion_msgs/msg/TaskState`)
- `/system_state` (`robot_motion_msgs/msg/SystemState`)
- `/joint_states` (`sensor_msgs/msg/JointState`)

### Service
- `/reset_system` (`robot_motion_msgs/srv/ResetSystem`)

---

## 启动方式
统一入口：

```bash
ros2 launch bringup_pkg bringup.launch.py
```

常见模式：

```bash
# 仅启动系统 5 个核心节点
ros2 launch bringup_pkg bringup.launch.py launch_mode:=system_only

# 系统 + 外部仿真（可选）
ros2 launch bringup_pkg bringup.launch.py \
  launch_mode:=sim_with_system \
  start_sim:=true \
  sim_launch_file:=/path/to/your_sim.launch.py
```

---

## 文档索引
- 架构说明：
  - `docs/architecture/system_architecture.md`
  - `docs/architecture/data_flow.md`
  - `docs/architecture/state_machine.md`
- 接口说明：
  - `docs/interfaces/topics.md`
  - `docs/interfaces/actions.md`
  - `docs/interfaces/services.md`
  - `docs/interfaces/params.md`
- 进度文档：
  - `docs/progress/进度规划.md`
  - `docs/progress/stage1_summary.md`
  - `docs/progress/stage2_summary.md`
- 实验文档：
  - `docs/experiments/experiment_plan.md`
  - `docs/experiments/experiment_records.md`
