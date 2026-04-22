# 数据流设计说明

## 1. 主链路（任务执行）

```text
客户端
  -> /move_joints (Action Goal)
motion_api_node
  -> /motion_command (MotionCommand)
planner_node
  -> /planned_traj (PlannedTrajectory)
controller_node
  -> /joint_cmd (MotionCommand)
robot_interface_node
  -> /joint_trajectory_controller/joint_trajectory (JointTrajectory)
底层控制器/仿真
  -> /joint_states (JointState)
planner/controller 消费反馈
```

---

## 2. 状态链路（系统管理）

```text
motion_api / planner / controller / robot_interface
  -> /motion_event (MotionEvent)
system_manager_node
  -> /task_state (TaskState)
  -> /system_state (SystemState)
```

说明：
- `/motion_event` 是“事件流”，允许不同模块上报细粒度过程。
- `/task_state` 是“正式状态流”，用于对外一致语义。

---

## 3. 关键字段传递

### 3.1 task_id
- 在 Action 接收后由 motion_api 生成。
- 随 `/motion_command`、`/planned_traj`、`/joint_cmd`、`/motion_event`、`/task_state` 传递。

### 3.2 速度与超时
- Action Goal 的 `speed_scale`、`timeout_sec` 在 motion_api 归一化（为空/非正回退默认参数）。
- planner 使用 `speed_scale` 影响轨迹总时长。
- controller 以任务级 `timeout_sec` 覆盖默认执行超时。

### 3.3 执行节拍
- controller 根据轨迹 `time_from_start` 推导 `point_interval_sec`。
- robot_interface 优先使用 `point_interval_sec`，否则回退配置参数。

---

## 4. 异常数据流

- 任一业务节点可上报 `is_error=true` 的 `MotionEvent`。
- system_manager 收敛为 `TaskState=failed/rejected/canceled` 并发布系统状态。
- watchdog 检测任务整体超时后强制收敛为 `failed`。

---

## 5. Reset 数据流

```text
客户端
  -> /reset_system (ResetSystem)
system_manager_node
  -> /system_state(resetting -> idle)
  -> 若有活动任务则发布 task_state=failed
```
