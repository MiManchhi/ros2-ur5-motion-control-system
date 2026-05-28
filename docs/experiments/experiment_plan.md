# 仿真实验计划

## 1. 实验目标

本阶段实验用于验证当前 ROS2 工业机器人运动控制系统是否能够在 UR5 Gazebo 仿真环境中稳定完成完整任务链路：

```text
/move_joints
  -> /motion_command
  -> /planned_traj
  -> /joint_cmd
  -> /joint_trajectory_controller/joint_trajectory
  -> /joint_states
```

同时验证状态链路：

```text
/motion_event
  -> /task_state
  -> /system_state
```

当前实验重点是系统架构与控制流程验证，不以高级规划算法、真实机械臂部署或动力学精度优化为目标。

## 2. 实验环境

- 工作空间：`/home/myubuntu/graduation_project/ros2_ws`
- 构建方式：`colcon build`
- ROS2 版本：Humble
- 机器人模型：UR5
- 仿真平台：Gazebo + ros2_control
- 外部仿真工作空间默认路径：`~/workspaces/ur_gazebo`
- 系统启动脚本：`scripts/run_graduation_tests.sh`

最近一次工程构建验证：2026-05-28，速度控制、规划、接口、系统管理与 bringup 相关 package 构建通过。完整工作空间包含 8 个核心 package。

## 3. 前置检查

正式实验前需要确认：

```bash
cd /home/myubuntu/graduation_project/ros2_ws
colcon build
source install/setup.bash
```

启动系统与仿真：

```bash
./scripts/run_graduation_tests.sh launch-sim
```

检查关键接口：

```bash
ros2 topic echo /joint_states --once
ros2 action list
ros2 service list
ros2 topic list
```

必须能观察到：

- `/move_joints`
- `/reset_system`
- `/joint_states`
- `/motion_command`
- `/planned_traj`
- `/joint_cmd`
- `/motion_event`
- `/task_state`
- `/system_state`
- `/joint_trajectory_controller/joint_trajectory`

## 4. 实验场景

### 4.1 正常运动实验

命令：

```bash
./scripts/run_graduation_tests.sh test-normal
```

验证点：

- Action Goal 被接受。
- `/task_state` 按 `accepted -> planning -> executing -> completed` 流转。
- Gazebo 中 UR5 出现可见运动。
- `/system_state` 在任务期间为 `busy`，任务结束后回到 `idle`。
- Action Result 返回成功。

### 4.2 替代目标位姿实验

命令：

```bash
./scripts/run_graduation_tests.sh test-alt
```

验证点：

- 系统能够连续处理不同目标位姿任务。
- 任务完成后误差低于 `goal_tolerance`。
- 不出现非法状态流转。

### 4.3 低速运动实验

命令：

```bash
./scripts/run_graduation_tests.sh test-slow
```

验证点：

- `speed_scale` 影响规划轨迹总时长，控制层按轨迹时间戳调度发布节奏。
- 控制层能够按轨迹 `time_from_start` 等待并逐点发布命令。
- 长时任务不触发非预期超时。

### 4.4 快速运动实验

命令：

```bash
./scripts/run_graduation_tests.sh test-fast
```

验证点：

- `speed_scale=1.00` 时，规划轨迹时长短于 normal/slow。
- 脚本输出 `Actual action elapsed`，可记录端到端真实耗时。
- 机械臂产生明显运动并最终完成。

### 4.5 非法输入实验

命令：

```bash
./scripts/run_graduation_tests.sh test-invalid
```

验证点：

- `motion_api_node` 拒绝关节名数量与目标位置数量不一致的 Goal。
- 系统不进入异常执行链路。
- 不产生错误的底层控制命令。

### 4.6 执行超时实验

命令：

```bash
./scripts/run_graduation_tests.sh test-timeout
```

验证点：

- 控制层根据任务级 `timeout_sec` 将任务收敛为 `failed`。
- Action Result 返回失败。
- `/system_state` 根据参数恢复 `idle` 或进入 `error`。

### 4.7 Reset 实验

命令：

```bash
./scripts/run_graduation_tests.sh reset
```

验证点：

- `/reset_system` 返回成功。
- `/system_state` 出现 `resetting -> idle`。
- 若 reset 时存在活动任务，该任务应收敛为 `failed`。

### 4.8 超时恢复组合实验

命令：

```bash
./scripts/run_graduation_tests.sh test-recovery
```

验证点：

- 先触发执行超时失败。
- 调用 `/reset_system` 后系统恢复 `idle`。
- reset 后再次发送正常恢复运动，任务应完成。

### 4.9 异常组合实验

命令：

```bash
./scripts/run_graduation_tests.sh test-abnormal-suite
```

验证点：

- 非法输入被拒绝或失败收敛。
- 执行超时进入 `failed`。
- reset 后系统恢复，并可再次完成正常任务。

## 5. 数据记录

建议每组实验单独录制 rosbag：

```bash
./scripts/run_graduation_tests.sh bag
```

默认记录话题：

- `/joint_states`
- `/planned_traj`
- `/joint_cmd`
- `/task_state`
- `/system_state`
- `/motion_event`

每组实验至少记录：

- 实验名称
- 命令与参数
- rosbag 路径
- Action 最终结果
- 脚本输出的 `Actual action elapsed`
- 任务状态流转
- 最大最终误差
- 总耗时
- 是否符合预期
- 异常现象与原因分析

## 6. 评价指标

- 构建结果：`colcon build` 是否通过。
- 功能正确性：Action 是否按预期成功或失败。
- 状态一致性：`/task_state` 与 `/system_state` 是否符合状态机规则。
- 闭环效果：最终最大关节误差是否低于 `goal_tolerance`。
- 稳定性：是否出现反馈超时、非法状态流转或节点异常退出。
- 可复现性：实验命令、参数、rosbag 路径是否完整记录。
