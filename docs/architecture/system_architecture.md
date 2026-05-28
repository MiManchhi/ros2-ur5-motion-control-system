# 系统总体架构说明

## 1. 架构目标
本系统面向 UR5 仿真场景，采用 ROS2 多节点分层架构，实现“任务请求 -> 轨迹规划 -> 控制执行 -> 接口适配 -> 状态收敛”的完整链路。

当前实现重点在工程骨架与链路可运行性，控制算法采用可解释、可验证的简化方案（关节空间插值 + 闭环到位判定）。

---

## 2. 分层与职责

### 2.1 任务入口层（motion_api_pkg）
- 提供 `/move_joints` Action Server。
- 校验 Goal 合法性（关节数量、维度一致性、参数非负）。
- 生成 `task_id`，发布内部命令 `/motion_command`。
- 监听 `/task_state`，把正式状态回传为 Action Feedback/Result。

### 2.2 规划层（planner_pkg）
- 订阅 `/motion_command`。
- 订阅 `/joint_states` 作为当前位姿输入。
- 使用 `SimpleJointPlanner` 生成关节轨迹。
- 发布 `/planned_traj` 给控制层。

### 2.3 控制执行层（controller_pkg）
- 订阅 `/planned_traj` 启动执行器。
- 定时器驱动 `TrajectoryExecutor`，按规划轨迹 `time_from_start` 等待并分步下发。
- 发布 `/joint_cmd` 给接口层。
- 基于 `/joint_states` 做反馈超时、执行超时与目标到位判定。

### 2.4 机器人接口层（robot_interface_pkg）
- 订阅 `/joint_cmd`。
- 转换为底层 `trajectory_msgs/JointTrajectory`。
- 发布到 `/joint_trajectory_controller/joint_trajectory`（可参数化）。

### 2.5 系统管理层（system_manager_pkg）
- 汇总 `/motion_event`，通过状态机收敛为正式 `/task_state`。
- 广播系统级 `/system_state`。
- 提供 `/reset_system` 服务。
- 通过 watchdog 监控活动任务事件停滞，任务级执行超时由控制层处理。

---

## 3. 核心设计特征

1. **事件与正式状态分离**
   - 业务节点发布 `MotionEvent`（发生了什么）。
   - System Manager 发布 `TaskState`（正式生命周期状态）。

2. **单任务串行语义**
   - 当前仅允许一个活动任务；新任务在系统忙时会被拒绝。

3. **状态机约束流转**
   - 合法主链路：`accepted -> planning -> executing -> completed`。
   - 异常/终止链路：`canceled / failed / rejected`。

4. **统一可追踪性**
   - 全链路通过 `task_id` 贯通日志、事件、状态与 Action 结果。

---

## 4. 启动方式
- `bringup.launch.py` 提供统一入口。
- `launch_mode=system_only`：仅启动 5 个系统节点。
- `launch_mode=sim_with_system`：可选先启动外部仿真 launch，再延迟启动系统节点。
