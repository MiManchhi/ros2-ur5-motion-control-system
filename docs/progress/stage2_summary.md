# 阶段文档（按当前代码事实同步）

## A. 当前处于什么阶段

当前代码状态已经**跨过“纯架构骨架阶段”**，处于：

- **阶段 3（运动控制流程闭环实现）已主体落地并可运行**；
- **阶段 4（系统仿真验证与实验分析）尚未系统完成**。

判定依据是：系统主链路、状态链路、任务状态机、控制闭环关键保护（固定频率执行、误差判定、执行超时、反馈超时）均已在代码中实现；但实验验证文档与系统化实验结果尚未形成闭环。

---

## B. 阶段 3 当前完成情况

### B.1 统一状态语义与常量体系已落地

当前代码已使用 `robot_common_pkg/constants.hpp` 统一定义并复用以下语义：

- 任务状态：`accepted/planning/executing/completed/canceled/failed/rejected`
- 系统状态：`init/idle/busy/resetting/error`
- 事件名：`task_received/planning_started/planning_done/execution_started/execution_done/...`
- 模块名：`motion_api/planner/controller/robot_interface/system_manager`
- 辅助判定：终态、活动态、错误态、事件到任务状态映射

这意味着“状态字符串与事件字符串”已从分散硬编码切换为统一常量口径。

### B.2 事件流与正式状态流已完成分层

当前实现采用“**事件上报 -> manager 收敛正式状态**”机制：

- 业务节点发布 `/motion_event`（发生了什么）；
- `system_manager_node` 基于事件和状态机发布 `/task_state`（正式任务状态）与 `/system_state`（系统级状态）。

当前实现中，旧的“直接写原始系统状态过渡流”不再作为主实现路径。

### B.3 单任务串行执行策略已落地

`motion_api_node` 当前严格执行“单任务串行”：

- 若已有活动任务，则新 Goal 被拒绝；
- `enable_preempt=true` 仅保留扩展点，当前仍拒绝抢占；
- 取消流程通过 `task_canceled` 事件交由 manager 收敛，不在入口层直接强行结束全链路状态。

### B.4 规划器已实现线性插值与速度缩放

`planner_pkg/simple_joint_planner.cpp` 当前行为：

- 基于起点与目标点做**关节空间线性插值**；
- 使用 `traj_points` 与 `plan_duration_sec` 生成轨迹点序列与 `time_from_start`；
- `speed_scale` 参与总时长计算（含边界保护与 clamp）。

### B.5 控制闭环关键机制已实现

`controller_pkg` 当前实现了：

- **按轨迹时间戳执行**：按 `control_rate_hz` 定时驱动 `step()`，但只有到达规划轨迹 `time_from_start` 对应节拍时才发布下一个控制点；
- **反馈超时判定**：`joint_states` 长时间未更新则报错失败；
- **执行超时判定**：任务执行超时则报错失败；
- **误差判定**：按目标点最大关节误差与 `goal_tolerance` 判定到位；
- **执行完成收敛**：发布 `execution_done` 事件，由 manager 输出正式终态。

### B.6 robot_interface 已适配单点 JointTrajectory

`robot_interface_pkg` 当前将 `/joint_cmd` 转换为底层 `JointTrajectory` 时采用：

- **单点轨迹命令**（每次 1 个 `JointTrajectoryPoint`）；
- `time_from_start` 优先使用控制层下传 `point_interval_sec`，无效时回退配置参数。

---

## C. 当前系统主链路

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
planner/controller 消费 joint_states 作为反馈
```

主链路事实要点：

1. `task_id` 在入口生成并贯穿主链路与状态链路。
2. `speed_scale`、`timeout_sec` 从 Action Goal 经 `motion_api` 下传到规划/控制阶段。
3. 控制层按轨迹 `time_from_start` 等待发布控制点，并推导 `point_interval_sec` 供接口层适配单点命令。

---

## D. 当前系统状态链路

```text
motion_api / planner / controller / robot_interface
  -> /motion_event (MotionEvent)
system_manager_node
  -> /task_state (TaskState)
  -> /system_state (SystemState)
```

状态链路事实要点：

1. `/motion_event` 是过程事件流，不等价于正式任务状态。
2. `system_manager_node` 使用任务状态机校验合法流转，拒绝非法跳转。
3. `/task_state` 对外提供任务正式状态与终态判定。
4. `/system_state` 对外提供系统级状态（`init/idle/busy/resetting/error`）。
5. watchdog 基于活动任务事件停滞时间做兜底收敛，并按参数决定恢复到 `idle` 或进入 `error`。

---

## E. 当前系统仍需优化项（按代码现状）

以下为当前代码中“已暴露、但尚未完全收敛”的事项：

1. **`speed_scale` 语义已统一到“规划时长 + 控制节拍”。**
   - 当前代码已在规划器中真实生效；
   - 控制层按规划轨迹时间戳等待发布控制点；
   - 接口层仍使用 `point_interval_sec` 适配单点 JointTrajectory，不把 `speed_scale` 解释为底层控制器速度百分比。

2. **`timeout_sec` 口径已进一步明确。**
   - Action Goal 的 `timeout_sec` 作为任务级执行超时下传给控制层；
   - manager watchdog 当前用于任务事件停滞兜底，不再覆盖慢速任务的合法执行时间。

3. **反馈过程仍可增强。**
   - 当前 Action Feedback 主要由 `/task_state` 驱动，已具备进度与误差字段；
   - 但对执行中细粒度统计（如阶段耗时、稳定到位窗口、抖动指标）当前代码尚未体现。

4. **阶段 4 实验验证尚未系统完成。**
   - 代码具备实验基础能力；
   - 当前仓库尚未形成完整的系统化实验矩阵、批量数据汇总与统计结论闭环。

---

## F. 下一阶段工作重点

1. **完成阶段 4 的系统化实验验证。**
   - 固定测试场景、固定参数集、固定评价指标；
   - 形成可复现实验记录（成功率、超时率、误差分布、任务耗时）。

2. **统一并固化超时机制口径。**
   - 明确 `timeout_sec` 与控制层/manager 超时参数的关系；
   - 给出推荐配置优先级，避免不同层参数互相覆盖造成理解偏差。

3. **增强反馈与观测能力。**
   - 在不改变现有状态链路前提下，补充可量化执行反馈字段；
   - 提升调参与故障定位效率。

4. **完善文档与代码的一致性维护流程。**
   - 将接口文档、进度文档、实验文档与当前实现持续同步；
   - 对“当前代码尚未体现”的条目保持明确标注，避免文档超前于实现。

---

## G. 已删除/改写的不一致旧表述（本次口径）

1. 已移除把旧的“原始系统状态过渡逻辑”描述为当前主实现的写法。  
2. 已统一改为“`/motion_event -> /task_state + /system_state`”的当前实现口径。  
3. 已将“阶段 3 未实现/待补充”改为“阶段 3 主体已落地、阶段 4 未系统完成”。  
4. 已按当前代码明确写入：单任务串行、规划线性插值、控制固定频率与多类超时判定、接口层单点轨迹适配。  
5. 对当前代码尚未完全闭环的部分（端到端速度语义统一、超时语义统一、反馈增强、阶段 4 实验闭环）已如实保留为“待优化项”。
