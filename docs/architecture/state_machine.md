# 状态机设计说明

## 1. 状态定义

任务状态（`TaskState.state`）：
- `accepted`
- `planning`
- `executing`
- `completed`
- `canceled`
- `failed`
- `rejected`

系统状态（`SystemState.state`）：
- `init`
- `idle`
- `busy`
- `resetting`
- `error`

---

## 2. 任务状态机流转规则

### 2.1 合法流转
- `accepted -> planning`
- `accepted -> canceled/rejected/failed`
- `planning -> executing`
- `planning -> canceled/failed`
- `executing -> completed`
- `executing -> canceled/failed`

### 2.2 终态
- `completed`
- `canceled`
- `failed`
- `rejected`

终态后不允许继续流转。

### 2.3 幂等规则
- 允许同状态重复进入（例如重复上报同一阶段事件）。

---

## 3. 事件到状态映射

默认映射（若 MotionEvent.related_state 为空）：
- `task_received -> accepted`
- `planning_started -> planning`
- `execution_started -> executing`
- `execution_done -> completed`
- `planning_failed -> failed`
- `execution_failed -> failed`
- `task_canceled -> canceled`
- `goal_rejected -> rejected`

说明：
- 若 `MotionEvent.related_state` 非空，manager 优先使用显式状态。

---

## 4. system_manager 行为约束

1. 无活动任务时，仅接受 `accepted` 作为新任务入口。
2. 有活动任务时，仅处理当前 `active_task_id` 的事件。
3. 非法状态流转会被收敛为 `failed`。
4. 任务进入终态后清理活动上下文。

---

## 5. Watchdog 与 Reset

- watchdog：按 `task_timeout_sec` 监控任务总耗时，超时后发布 `failed`。
- reset 服务：先发布 `resetting`，再清理上下文，最终发布 `idle`。
