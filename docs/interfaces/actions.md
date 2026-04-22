# Action 接口说明

## `/move_joints`

- **Action Type**: `robot_motion_msgs/action/MoveJoints`
- **Server**: `motion_api_node`
- **用途**: 提交一次关节空间运动任务。

---

## Goal

| 字段 | 类型 | 说明 |
|---|---|---|
| `task_name` | `string` | 人类可读任务名 |
| `joint_names` | `string[]` | 目标关节名 |
| `target_positions` | `float64[]` | 目标关节位置（通常 rad） |
| `speed_scale` | `float64` | 速度缩放，<=0 时回退默认值 |
| `timeout_sec` | `float64` | 任务超时，<=0 时回退默认值 |

### 校验规则（当前代码）
- `joint_names` / `target_positions` 不能为空。
- 两者长度必须一致。
- 当前实现要求关节数为 6（UR5）。
- `speed_scale`、`timeout_sec` 不能为负值。

---

## Feedback

| 字段 | 类型 | 说明 |
|---|---|---|
| `task_id` | `string` | 任务唯一标识 |
| `current_state` | `string` | 正式任务状态 |
| `progress` | `float64` | 进度 |
| `current_error` | `float64` | 当前误差 |

---

## Result

| 字段 | 类型 | 说明 |
|---|---|---|
| `success` | `bool` | 是否成功 |
| `task_id` | `string` | 任务唯一标识 |
| `message` | `string` | 结果说明 |
| `final_error` | `float64` | 最终误差 |

---

## 行为说明

- Action 生命周期由 `/task_state` 驱动：
  - `completed` -> succeed
  - `canceled` -> canceled
  - `failed/rejected` -> abort
- 入口层采用单任务串行策略，忙时拒绝新任务。
