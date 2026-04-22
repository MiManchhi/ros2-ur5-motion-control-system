# Service 接口说明

## `/reset_system`

- **Service Type**: `robot_motion_msgs/srv/ResetSystem`
- **Server**: `system_manager_node`
- **用途**: 重置系统运行上下文并恢复空闲状态。

---

## Request

| 字段 | 类型 | 说明 |
|---|---|---|
| `force_reset` | `bool` | 是否强制复位（当前代码未区分处理） |

## Response

| 字段 | 类型 | 说明 |
|---|---|---|
| `success` | `bool` | 复位是否成功 |
| `message` | `string` | 结果说明 |

---

## 当前行为

1. 收到请求后先发布系统状态 `resetting`。
2. 若存在活动任务，先将该任务收敛为 `failed`。
3. 清空 manager 活动任务上下文。
4. 最终发布系统状态 `idle` 并返回成功。
