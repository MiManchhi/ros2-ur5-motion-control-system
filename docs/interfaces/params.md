# 参数说明

> 本文档描述“当前代码会实际读取并生效”的参数。

## 1. motion_api_node

| 参数名 | 默认值 | 作用 |
|---|---:|---|
| `default_speed_scale` | `1.0` | Goal 未给有效速度时的回退值 |
| `default_timeout_sec` | `10.0` | Goal 未给有效超时时间时的回退值 |
| `enable_preempt` | `false` | 是否允许抢占入口（当前仍未实现完整抢占） |

## 2. planner_node

| 参数名 | 默认值 | 作用 |
|---|---:|---|
| `traj_points` | `50` | 插值轨迹点数量 |
| `plan_duration_sec` | `5.0` | 基准规划时长（会被 `speed_scale` 调整） |

## 3. controller_node

| 参数名 | 默认值 | 作用 |
|---|---:|---|
| `control_rate_hz` | `50.0` | 控制循环频率 |
| `goal_tolerance` | `0.01` | 到位误差阈值 |
| `execution_timeout_sec` | `15.0` | 默认执行超时 |
| `feedback_timeout_sec` | `1.0` | 关节反馈超时阈值 |
| `min_publish_interval_sec` | `0.02` | 最小控制点发送间隔 |

## 4. robot_interface_node

| 参数名 | 默认值 | 作用 |
|---|---:|---|
| `controller_topic` | `/joint_trajectory_controller/joint_trajectory` | 底层控制器命令话题 |
| `point_time_from_start_sec` | `0.1` | 未携带 `point_interval_sec` 时的回退节拍 |

## 5. system_manager_node

| 参数名 | 默认值 | 作用 |
|---|---:|---|
| `task_timeout_sec` | `20.0` | 活动任务事件停滞 watchdog 超时 |
| `watchdog_rate_hz` | `2.0` | watchdog 检查频率 |
| `auto_reset_on_failure` | `true` | 超时失败后是否回到 `idle` |

---

## 6. 注意事项

- 各 package 的 `config/*.yaml` 中仍存在一部分“保留参数字段”，当前代码未读取。
- 以节点源码中的 `declare_parameter/get_parameter` 为最终生效基准。
