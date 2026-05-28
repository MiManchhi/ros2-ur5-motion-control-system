# 实验记录

## 1. 工程构建记录

### 2026-05-28 构建验证

执行目录：

```bash
/home/myubuntu/graduation_project/ros2_ws
```

执行命令：

```bash
colcon build --packages-select controller_pkg planner_pkg motion_api_pkg robot_interface_pkg bringup_pkg
colcon build --packages-select system_manager_pkg bringup_pkg
```

构建结果：

- `robot_common_pkg`：通过
- `robot_motion_msgs`：通过
- `motion_api_pkg`：通过
- `planner_pkg`：通过
- `controller_pkg`：通过
- `robot_interface_pkg`：通过
- `system_manager_pkg`：通过
- `bringup_pkg`：通过

结论：速度控制、规划、接口、系统管理与 bringup 相关 package 构建成功。当前工程具备继续进行系统化仿真实验和最终材料归档的基础。

## 2. 待执行实验记录表

| 序号 | 实验名称 | 命令 | 预期结果 | 当前状态 |
|---|---|---|---|---|
| 1 | 正常运动实验 | `./scripts/run_graduation_tests.sh test-normal` | Action 成功，任务状态最终 `completed` | 待执行 |
| 2 | 替代目标位姿实验 | `./scripts/run_graduation_tests.sh test-alt` | 不同目标位姿可正常执行 | 待执行 |
| 3 | 低速运动实验 | `./scripts/run_graduation_tests.sh test-slow` | `speed_scale` 影响轨迹时长且任务完成 | 待执行 |
| 4 | 快速运动实验 | `./scripts/run_graduation_tests.sh test-fast` | 任务完成且耗时短于 normal/slow | 待执行 |
| 5 | 非法输入实验 | `./scripts/run_graduation_tests.sh test-invalid` | Goal 被拒绝，不进入执行链路 | 待执行 |
| 6 | 执行超时实验 | `./scripts/run_graduation_tests.sh test-timeout` | 任务收敛为 `failed` | 待执行 |
| 7 | Reset 实验 | `./scripts/run_graduation_tests.sh reset` | 系统状态 `resetting -> idle` | 待执行 |
| 8 | 超时恢复实验 | `./scripts/run_graduation_tests.sh test-recovery` | 超时后 reset，随后正常任务完成 | 待执行 |
| 9 | 异常组合实验 | `./scripts/run_graduation_tests.sh test-abnormal-suite` | 非法输入、超时、reset、恢复链路符合预期 | 待执行 |

## 3. 单次实验记录模板

### 实验编号

- 实验名称：
- 执行日期：
- 执行命令：
- rosbag 路径：
- 参数设置：
- Action Result：
- 脚本输出耗时（Actual action elapsed）：
- `/task_state` 流转：
- `/system_state` 流转：
- 最终误差：
- 总耗时：
- Gazebo 观察结果：
- 是否符合预期：
- 问题与分析：

## 4. 当前阶段结论

截至 2026-05-28，工程已完成速度节拍控制、超时口径统一、异常恢复测试脚本和核心包构建验证。后续实验记录应以 rosbag、Action 输出耗时和最终误差为依据补齐表格，不填写无来源数值。
