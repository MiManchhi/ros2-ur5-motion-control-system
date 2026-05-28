# 基于 ROS2 的工业机器人运动控制系统

本仓库是一个面向 UR5 工业机器人仿真的 ROS2 运动控制系统示例工程。系统以“任务入口、轨迹规划、控制执行、接口适配、状态管理”为主要分层，验证从上层关节运动请求到 Gazebo/ros2_control 仿真执行和任务状态反馈的完整闭环。

项目重点是系统架构、接口设计、任务状态管理和仿真验证，不包含避障规划、动力学最优控制、真实机械臂安全联锁或多任务并发调度。

## 技术栈

- Ubuntu 22.04
- ROS2 Humble
- Gazebo + ros2_control
- C++17
- Python 3
- colcon / ament_cmake
- UR5 仿真模型

## 工作空间结构

```text
ros2_ws/
  src/                  ROS2 功能包源码
  docs/                 架构、接口、实验和进度文档
  scripts/              构建、测试、绘图和实验辅助脚本
  bags/                 rosbag 实验数据目录
  build/ install/ log/  colcon 生成目录
```

## 功能包说明

| 包名 | 职责 |
|---|---|
| `robot_motion_msgs` | 定义 Action、msg、srv 接口 |
| `robot_common_pkg` | 共享状态、事件、模块名等常量 |
| `motion_api_pkg` | `/move_joints` Action 入口、Goal 校验、任务下发 |
| `planner_pkg` | 基于 `/joint_states` 和目标关节角生成关节空间轨迹 |
| `controller_pkg` | 按轨迹时间戳执行控制点、到位判定、超时检测 |
| `robot_interface_pkg` | 将内部命令适配为底层 `JointTrajectory` |
| `system_manager_pkg` | 任务状态机、系统状态、reset、watchdog |
| `bringup_pkg` | 统一 launch 入口和参数加载 |

## 系统架构

主运动链路：

```text
Client
  -> /move_joints
motion_api_node
  -> /motion_command
planner_node
  -> /planned_traj
controller_node
  -> /joint_cmd
robot_interface_node
  -> /joint_trajectory_controller/joint_trajectory
Gazebo / ros2_control
  -> /joint_states
```

状态管理链路：

```text
motion_api / planner / controller / robot_interface
  -> /motion_event
system_manager_node
  -> /task_state
  -> /system_state
```

核心设计点：

- `speed_scale` 用于调整规划轨迹总时长，控制层按轨迹 `time_from_start` 调度控制点。
- `timeout_sec` 是任务级执行超时，控制层负责判定。
- `system_manager_node` 使用事件流收敛正式任务状态，并通过 reset 服务恢复系统。
- 当前接口层采用单点 `JointTrajectory` 适配方式，便于对接 Gazebo 的轨迹控制器。

## 构建

```bash
cd /home/myubuntu/graduation_project/ros2_ws
colcon build
source install/setup.bash
```

如果只想构建核心业务包：

```bash
colcon build --packages-select \
  robot_motion_msgs robot_common_pkg motion_api_pkg planner_pkg \
  controller_pkg robot_interface_pkg system_manager_pkg bringup_pkg
source install/setup.bash
```

## 配置

主要参数文件：

| 文件 | 说明 |
|---|---|
| `src/motion_api_pkg/config/motion_api_params.yaml` | 默认速度、默认超时、抢占开关 |
| `src/planner_pkg/config/planner_params.yaml` | 轨迹点数量、基准规划时长 |
| `src/controller_pkg/config/controller_params.yaml` | 控制频率、误差阈值、执行/反馈超时 |
| `src/robot_interface_pkg/config/interface_params.yaml` | 底层控制器话题、单点轨迹默认时间 |
| `src/system_manager_pkg/config/manager_params.yaml` | watchdog、reset 后状态策略 |

参数说明见 [docs/interfaces/params.md](/home/myubuntu/graduation_project/ros2_ws/docs/interfaces/params.md)。

## 运行

仅启动系统节点：

```bash
source install/setup.bash
ros2 launch bringup_pkg bringup.launch.py launch_mode:=system_only
```

启动仿真和系统：

```bash
./scripts/run_graduation_tests.sh launch-sim
```

如果外部 UR Gazebo 工作空间路径不同：

```bash
SIM_WORKSPACE_DIR=/path/to/ur_gazebo \
./scripts/run_graduation_tests.sh launch-sim
```

## 测试

常用测试由 `scripts/run_graduation_tests.sh` 封装：

```bash
./scripts/run_graduation_tests.sh test-fast
./scripts/run_graduation_tests.sh test-normal
./scripts/run_graduation_tests.sh test-alt
./scripts/run_graduation_tests.sh test-slow
./scripts/run_graduation_tests.sh test-invalid
./scripts/run_graduation_tests.sh test-timeout
./scripts/run_graduation_tests.sh test-recovery
./scripts/run_graduation_tests.sh test-abnormal-suite
./scripts/run_graduation_tests.sh reset
```

运动测试会打印：

- `expected_planned_duration`：由 `plan_duration_sec / speed_scale` 得到的规划轨迹时长。
- `Actual action elapsed`：Action 发送到返回的端到端真实耗时。

当前测试档位：

| 命令 | 用途 | speed_scale | timeout_sec |
|---|---|---:|---:|
| `test-fast` | 快速明显运动 | 1.00 | 12.0 |
| `test-normal` | 正常闭环运动 | 0.65 | 18.0 |
| `test-alt` | 不同目标运动 | 0.65 | 18.0 |
| `test-slow` | 慢速对比运动 | 0.35 | 25.0 |
| `test-timeout` | 执行超时验证 | 0.40 | 0.5 |

## 数据记录

录制 rosbag：

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

查看最新 bag：

```bash
ros2 bag info $(ls -td bags/graduation_test_* | head -1)
```

绘图脚本位于 `scripts/`：

- `plot_joint_states_from_bag.py`
- `plot_max_joint_error_from_bag.py`
- `plot_task_duration_comparison_from_bag.py`

## 如何阅读项目

建议按下面顺序阅读：

1. [docs/architecture/system_architecture.md](/home/myubuntu/graduation_project/ros2_ws/docs/architecture/system_architecture.md)：先看系统分层。
2. [docs/architecture/data_flow.md](/home/myubuntu/graduation_project/ros2_ws/docs/architecture/data_flow.md)：理解主链路和状态链路。
3. [docs/interfaces/topics.md](/home/myubuntu/graduation_project/ros2_ws/docs/interfaces/topics.md)：查看核心话题。
4. [docs/interfaces/actions.md](/home/myubuntu/graduation_project/ros2_ws/docs/interfaces/actions.md)：查看 `/move_joints` Action。
5. [docs/interfaces/params.md](/home/myubuntu/graduation_project/ros2_ws/docs/interfaces/params.md)：查看可调参数。
6. `src/motion_api_pkg` -> `src/planner_pkg` -> `src/controller_pkg` -> `src/robot_interface_pkg` -> `src/system_manager_pkg`：按运行链路阅读源码。

## 常用调试命令

```bash
ros2 node list
ros2 topic list
ros2 action list
ros2 service list
ros2 topic echo /task_state
ros2 topic echo /system_state
ros2 topic echo /joint_states --once
ros2 topic echo /planned_traj --once
```

## 文档索引

- 架构文档：`docs/architecture/`
- 接口文档：`docs/interfaces/`
- 实验文档：`docs/experiments/`
- 进度和论文辅助材料：`docs/progress/`
- 脚本说明：`scripts/README.md`、`scripts/测试执行说明.md`
