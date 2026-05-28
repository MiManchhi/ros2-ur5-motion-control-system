# 毕业设计最终完成执行计划

## 1. 当前基线判断

根据当前代码和文档状态，本项目已经完成从“架构设计”到“核心闭环实现”的主要开发工作，当前应按如下进度管理：

- 阶段 0：选题确认与技术路线规划，已完成。
- 阶段 1：开发环境搭建与最小系统验证，已完成。
- 阶段 2：系统总体架构实现，已完成主体内容。
- 阶段 3：运动控制流程闭环实现，主体已落地并具备运行条件。
- 阶段 4：系统仿真验证与实验分析，正在进入，尚未系统完成。
- 阶段 5：论文撰写与成果整理，已有材料基础，但尚未完成最终整合。

当前代码已经具备以下能力：

- 通过 `/move_joints` Action 接收关节空间运动任务。
- 由 `motion_api_node` 完成任务校验、`task_id` 生成、任务上下文维护和 Action 反馈。
- 由 `planner_node` 读取 `/joint_states`，生成关节空间线性插值轨迹。
- 由 `controller_node` 按固定频率执行轨迹点，下发 `/joint_cmd`，并基于关节反馈判断到位、超时和失败。
- 由 `robot_interface_node` 将上层 `MotionCommand` 适配为底层 `JointTrajectory`。
- 由 `system_manager_node` 统一收敛 `/motion_event`，发布正式 `/task_state` 和 `/system_state`。
- 通过 `bringup_pkg` 和 `scripts/run_graduation_tests.sh` 启动系统、运行测试、录制 rosbag 和执行 reset。

当前仍需补齐的关键内容：

- 完整编译和运行前的工程健康检查。
- 实验计划、实验记录、数据统计和结果分析。
- README、进度文档、接口文档、参数文档与当前实现之间的口径统一。
- 论文正文、附录、演示材料和答辩材料。
- 对已知限制进行明确说明，例如当前采用单任务串行策略，抢占仅预留接口，不作为已完成能力。

## 2. 最终完成目标

最终完成时，本项目应达到以下状态：

1. 工程可复现  
   在指定 ROS2 Humble + Gazebo + UR5 仿真环境下，项目能够完成编译、启动、测试和 rosbag 记录。

2. 功能可演示  
   能够演示一条完整运动任务链路：`Action Goal -> 规划 -> 控制 -> 接口适配 -> 仿真运动 -> 状态反馈 -> Action Result`。

3. 异常可验证  
   能够演示并记录非法输入、执行超时、reset 复位等异常或恢复场景。

4. 数据可分析  
   每组实验都应有明确输入、过程记录、结果状态、耗时、误差、rosbag 路径和分析结论。

5. 文档可支撑论文  
   架构、接口、状态机、控制流程、实验流程、实验结果和不足展望均有对应材料。

6. 答辩可说明  
   能够清楚说明本课题做了什么、为什么这样设计、当前完成到什么程度、实验结果如何、还有哪些不足。

## 3. 总体推进路线

后续工作按照六个执行阶段推进：

```text
执行阶段 A：工程健康检查与文档口径统一
执行阶段 B：实验环境固化与测试脚本确认
执行阶段 C：系统化仿真实验执行
执行阶段 D：实验数据整理与结果分析
执行阶段 E：论文与项目文档收口
执行阶段 F：最终演示、答辩材料和归档
```

推荐先完成 A 和 B，再进入 C。不要在实验尚未稳定前急着写最终结论，否则后续数据变化会导致论文反复修改。

## 4. 执行阶段 A：工程健康检查与文档口径统一

### A.1 清理并确认编译状态

目标：

- 确认当前源码能够完整编译。
- 排除旧 build/install/log 产物导致的构建问题。
- 得到一个可作为后续实验基线的工作区状态。

具体任务：

1. 记录当前 git 状态。
2. 检查是否存在未提交或未知来源修改。
3. 清理旧构建产物后重新编译。
4. 编译成功后重新 source 环境。
5. 记录编译命令、编译结果和问题处理过程。

建议命令：

```bash
cd /home/myubuntu/graduation_project/ros2_ws
git status --short

# 若确认 build/install/log 中没有需要保留的文件，再清理旧构建产物
rm -rf build install log

colcon build
source install/setup.bash
```

完成标准：

- `colcon build` 完整通过。
- `install/setup.bash` 存在并可正常 source。
- 所有核心包均完成构建：`robot_motion_msgs`、`robot_common_pkg`、`motion_api_pkg`、`planner_pkg`、`controller_pkg`、`robot_interface_pkg`、`system_manager_pkg`、`bringup_pkg`。
- 若出现构建问题，应在 `docs/experiments/experiment_records.md` 或单独问题记录中说明原因和解决方式。

注意事项：

- 之前曾出现 `robot_motion_msgs` 的旧构建产物阻塞符号链接创建的问题，这类问题优先按构建缓存问题处理。
- 若清理构建产物后仍失败，再判断是否是 CMake、接口定义或依赖声明问题。

### A.2 统一当前进度口径

目标：

- 避免 README、进度总结、中期材料之间互相矛盾。
- 将当前进度统一描述为“阶段 3 主体完成，阶段 4 正在进行”。

需要更新或检查的文档：

- `README.md`
- `docs/progress/stage2_summary.md`
- `docs/progress/进度规划.md`
- `docs/progress/中期检查材料.md`
- `docs/progress/中期检查演示文稿.md`

建议统一表述：

```text
当前项目已经完成系统架构搭建和运动控制主链路实现，阶段 3 的运动控制闭环功能已主体落地。后续工作重点是阶段 4 的系统化仿真实验、实验数据分析，以及阶段 5 的论文和答辩材料整理。
```

完成标准：

- 不再出现“阶段 2 仍在进行，但阶段 3 未完成”的旧表述。
- 已完成内容和未完成内容边界清楚。
- 抢占、真实机械臂对接、高级轨迹算法等未实现内容被明确写为后续扩展，而不是当前成果。

### A.3 固化参数和接口说明

目标：

- 明确实验时采用哪些参数。
- 明确 `speed_scale`、`timeout_sec`、`goal_tolerance` 等参数含义。

需要检查的文档：

- `docs/interfaces/topics.md`
- `docs/interfaces/actions.md`
- `docs/interfaces/services.md`
- `docs/interfaces/params.md`
- 各包 `config/*.yaml`

重点说明：

- `speed_scale` 当前通过规划轨迹总时长和控制层 `time_from_start` 调度节奏生效。
- `timeout_sec` 会从 Action Goal 下传到规划和控制阶段，控制层据此进行执行超时判断。
- manager 还有 `task_timeout_sec`，用于活动任务事件停滞 watchdog，不覆盖慢速任务的合法执行时间。
- 控制层有 `feedback_timeout_sec`，用于判断 `/joint_states` 是否中断。
- `goal_tolerance` 用于最终到位判定，当前按最大关节误差判断。

完成标准：

- 参数文档能解释每个关键参数在哪里使用。
- 实验采用的参数值明确记录。
- 论文中可以直接引用这些参数说明。

## 5. 执行阶段 B：实验环境固化与测试脚本确认

### B.1 固定实验环境

目标：

- 确保每次实验在同一套环境中执行，减少不可控差异。

需要记录：

- 操作系统：Ubuntu 22.04。
- ROS2：Humble。
- 仿真平台：Gazebo。
- 机器人模型：UR5。
- 项目工作空间：`/home/myubuntu/graduation_project/ros2_ws`。
- UR Gazebo 仿真工作空间：`~/workspaces/ur_gazebo`。
- 外部仿真 launch 文件路径。
- 核心控制器话题：`/joint_trajectory_controller/joint_trajectory`。

完成标准：

- `scripts/测试执行说明.md` 中的路径与实际环境一致。
- `scripts/run_graduation_tests.sh launch-sim` 能正常启动仿真和系统。
- `/joint_states` 能稳定发布。
- 核心 topic 和 action 能被 `ros2 topic list`、`ros2 action list` 查到。

### B.2 固定实验观察项

每次实验至少观察以下内容：

- Gazebo 中 UR5 是否产生可见运动。
- `/move_joints` Action 是否返回成功或符合预期的失败。
- `/task_state` 是否按照预期状态流转。
- `/system_state` 是否在任务期间进入 `busy`，任务结束后回到 `idle`。
- `/motion_event` 是否能看到各模块事件。
- `/joint_states` 是否持续更新。
- rosbag 是否成功写入 `metadata.yaml`。

建议检查命令：

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /task_state
ros2 topic echo /system_state
ros2 topic echo /motion_event
ros2 action list
ros2 service list
```

完成标准：

- 所有实验前置检查通过。
- 若某项检查失败，应先修复环境或启动顺序，不进入正式实验记录。

### B.3 固定 rosbag 记录范围

建议记录 topic：

```text
/joint_states
/planned_traj
/joint_cmd
/task_state
/system_state
/motion_event
```

建议输出目录格式：

```text
bags/graduation_test_YYYYMMDD_HHMMSS/
```

完成标准：

- 每组正式实验都有对应 bag。
- bag 目录名称能看出实验时间或实验编号。
- 实验记录文档中写明 bag 路径。

## 6. 执行阶段 C：系统化仿真实验执行

阶段 C 是后续最关键的工作。建议至少完成 6 类实验，其中正常任务类实验需要多次重复，以支撑成功率和误差分析。

### C.1 实验 1：正常运动任务

目的：

- 验证系统主链路可以完成一次完整运动任务。

测试命令：

```bash
./scripts/run_graduation_tests.sh test-normal
```

预期结果：

- Action Goal 被接受。
- `/task_state` 状态流转为 `accepted -> planning -> executing -> completed`。
- Gazebo 中 UR5 产生运动。
- 最终 Action Result 中 `success=true`。
- `/system_state` 最终回到 `idle`。

记录内容：

- 目标关节角。
- 任务开始时间和结束时间。
- 最终误差 `final_error`。
- rosbag 路径。
- Gazebo 截图。
- `/task_state` 关键输出。

完成标准：

- 至少成功执行 3 次。
- 若失败，需要记录失败原因并判断是否为环境启动问题、超时参数问题或代码逻辑问题。

### C.2 实验 2：不同目标点运动任务

目的：

- 验证系统不是只能完成单一目标点，而是能够处理不同关节目标。

测试命令：

```bash
./scripts/run_graduation_tests.sh test-alt
```

预期结果：

- 与实验 1 类似，最终完成。
- UR5 运动方向或姿态变化与实验 1 不同。

记录内容：

- 目标关节角与实验 1 的差异。
- 任务耗时。
- 最终误差。
- 状态流转。
- Gazebo 截图。

完成标准：

- 至少成功执行 3 次。
- 记录不同目标点下系统仍能保持完整状态链路。

### C.3 实验 3：慢速运动任务

目的：

- 验证 `speed_scale` 对规划时长和执行节奏有影响。

测试命令：

```bash
./scripts/run_graduation_tests.sh test-slow
```

预期结果：

- 轨迹总时长比普通速度任务更长。
- `/planned_traj` 中 `time_from_start` 分布体现较慢执行节奏。
- 任务仍能完成。

记录内容：

- `speed_scale` 值。
- 规划点数量。
- 规划总时长。
- 实际任务耗时。
- 最终误差。

完成标准：

- 至少执行 2 次。
- 能在结果分析中说明速度缩放对轨迹时间的影响。

### C.4 实验 4：非法输入任务

目的：

- 验证入口层参数校验能力。

测试命令：

```bash
./scripts/run_graduation_tests.sh test-invalid
```

预期结果：

- Action Goal 被拒绝或任务无法进入正式执行链路。
- 系统不会向下游发布错误轨迹。
- 系统不会进入异常死锁。

记录内容：

- 非法输入类型，例如关节数量和目标位置数量不一致。
- Action 返回结果。
- `/task_state` 是否产生状态。
- 系统是否保持可继续运行。

完成标准：

- 非法输入不会导致系统崩溃。
- 后续正常任务仍可执行。

### C.5 实验 5：执行超时任务

目的：

- 验证任务超时保护能力。

测试命令：

```bash
./scripts/run_graduation_tests.sh test-timeout
```

预期结果：

- 任务因超时进入 `failed`。
- `/motion_event` 中出现失败事件。
- `/task_state` 最终为 `failed`。
- Action Result 中 `success=false`。
- 系统最终恢复到可继续测试状态。

记录内容：

- 设置的 `timeout_sec`。
- 失败发生阶段。
- `/motion_event` 中的失败 detail。
- `/task_state` 终态。
- reset 是否需要介入。

完成标准：

- 至少成功复现 2 次。
- 能说明超时机制属于可靠性保护设计。

### C.6 实验 6：reset 复位

目的：

- 验证系统复位服务可用。

测试命令：

```bash
./scripts/run_graduation_tests.sh reset
```

建议场景：

- 在系统空闲时调用 reset。
- 在任务执行中调用 reset。
- 在异常任务后调用 reset。

预期结果：

- `/reset_system` 返回 success。
- `/system_state` 出现 `resetting` 后恢复 `idle`。
- 若执行中 reset，当前任务应被终止。
- reset 后可以继续发送正常任务。

完成标准：

- 至少完成空闲 reset 和异常后 reset。
- 若时间允许，补充执行中 reset。

### C.7 可选实验：连续多轮正常任务

目的：

- 验证系统在多次任务下不会出现状态残留。

建议方法：

- 连续执行 `test-normal`、`test-alt`、`test-normal`。
- 每次任务之间观察 `/system_state` 是否回到 `idle`。

完成标准：

- 连续 3 次任务均能完成。
- 每次任务具有独立 `task_id`。
- `system_manager_node` 不错误保留上一个任务上下文。

## 7. 执行阶段 D：实验数据整理与结果分析

### D.1 建立实验记录表

在 `docs/experiments/experiment_records.md` 中建立统一记录表，建议字段：

```text
实验编号
实验名称
测试命令
目标关节角
speed_scale
timeout_sec
是否成功
最终状态
任务耗时
最终误差
rosbag 路径
截图路径
备注
```

完成标准：

- 每组正式实验都有记录。
- 成功和失败实验都记录，不只记录成功案例。
- 记录中能看出实验输入和实验结果之间的对应关系。

### D.2 提取核心评价指标

建议评价指标：

- 任务成功率。
- 平均任务耗时。
- 最终最大关节误差。
- 状态流转完整性。
- 异常任务识别结果。
- reset 后恢复能力。

统计方式：

- 正常任务和不同目标点任务用于统计成功率、耗时和误差。
- 慢速任务用于比较速度参数影响。
- 非法输入、超时、reset 用于验证异常处理能力。

完成标准：

- 至少形成一张实验结果汇总表。
- 至少形成一段对每类实验的文字分析。
- 论文中可以直接引用实验结果表。

### D.3 rosbag 数据处理

目标：

- 让实验具备可复现性和可追踪性。

建议处理方式：

1. 为每个 bag 添加说明，例如对应实验编号和命令。
2. 记录 bag 中包含哪些 topic。
3. 使用 `ros2 bag info` 查看 bag 概况。
4. 若需要，可从 `/task_state` 和 `/motion_event` 中提取状态变化时间。

建议命令：

```bash
ros2 bag info bags/graduation_test_xxx
```

完成标准：

- 正式论文引用的实验均有对应 bag。
- bag 路径和实验编号能一一对应。

### D.4 形成实验结论

实验结论至少包含：

- 系统能够完成完整运动控制主链路。
- 系统状态管理链路能够反映任务生命周期。
- 控制层具备基本到位判定、执行超时和反馈超时保护。
- 入口层具备输入校验能力。
- reset 服务可用于异常后恢复。
- 当前实现仍偏系统架构验证和关节空间简单规划，未实现复杂轨迹优化、动力学控制和真实机器人部署。

完成标准：

- 结论不夸大当前能力。
- 结论能支撑课题题目中的“架构设计与仿真验证”。

## 8. 执行阶段 E：论文与项目文档收口

### E.1 论文结构建议

建议论文按以下结构组织：

1. 绪论  
   说明研究背景、课题意义、研究内容和论文结构。

2. 相关技术基础  
   介绍 ROS2、Action/Topic/Service、Gazebo、UR5、ros2_control 等。

3. 系统需求与总体架构设计  
   说明系统目标、功能需求、非功能需求、分层架构和模块职责。

4. 系统接口与状态管理设计  
   说明自定义 msg/action/srv、topic 链路、任务状态机、系统状态机和事件收敛机制。

5. 运动规划与控制执行实现  
   说明关节空间线性插值、速度缩放、轨迹执行、误差判定、超时保护和接口适配。

6. 仿真实验与结果分析  
   说明实验环境、实验方案、实验数据、正常任务结果、异常任务结果和分析结论。

7. 总结与展望  
   总结成果，说明不足和后续可扩展方向。

### E.2 论文素材对应关系

可直接复用的文档：

- 架构设计：`docs/architecture/system_architecture.md`
- 数据流：`docs/architecture/data_flow.md`
- 状态机：`docs/architecture/state_machine.md`
- 接口设计：`docs/interfaces/*.md`
- 阶段总结：`docs/progress/stage1_summary.md`、`docs/progress/stage2_summary.md`
- 实验说明：`scripts/测试执行说明.md`
- 实验计划与记录：`docs/experiments/experiment_plan.md`、`docs/experiments/experiment_records.md`

完成标准：

- 每一章都有对应代码或文档依据。
- 论文中的系统能力与代码实现一致。
- 未实现功能放在“不足与展望”，不写成已完成成果。

### E.3 代码和仓库整理

需要整理：

- `README.md` 当前阶段描述。
- 各 `package.xml` 中 TODO 描述和 license。
- `scripts/README.md` 与实际脚本命令一致。
- `docs/experiments` 中补齐实验计划和记录。
- `bags/README.md` 中说明 bag 存放方式。

完成标准：

- 仓库首页能让老师快速理解项目。
- 文档索引完整。
- 常用启动、测试、录包命令清楚。

## 9. 执行阶段 F：最终演示、答辩材料和归档

### F.1 最终演示流程

建议最终演示控制在 8 到 12 分钟：

1. 展示项目 README 和系统架构图。
2. 启动 Gazebo + 系统节点。
3. 检查 `/joint_states` 和核心 topic。
4. 发送正常运动任务，展示 UR5 运动。
5. 展示 `/task_state` 从 `accepted` 到 `completed`。
6. 展示 rosbag 记录或已有 bag 信息。
7. 演示非法输入或超时失败。
8. 演示 reset。
9. 总结实验结果和系统不足。

完成标准：

- 演示流程不依赖临时手敲复杂命令。
- 关键命令都能从脚本执行。
- 即使现场仿真不稳定，也有截图、bag 和录屏作为备份材料。

### F.2 答辩 PPT 内容建议

建议 PPT 包含：

- 课题背景与研究目标。
- 技术路线。
- 系统总体架构。
- ROS2 功能包划分。
- 通信接口设计。
- 状态管理机制。
- 运动规划与控制流程。
- 仿真实验设计。
- 实验结果与分析。
- 总结与展望。

完成标准：

- PPT 不堆代码，重点展示架构、流程、结果。
- 每个核心结论都有实验或代码依据。
- 能清楚回答“你的创新点或工作量在哪里”。

### F.3 最终归档

最终提交前应归档：

- 源代码。
- README。
- 架构与接口文档。
- 实验计划与实验记录。
- rosbag 数据或数据说明。
- 关键截图。
- 论文终稿。
- 答辩 PPT。
- 演示视频或录屏。

完成标准：

- 换一台同样环境的机器，按照 README 和脚本能复现主要流程。
- 老师查看仓库时能迅速看到完成情况、运行方式和实验结果。

## 10. 推荐时间安排

若按 4 周推进，建议如下：

### 第 1 周：工程稳定与实验准备

- 完成构建清理和完整编译。
- 启动仿真和系统，确认脚本可用。
- 更新 README 和进度口径。
- 补齐 `docs/experiments/experiment_plan.md`。
- 固定实验参数和实验矩阵。

本周完成标准：

- 工程可编译。
- 系统可启动。
- 实验计划可执行。

### 第 2 周：完成主要仿真实验

- 执行正常任务实验。
- 执行不同目标点实验。
- 执行慢速任务实验。
- 执行非法输入实验。
- 执行超时实验。
- 执行 reset 实验。
- 每组实验记录 rosbag、截图和结果。

本周完成标准：

- 形成完整实验记录初稿。
- 至少有一组可用于答辩演示的稳定流程。

### 第 3 周：数据分析与论文主体

- 整理实验结果表。
- 分析任务耗时、最终误差、成功率和异常处理结果。
- 完成论文系统设计章节。
- 完成论文实现章节。
- 完成论文实验章节初稿。

本周完成标准：

- 论文主体内容基本成型。
- 实验结果能支撑论文结论。

### 第 4 周：收尾、答辩和归档

- 修改论文格式和文字。
- 完成 PPT。
- 录制或准备最终演示。
- 检查仓库文档和脚本。
- 做一次完整彩排。
- 归档最终材料。

本周完成标准：

- 论文终稿完成。
- PPT 完成。
- 演示流程稳定。
- 仓库材料完整。

若时间不足，优先级如下：

1. 保证工程可编译和可演示。
2. 保证正常任务、非法输入、超时、reset 四类实验有记录。
3. 保证论文中架构、实现、实验三部分完整。
4. 再补充更多重复实验和细粒度统计。

## 11. 风险与处理策略

### 风险 1：Gazebo 或 UR 仿真启动不稳定

处理策略：

- 固定启动脚本和工作空间路径。
- 启动前检查仿真工作空间是否 source。
- 记录一份成功启动的命令。
- 准备已录制视频和截图作为答辩备份。

### 风险 2：`/joint_states` 未发布

处理策略：

- 先检查 Gazebo 和控制器是否正常启动。
- 使用 `ros2 topic list` 确认话题存在。
- 使用 `ros2 topic echo /joint_states --once` 确认数据。
- 若无反馈，不进入正式实验。

### 风险 3：任务超时参数相互影响

处理策略：

- 明确区分 Action `timeout_sec`、controller 执行超时、manager watchdog。
- 正常实验使用较宽松超时。
- 超时实验单独设置很小 `timeout_sec`。
- 在论文中将多层超时解释为可靠性保护。

### 风险 4：现场演示失败

处理策略：

- 提前录制一版完整演示视频。
- 保留 rosbag、截图和终端输出。
- 答辩现场优先演示最稳定的正常任务。
- 异常实验可用录屏或截图说明。

### 风险 5：文档描述超过代码能力

处理策略：

- 所有成果描述都以当前代码为准。
- 未实现的抢占、真实机械臂对接、复杂规划算法写入展望。
- 避免使用“高精度控制”“最优轨迹”“真实工业部署”等过度表述。

## 12. 最终验收清单

### 工程验收

- [ ] `colcon build` 通过。
- [ ] `source install/setup.bash` 正常。
- [ ] `ros2 launch bringup_pkg bringup.launch.py launch_mode:=system_only` 可启动。
- [ ] `scripts/run_graduation_tests.sh launch-sim` 可启动仿真和系统。
- [ ] `/joint_states` 可正常接收。
- [ ] `/move_joints` Action 可用。
- [ ] `/reset_system` Service 可用。

### 功能验收

- [ ] 正常任务能完成。
- [ ] 不同目标点任务能完成。
- [ ] 慢速任务能体现速度缩放。
- [ ] 非法输入能被拒绝或正确处理。
- [ ] 超时任务能进入失败状态。
- [ ] reset 能恢复系统状态。
- [ ] 任务状态能通过 `/task_state` 观察。
- [ ] 系统状态能通过 `/system_state` 观察。

### 实验验收

- [ ] `experiment_plan.md` 已补齐。
- [ ] `experiment_records.md` 已补齐。
- [ ] 每组实验有命令、参数、结果和结论。
- [ ] 关键实验有 rosbag。
- [ ] 关键实验有截图或录屏。
- [ ] 已形成实验结果汇总表。

### 文档验收

- [ ] README 当前阶段描述准确。
- [ ] 架构文档与代码一致。
- [ ] 接口文档与 msg/action/srv 一致。
- [ ] 参数文档与 config 一致。
- [ ] 进度文档与实际进度一致。
- [ ] 论文引用的内容都能在代码或实验中找到依据。

### 答辩验收

- [ ] 论文终稿完成。
- [ ] 答辩 PPT 完成。
- [ ] 演示脚本稳定。
- [ ] 演示视频或截图备份完成。
- [ ] 能说明当前不足和后续扩展方向。

## 13. 当前最优下一步

从当前状态继续推进，建议立即按以下顺序做：

1. 清理旧构建产物并重新完整编译。
2. 更新 README 当前阶段描述。
3. 补齐 `docs/experiments/experiment_plan.md`。
4. 按实验 1 到实验 6 执行正式测试并记录。
5. 补齐 `docs/experiments/experiment_records.md`。
6. 根据实验结果写论文实验章节。
7. 整理最终 PPT 和演示材料。

这条路线能最大化利用当前已经完成的代码成果，把后续工作集中在毕业设计最终最看重的部分：可运行、可验证、可说明、可交付。
