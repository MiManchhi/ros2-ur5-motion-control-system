# 辅助脚本

- `build_for_vscode.sh`: 使用 `colcon build` 构建，并自动更新 VSCode/clangd 使用的根目录 `compile_commands.json`。
- `merge_compile_commands.sh`: 合并各包构建目录下的 `compile_commands.json`。
- `run_graduation_tests.sh`: 启动系统/仿真、发送正常/快速/慢速/异常/恢复测试任务、调用 reset，并可录制 rosbag。运动测试会打印估算规划时长和 Action 端到端真实耗时。
- `plot_joint_states_from_bag.py`: 从 rosbag 读取 `/joint_states` 并绘制关节角变化曲线。
- `plot_max_joint_error_from_bag.py`: 从 rosbag 计算并绘制最大关节误差曲线。
- `plot_task_duration_comparison_from_bag.py`: 从 rosbag 汇总 `speed_scale` 与规划轨迹时长，用于生成速度对比图。
