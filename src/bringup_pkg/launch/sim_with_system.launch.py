from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def _launch_setup(context, *args, **kwargs):
    """
    联合启动入口。

    当前设计目标：
    1. 可只启动系统节点
    2. 可在提供外部仿真 launch 文件时，先启动仿真，再延迟启动系统节点

    这样做的好处：
    - 现在就能用，不依赖你必须立刻把 UR/Gazebo launch 完全接进 bringup_pkg
    - 后面接正式仿真 launch 时，也不用重写整体结构
    """

    bringup_share = get_package_share_directory('bringup_pkg')
    system_only_launch = os.path.join(
        bringup_share, 'launch', 'system_only.launch.py'
    )

    start_sim_raw = LaunchConfiguration('start_sim').perform(context).strip().lower()
    sim_launch_file_raw = LaunchConfiguration('sim_launch_file').perform(context).strip()

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    system_only_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(system_only_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'log_level': log_level,
        }.items()
    )

    # --------------------------------------------------
    # 情况 1：不启动仿真，直接只起系统节点
    # --------------------------------------------------
    if start_sim_raw not in ('true', '1', 'yes'):
        return [
            LogInfo(msg='[bringup] sim_with_system 模式：当前未启用外部仿真，仅启动系统节点'),
            system_only_include
        ]

    # --------------------------------------------------
    # 情况 2：要求启仿真，但没给仿真 launch 文件
    # --------------------------------------------------
    if not sim_launch_file_raw:
        return [
            LogInfo(msg='[bringup] start_sim=true，但未提供 sim_launch_file，当前仅启动系统节点'),
            system_only_include
        ]

    # --------------------------------------------------
    # 情况 3：启用仿真，并提供了外部仿真 launch 文件
    #
    # 这里先 include 外部仿真 launch，
    # 再延迟 3 秒启动系统节点，给底层控制器和 /joint_states 一点准备时间。
    # --------------------------------------------------
    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_file_raw)
    )

    delayed_system = TimerAction(
        period=3.0,
        actions=[system_only_include]
    )

    return [
        LogInfo(msg=['[bringup] 已启用外部仿真 launch：', sim_launch_file_raw]),
        LogInfo(msg='[bringup] 先启动仿真环境，3 秒后启动系统节点'),
        sim_include,
        delayed_system
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时间'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS 日志级别'
        ),
        DeclareLaunchArgument(
            'start_sim',
            default_value='false',
            description='是否同时启动外部仿真 launch'
        ),
        DeclareLaunchArgument(
            'sim_launch_file',
            default_value='',
            description='外部仿真 launch 文件完整路径'
        ),
        OpaqueFunction(function=_launch_setup)
    ])