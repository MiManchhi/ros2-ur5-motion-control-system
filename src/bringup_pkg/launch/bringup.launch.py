from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def _launch_setup(context, *args, **kwargs):
    """
    根据 launch_mode 动态选择实际启动入口。

    支持两种模式：
    1. system_only      -> 只启动系统五个核心节点
    2. sim_with_system  -> 启动“仿真 + 系统”联合入口
    """

    bringup_share = get_package_share_directory('bringup_pkg')

    system_only_launch = os.path.join(
        bringup_share, 'launch', 'system_only.launch.py'
    )
    sim_with_system_launch = os.path.join(
        bringup_share, 'launch', 'sim_with_system.launch.py'
    )

    launch_mode = LaunchConfiguration('launch_mode').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    start_sim = LaunchConfiguration('start_sim')
    sim_launch_file = LaunchConfiguration('sim_launch_file')

    # 通用参数
    common_args = {
        'use_sim_time': use_sim_time,
        'log_level': log_level,
    }

    # 模式选择
    if launch_mode == 'system_only':
        return [
            LogInfo(msg='[bringup] 启动模式：system_only（仅系统节点）'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(system_only_launch),
                launch_arguments=common_args.items()
            )
        ]

    if launch_mode == 'sim_with_system':
        sim_args = {
            'use_sim_time': use_sim_time,
            'log_level': log_level,
            'start_sim': start_sim,
            'sim_launch_file': sim_launch_file,
        }
        return [
            LogInfo(msg='[bringup] 启动模式：sim_with_system（系统 + 可选仿真）'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_with_system_launch),
                launch_arguments=sim_args.items()
            )
        ]

    # 未知模式时，回退到 system_only，避免直接启动失败
    return [
        LogInfo(msg=[
            '[bringup] 未识别的 launch_mode=',
            launch_mode,
            '，已自动回退为 system_only'
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_only_launch),
            launch_arguments=common_args.items()
        )
    ]


def generate_launch_description():
    """
    bringup 总入口。

    推荐使用方式：
    ros2 launch bringup_pkg bringup.launch.py
    ros2 launch bringup_pkg bringup.launch.py launch_mode:=system_only
    ros2 launch bringup_pkg bringup.launch.py launch_mode:=sim_with_system start_sim:=true sim_launch_file:=/xxx/xxx.launch.py
    """

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_mode',
            default_value='system_only',
            description='启动模式：system_only 或 sim_with_system'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS 日志级别，例如 debug / info / warn / error'
        ),
        DeclareLaunchArgument(
            'start_sim',
            default_value='false',
            description='在 sim_with_system 模式下，是否同时启动外部仿真 launch'
        ),
        DeclareLaunchArgument(
            'sim_launch_file',
            default_value='',
            description='外部仿真 launch 文件完整路径，例如 /home/xxx/ur_sim.launch.py'
        ),
        OpaqueFunction(function=_launch_setup)
    ])