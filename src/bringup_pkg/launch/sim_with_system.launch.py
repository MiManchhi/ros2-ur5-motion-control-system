from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # =========================
    # 阶段2预留：系统 + 仿真 联动启动入口
    # 当前先只 include 系统节点启动
    # 后续你可以在这里接入 UR / Gazebo 的 launch 文件
    # =========================

    system_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('src/bringup_pkg/launch/system_only.launch.py')
    )

    return LaunchDescription([
        LogInfo(msg='Launching system nodes. Gazebo/UR simulation launch can be added here later.'),
        system_only
    ])