from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # =========================
    # 总入口：当前阶段先默认启动 system_only
    # 后续阶段可以在这里做模式切换
    # =========================

    system_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('src/bringup_pkg/launch/system_only.launch.py')
    )

    return LaunchDescription([
        system_only
    ])