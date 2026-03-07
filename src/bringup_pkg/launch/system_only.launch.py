from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # =========================
    # 阶段2：只启动系统节点
    # 不包含 Gazebo / ros2_control 启动逻辑
    # =========================

    motion_api_node = Node(
        package='motion_api_pkg',
        executable='motion_api_node',
        name='motion_api_node',
        output='screen',
        parameters=['src/motion_api_pkg/config/motion_api_params.yaml']
    )

    planner_node = Node(
        package='planner_pkg',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=['src/planner_pkg/config/planner_params.yaml']
    )

    controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=['src/controller_pkg/config/controller_params.yaml']
    )

    robot_interface_node = Node(
        package='robot_interface_pkg',
        executable='robot_interface_node',
        name='robot_interface_node',
        output='screen',
        parameters=['src/robot_interface_pkg/config/interface_params.yaml']
    )

    system_manager_node = Node(
        package='system_manager_pkg',
        executable='system_manager_node',
        name='system_manager_node',
        output='screen',
        parameters=['src/system_manager_pkg/config/manager_params.yaml']
    )

    return LaunchDescription([
        motion_api_node,
        planner_node,
        controller_node,
        robot_interface_node,
        system_manager_node,
    ])