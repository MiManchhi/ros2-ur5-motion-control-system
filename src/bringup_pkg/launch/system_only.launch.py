from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """
    只启动系统业务节点，不包含 Gazebo / ros2_control / UR 驱动本身。

    当前启动节点：
    1. motion_api_node
    2. planner_node
    3. controller_node
    4. robot_interface_node
    5. system_manager_node

    说明：
    - 该 launch 适合“系统层单独调试”
    - 若要真正驱动仿真机械臂，还需要外部先提供 /joint_states 和底层控制器
    """

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    # --------------------------------------------------
    # 参数文件路径
    # 使用 package share 路径，不再写死 src/... 形式
    # --------------------------------------------------
    motion_api_params = os.path.join(
        get_package_share_directory('motion_api_pkg'),
        'config',
        'motion_api_params.yaml'
    )

    planner_params = os.path.join(
        get_package_share_directory('planner_pkg'),
        'config',
        'planner_params.yaml'
    )

    controller_params = os.path.join(
        get_package_share_directory('controller_pkg'),
        'config',
        'controller_params.yaml'
    )

    interface_params = os.path.join(
        get_package_share_directory('robot_interface_pkg'),
        'config',
        'interface_params.yaml'
    )

    manager_params = os.path.join(
        get_package_share_directory('system_manager_pkg'),
        'config',
        'manager_params.yaml'
    )

    # --------------------------------------------------
    # 各节点定义
    # --------------------------------------------------
    motion_api_node = Node(
        package='motion_api_pkg',
        executable='motion_api_node',
        name='motion_api_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            motion_api_params,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    planner_node = Node(
        package='planner_pkg',
        executable='planner_node',
        name='planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            planner_params,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            controller_params,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    robot_interface_node = Node(
        package='robot_interface_pkg',
        executable='robot_interface_node',
        name='robot_interface_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            interface_params,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    system_manager_node = Node(
        package='system_manager_pkg',
        executable='system_manager_node',
        name='system_manager_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            manager_params,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS 日志级别'
        ),

        LogInfo(msg=[
            '[bringup] 启动 system_only 模式，use_sim_time=',
            use_sim_time,
            '，log_level=',
            log_level
        ]),

        motion_api_node,
        planner_node,
        controller_node,
        robot_interface_node,
        system_manager_node,
    ])