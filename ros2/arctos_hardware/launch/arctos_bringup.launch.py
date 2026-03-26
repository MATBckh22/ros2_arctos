"""
Full bringup launch file for Arctos robot — LEGACY.

This launches the legacy arctos_hardware_node.py path which bypasses
ros2_control. For ros2_control + MoveIt integration, use
real_hardware.launch.py in arctos_moveit_config instead.

Kept for backward compatibility and direct hardware testing.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Try to find MoveIt config package
    try:
        moveit_config_dir = get_package_share_directory('arctos_moveit_config')
        has_moveit_config = True
    except Exception:
        moveit_config_dir = ''
        has_moveit_config = False
    
    # ===== Launch Arguments =====
    
    can_device_arg = DeclareLaunchArgument(
        'can_device',
        default_value='can0',
        description='CAN interface name or serial device path'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Launch MoveIt2 move_group'
    )
    
    use_isaac_sim_arg = DeclareLaunchArgument(
        'use_isaac_sim',
        default_value='false',
        description='Include Isaac Sim bridge'
    )
    
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Auto-enable motors on startup'
    )

    active_motor_ids_arg = DeclareLaunchArgument(
        'active_motor_ids',
        default_value='1,2,3,4,5,6',
        description='Comma-separated list of active motor CAN IDs'
    )
    
    # ===== Nodes =====
    use_visualization = IfCondition(
        PythonExpression([
            "'",
            LaunchConfiguration('use_moveit'),
            "' == 'true' or '",
            LaunchConfiguration('use_rviz'),
            "' == 'true'",
        ])
    )
    
    # Hardware node
    hardware_node = Node(
        package='arctos_hardware',
        executable='arctos_hardware_node.py',
        name='arctos_hardware_node',
        output='screen',
        parameters=[{
            'can_device': LaunchConfiguration('can_device'),
            'can_bitrate': 500000,
            'publish_rate': 15.0,
            'coupled_axis_mode': True,
            'auto_enable': LaunchConfiguration('auto_enable'),
            'active_motor_ids': LaunchConfiguration('active_motor_ids'),
        }]
    )
    
    # Robot state publisher (from MoveIt config)
    robot_state_publisher_launch = None
    if has_moveit_config:
        robot_state_publisher_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                moveit_config_dir, '/launch/rsp.launch.py'
            ]),
            condition=use_visualization
        )
    
    # MoveIt move_group (from MoveIt config)
    move_group_launch = None
    if has_moveit_config:
        move_group_launch = TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        moveit_config_dir, '/launch/move_group.launch.py'
                    ]),
                    condition=IfCondition(LaunchConfiguration('use_moveit'))
                )
            ],
        )
    
    # RViz
    rviz_launch = None
    if has_moveit_config:
        rviz_launch = TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        moveit_config_dir, '/launch/moveit_rviz.launch.py'
                    ]),
                    condition=IfCondition(LaunchConfiguration('use_rviz'))
                )
            ],
        )
    
    # Isaac Sim bridge
    isaac_sim_bridge = Node(
        package='arctos_moveit_config',
        executable='isaac_sim_bridge.py',
        name='isaac_sim_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_isaac_sim'))
    )
    
    # ===== Build Launch Description =====
    
    ld = LaunchDescription([
        # Arguments
        can_device_arg,
        use_rviz_arg,
        use_moveit_arg,
        use_isaac_sim_arg,
        auto_enable_arg,
        active_motor_ids_arg,
        
        # Hardware
        hardware_node,
    ])
    
    # Add optional launches
    if robot_state_publisher_launch:
        ld.add_action(robot_state_publisher_launch)
    
    if move_group_launch:
        ld.add_action(move_group_launch)
    
    if rviz_launch:
        ld.add_action(rviz_launch)
    
    ld.add_action(isaac_sim_bridge)
    
    return ld
