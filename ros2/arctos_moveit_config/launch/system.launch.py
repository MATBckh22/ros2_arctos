"""
Unified Arctos launch entrypoint with mode selection.

Supported modes:
  demo              - MoveIt demo / mock hardware
  isaac_sim         - Isaac Sim as ros2_control backend
  real              - Real hardware via CAN bridge + MoveIt
  real_with_isaac   - Real hardware + Isaac visual twin
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _mode_is(*values):
    expr = []
    for i, value in enumerate(values):
        if i:
            expr.append(" or ")
        expr.extend([
            "'",
            LaunchConfiguration('mode'),
            "' == '",
            value,
            "'",
        ])
    return IfCondition(PythonExpression(expr))


def generate_launch_description():
    moveit_pkg = get_package_share_directory('arctos_moveit_config')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='One of: demo, isaac_sim, real, real_with_isaac'
    )
    can_device_arg = DeclareLaunchArgument(
        'can_device',
        default_value='can0',
        description='CAN interface used in real-hardware modes'
    )
    use_suction_arg = DeclareLaunchArgument(
        'use_suction',
        default_value='false',
        description='Launch suction gripper nodes in real-hardware modes'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz where supported'
    )
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Launch move_group where supported'
    )
    bridge_rate_arg = DeclareLaunchArgument(
        'isaac_bridge_command_rate',
        default_value='30.0',
        description='Command publish rate for Isaac bridge modes'
    )

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_pkg, '/launch/demo.launch.py']),
        condition=_mode_is('demo'),
    )

    isaac_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_pkg, '/launch/isaac_sim.launch.py']),
        condition=_mode_is('isaac_sim'),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_moveit': LaunchConfiguration('use_moveit'),
        }.items(),
    )

    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_pkg, '/launch/real_hardware.launch.py']),
        condition=_mode_is('real', 'real_with_isaac'),
        launch_arguments={
            'can_device': LaunchConfiguration('can_device'),
            'use_suction': LaunchConfiguration('use_suction'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_moveit': LaunchConfiguration('use_moveit'),
        }.items(),
    )

    isaac_visual_twin = Node(
        package='arctos_moveit_config',
        executable='isaac_sim_bridge.py',
        name='isaac_visual_twin_bridge',
        output='screen',
        condition=_mode_is('real_with_isaac'),
        parameters=[{
            'mode': 'visual_twin',
            'command_rate': LaunchConfiguration('isaac_bridge_command_rate'),
            'enable_filtering': True,
            'min_position_change': 0.001,
            'visual_twin_state_topic': '/joint_states',
            'isaac_command_topic': '/joint_command',
        }],
    )

    return LaunchDescription([
        mode_arg,
        can_device_arg,
        use_suction_arg,
        use_rviz_arg,
        use_moveit_arg,
        bridge_rate_arg,
        demo_launch,
        isaac_sim_launch,
        real_launch,
        isaac_visual_twin,
    ])
