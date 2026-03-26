"""
Complete Isaac Sim + MoveIt2 + RViz Launch File

Launches everything needed for Isaac Sim integration with proper kinematics.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    
    # Launch arguments
    bridge_rate_arg = DeclareLaunchArgument(
        'bridge_command_rate',
        default_value='50.0',
        description='Isaac Sim bridge command rate (Hz)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # Build MoveIt config (includes kinematics!)
    moveit_config = MoveItConfigsBuilder(
        "arctos_urdf", 
        package_name="arctos_moveit_config"
    ).to_moveit_configs()
    
    moveit_config_pkg = get_package_share_directory("arctos_moveit_config")
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    
    # MoveGroup node with full config
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    # Isaac Sim bridge with rate limiting
    isaac_bridge = Node(
        package='arctos_moveit_config',
        executable='isaac_sim_bridge.py',
        name='isaac_sim_bridge',
        output='screen',
        parameters=[{
            'command_rate': LaunchConfiguration('bridge_command_rate'),
            'enable_filtering': True,
            'min_position_change': 0.001,
        }]
    )
    
    # RViz with FULL MoveIt config (including kinematics!)
    rviz_config_file = os.path.join(moveit_config_pkg, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Delay move_group slightly to let RSP start
    delayed_move_group = TimerAction(
        period=2.0,
        actions=[move_group_node],
    )
    
    return LaunchDescription([
        # Arguments
        bridge_rate_arg,
        use_rviz_arg,
        
        # Nodes
        robot_state_publisher,
        delayed_move_group,
        isaac_bridge,
        rviz_node,
    ])
