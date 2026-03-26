"""
Launch file for Arctos hardware node.

This launches the main hardware control node that interfaces with the
physical Arctos robot arm via CAN bus.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    can_device_arg = DeclareLaunchArgument(
        'can_device',
        default_value='can0',
        description='CAN interface name or serial device path'
    )
    
    can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value='500000',
        description='CAN bus bitrate'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Joint state publish rate (Hz)'
    )
    
    coupled_axis_arg = DeclareLaunchArgument(
        'coupled_axis_mode',
        default_value='true',
        description='Enable differential wrist coupling'
    )
    
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Automatically enable motors on startup'
    )

    active_motor_ids_arg = DeclareLaunchArgument(
        'active_motor_ids',
        default_value='1,2,3,4,5,6',
        description='Comma-separated list of active motor CAN IDs'
    )
    
    # Hardware node
    hardware_node = Node(
        package='arctos_hardware',
        executable='arctos_hardware_node.py',
        name='arctos_hardware_node',
        output='screen',
        parameters=[{
            'can_device': LaunchConfiguration('can_device'),
            'can_bitrate': LaunchConfiguration('can_bitrate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'coupled_axis_mode': LaunchConfiguration('coupled_axis_mode'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'active_motor_ids': LaunchConfiguration('active_motor_ids'),
        }],
        remappings=[
            # Remap joint states to match expected topic
            ('/joint_states', '/joint_states'),
        ]
    )
    
    return LaunchDescription([
        can_device_arg,
        can_bitrate_arg,
        publish_rate_arg,
        coupled_axis_arg,
        auto_enable_arg,
        active_motor_ids_arg,
        hardware_node,
    ])
