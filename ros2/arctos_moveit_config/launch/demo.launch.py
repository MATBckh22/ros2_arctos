import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build moveit config
    moveit_config = MoveItConfigsBuilder(
        "arctos_urdf", 
        package_name="arctos_moveit_config"
    ).to_moveit_configs()
    
    moveit_config_pkg = get_package_share_directory("arctos_moveit_config")
    ros2_controllers_yaml = os.path.join(moveit_config_pkg, "config", "ros2_controllers.yaml")
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    
    # ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_yaml,
        ],
        output="screen",
    )
    
    # Spawners (delayed to let controller_manager start)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_arm_controller"],
        output="screen",
    )
    
    # MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    # RViz
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
    )
    
    # Delayed start for spawners
    delayed_spawners = TimerAction(
        period=2.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ],
    )
    
    # Delayed start for move_group
    delayed_move_group = TimerAction(
        period=4.0,
        actions=[move_group_node],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_spawners,
        delayed_move_group,
        rviz_node,
    ])
