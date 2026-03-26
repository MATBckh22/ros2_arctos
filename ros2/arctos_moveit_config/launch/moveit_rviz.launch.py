import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arctos_urdf", package_name="arctos_moveit_config").to_moveit_configs()
    moveit_config_pkg = os.path.dirname(os.path.dirname(__file__))
    rviz_config_file = os.path.join(moveit_config_pkg, "config", "moveit.rviz")

    return LaunchDescription([
        Node(
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
    ])
