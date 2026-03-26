from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arctos_urdf", package_name="arctos_moveit_config").to_moveit_configs()
    moveit_dict = moveit_config.to_dict()
    moveit_dict.update({
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 10.0,
        "trajectory_execution.allowed_start_tolerance": 0.1,
        "trajectory_execution.execution_duration_monitoring": False,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    })

    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_dict],
        )
    ])
