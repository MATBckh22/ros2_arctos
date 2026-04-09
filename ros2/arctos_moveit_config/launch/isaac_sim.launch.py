"""
Launch MoveIt2 with Isaac Sim integration via topic_based_ros2_control
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

ISAAC_HW_PLUGIN = "joint_state_topic_hardware_interface/JointStateTopicSystem"
ISAAC_CMD_TOPIC = "/isaac_joint_commands"
ISAAC_STATE_TOPIC = "/isaac_joint_states"


def generate_launch_description():
    urdf_pkg = get_package_share_directory("arctos_urdf_description")
    urdf_file = os.path.join(urdf_pkg, "urdf", "arctos.urdf")

    robot_description_content = ParameterValue(
        Command([
            "xacro ", urdf_file,
            " hardware_plugin:=", ISAAC_HW_PLUGIN,
            " joint_commands_topic:=", ISAAC_CMD_TOPIC,
            " joint_states_topic:=", ISAAC_STATE_TOPIC,
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    hw_mappings = {
        "hardware_plugin": ISAAC_HW_PLUGIN,
        "joint_commands_topic": ISAAC_CMD_TOPIC,
        "joint_states_topic": ISAAC_STATE_TOPIC,
    }
    moveit_config = MoveItConfigsBuilder(
        "arctos_urdf",
        package_name="arctos_moveit_config"
    ).robot_description(
        mappings=hw_mappings,
    ).to_moveit_configs()

    moveit_config_pkg = get_package_share_directory("arctos_moveit_config")
    ros2_controllers_yaml = os.path.join(moveit_config_pkg, "config", "ros2_controllers.yaml")

    moveit_dict = moveit_config.to_dict()
    trajectory_execution = {
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.1,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_yaml,
        ],
        output="screen",
    )

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

    moveit_dict.update(trajectory_execution)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

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

    isaac_bridge_node = Node(
        package="arctos_moveit_config",
        executable="isaac_sim_bridge.py",
        name="isaac_sim_bridge",
        output="screen",
    )

    delayed_spawners = TimerAction(
        period=2.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ],
    )

    delayed_move_group = TimerAction(
        period=5.0,
        actions=[move_group_node],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_spawners,
        delayed_move_group,
        rviz_node,
        isaac_bridge_node,
    ])
