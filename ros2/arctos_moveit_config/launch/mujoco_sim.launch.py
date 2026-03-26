"""
Launch MoveIt 2 with MuJoCo simulation backend via mujoco_ros2_control.

Data flow:
  MoveIt -> JointTrajectoryController -> mujoco_ros2_control/MujocoSystemInterface
    -> MuJoCo physics engine -> joint_state_broadcaster -> /joint_states -> MoveIt
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

MUJOCO_PLUGIN = "mujoco_ros2_control/MujocoSystemInterface"


def launch_setup(context, *args, **kwargs):
    urdf_pkg = get_package_share_directory("arctos_urdf_description")
    moveit_config_pkg = get_package_share_directory("arctos_moveit_config")

    urdf_file = os.path.join(urdf_pkg, "urdf", "arctos.urdf")
    mujoco_model = os.path.join(urdf_pkg, "mujoco", "arctos.xml")
    ros2_controllers_yaml = os.path.join(moveit_config_pkg, "config", "ros2_controllers.yaml")

    headless = LaunchConfiguration("headless").perform(context)

    robot_description_content = ParameterValue(
        Command([
            "xacro ", urdf_file,
            " hardware_plugin:=", MUJOCO_PLUGIN,
            " mujoco_model:=", mujoco_model,
            " fake_sensor_commands:=false",
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    hw_mappings = {
        "hardware_plugin": MUJOCO_PLUGIN,
        "mujoco_model": mujoco_model,
        "fake_sensor_commands": "false",
    }
    moveit_config = MoveItConfigsBuilder(
        "arctos_urdf",
        package_name="arctos_moveit_config",
    ).robot_description(
        mappings=hw_mappings,
    ).to_moveit_configs()

    nodes = []

    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    ))

    nodes.append(Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": True},
            ros2_controllers_yaml,
            {"headless": headless == "true"},
        ],
        on_exit=Shutdown(),
    ))

    controllers = ["joint_state_broadcaster", "arctos_arm_controller", "gripper_controller"]
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[c, "--param-file", ros2_controllers_yaml],
            output="screen",
        )
        for c in controllers
    ]

    nodes.append(TimerAction(
        period=3.0,
        actions=controller_spawners,
    ))

    moveit_dict = moveit_config.to_dict()
    moveit_dict["use_sim_time"] = True
    nodes.append(TimerAction(
        period=6.0,
        actions=[Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_dict],
        )],
    ))

    rviz_config_file = os.path.join(moveit_config_pkg, "config", "moveit.rviz")
    nodes.append(Node(
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
            {"use_sim_time": True},
        ],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run MuJoCo simulation without visualization window",
        ),
        OpaqueFunction(function=launch_setup),
    ])
