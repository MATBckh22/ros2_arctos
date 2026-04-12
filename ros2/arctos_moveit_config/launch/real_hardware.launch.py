"""
Launch MoveIt 2 with real Arctos hardware via topic_based_ros2_control.

Data flow:
  MoveIt -> JointTrajectoryController -> topic_based_ros2_control
    -> /arctos/joint_commands -> arctos_can_bridge -> CAN bus -> motors
    -> CAN bus -> arctos_can_bridge -> /arctos/joint_states
    -> topic_based_ros2_control -> joint_state_broadcaster -> /joint_states -> MoveIt

"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

HW_PLUGIN = "joint_state_topic_hardware_interface/JointStateTopicSystem"
CMD_TOPIC = "/arctos/joint_commands"
STATE_TOPIC = "/arctos/joint_states"


def generate_launch_description():
    can_device_arg = DeclareLaunchArgument(
        'can_device', default_value='can0',
        description='CAN interface device name'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz'
    )
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit', default_value='true',
        description='Launch move_group'
    )

    urdf_pkg = get_package_share_directory("arctos_urdf_description")
    urdf_file = os.path.join(urdf_pkg, "urdf", "arctos.urdf")

    robot_description_content = ParameterValue(
        Command([
            "xacro ", urdf_file,
            " hardware_plugin:=", HW_PLUGIN,
            " joint_commands_topic:=", CMD_TOPIC,
            " joint_states_topic:=", STATE_TOPIC,
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    hw_mappings = {
        "hardware_plugin": HW_PLUGIN,
        "joint_commands_topic": CMD_TOPIC,
        "joint_states_topic": STATE_TOPIC,
    }
    moveit_config = MoveItConfigsBuilder(
        "arctos_urdf",
        package_name="arctos_moveit_config"
    ).robot_description(
        mappings=hw_mappings,
    ).to_moveit_configs()

    moveit_config_pkg = get_package_share_directory("arctos_moveit_config")
    ros2_controllers_yaml = os.path.join(
        moveit_config_pkg, "config", "ros2_controllers_real.yaml"
    )
    moveit_controllers_real_yaml = os.path.join(
        moveit_config_pkg, "config", "moveit_controllers_real.yaml"
    )

    moveit_dict = moveit_config.to_dict()
    with open(moveit_controllers_real_yaml, 'r', encoding='utf-8') as f:
        moveit_dict.update(yaml.safe_load(f))
    trajectory_execution = {
        "trajectory_execution.allowed_execution_duration_scaling": 4.0,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        "trajectory_execution.allowed_start_tolerance": 0.15,
        "trajectory_execution.execution_duration_monitoring": True,
    }
    moveit_dict.update(trajectory_execution)

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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_dict],
        arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(LaunchConfiguration("use_moveit")),
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
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    can_bridge_node = Node(
        package="arctos_hardware",
        executable="arctos_can_bridge.py",
        name="arctos_can_bridge",
        output="screen",
        parameters=[{
            "can_device": LaunchConfiguration("can_device"),
            "can_bitrate": 500000,
            "coupled_axis_mode": True,
            "state_publish_rate": 5.0,
            "command_send_rate": 50.0,
            "command_timeout": 2.0,
            "active_joints": [1, 2, 3, 4, 5, 6],
            "state_joint_signs": [1.0, 1.0, -1.0, 1.0, -1.0, 1.0],
        }],
    )

    suction_driver_node = Node(
        package="arctos_hardware",
        executable="suction_driver.py",
        name="suction_driver",
        output="screen",
        parameters=[{
            "can_device": LaunchConfiguration("can_device"),
            "default_pump_pwm": 200,
            "status_poll_rate": 2.0,
            "watchdog_timeout_100ms": 10,
        }],
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
        can_device_arg,
        use_rviz_arg,
        use_moveit_arg,
        robot_state_publisher,
        ros2_control_node,
        can_bridge_node,
        suction_driver_node,
        delayed_spawners,
        delayed_move_group,
        rviz_node,
    ])
