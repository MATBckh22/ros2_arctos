from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_device_arg = DeclareLaunchArgument(
        "can_device",
        default_value="can0",
        description="CAN interface device name",
    )

    default_pump_pwm_arg = DeclareLaunchArgument(
        "default_pump_pwm",
        default_value="200",
        description="Default pump PWM for /suction/activate",
    )

    status_poll_rate_arg = DeclareLaunchArgument(
        "status_poll_rate",
        default_value="2.0",
        description="Status polling rate in Hz",
    )

    watchdog_timeout_arg = DeclareLaunchArgument(
        "watchdog_timeout_100ms",
        default_value="10",
        description="Arduino watchdog timeout in 100 ms units",
    )

    suction_driver_node = Node(
        package="arctos_hardware",
        executable="suction_driver.py",
        name="suction_driver",
        output="screen",
        parameters=[
            {
                "can_device": LaunchConfiguration("can_device"),
                "default_pump_pwm": LaunchConfiguration("default_pump_pwm"),
                "status_poll_rate": LaunchConfiguration("status_poll_rate"),
                "watchdog_timeout_100ms": LaunchConfiguration("watchdog_timeout_100ms"),
            }
        ],
    )

    suction_gripper_action_node = Node(
        package="arctos_hardware",
        executable="suction_gripper_action.py",
        name="suction_gripper_action",
        output="screen",
    )

    return LaunchDescription(
        [
            can_device_arg,
            default_pump_pwm_arg,
            status_poll_rate_arg,
            watchdog_timeout_arg,
            suction_driver_node,
            suction_gripper_action_node,
        ]
    )
