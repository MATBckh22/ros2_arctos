#!/usr/bin/env python3
"""
Isaac Sim ROS bridge: bidirectional JointState remapping for MoveIt + ros2_control.

- /joint_states (from Isaac)        -> /isaac_joint_states
- /isaac_joint_commands (from ros2_control) -> /joint_command
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IsaacSimBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')

        self.declare_parameter('isaac_state_topic', '/joint_states')
        self.declare_parameter('moveit_state_topic', '/isaac_joint_states')
        self.declare_parameter('controller_command_topic', '/isaac_joint_commands')
        self.declare_parameter('isaac_command_topic', '/joint_command')

        self.isaac_state_topic = self.get_parameter('isaac_state_topic').value
        self.moveit_state_topic = self.get_parameter('moveit_state_topic').value
        self.controller_command_topic = self.get_parameter('controller_command_topic').value
        self.isaac_command_topic = self.get_parameter('isaac_command_topic').value

        self.cmd_count = 0
        self.state_count = 0

        self.moveit_pub = self.create_publisher(
            JointState,
            self.moveit_state_topic,
            10
        )
        self.isaac_cmd_pub = self.create_publisher(
            JointState,
            self.isaac_command_topic,
            10
        )

        self.isaac_sub = self.create_subscription(
            JointState,
            self.isaac_state_topic,
            self.isaac_to_moveit_callback,
            10
        )
        self.moveit_cmd_sub = self.create_subscription(
            JointState,
            self.controller_command_topic,
            self.moveit_to_isaac_callback,
            10
        )

        self.get_logger().info('Isaac Sim Bridge started')
        self.get_logger().info(
            f'Bridging: {self.isaac_state_topic} -> {self.moveit_state_topic}'
        )
        self.get_logger().info(
            f'Bridging: {self.controller_command_topic} -> {self.isaac_command_topic}'
        )

    def isaac_to_moveit_callback(self, msg: JointState):
        """Forward Isaac Sim joint states to ros2_control."""
        self.moveit_pub.publish(msg)
        self.state_count += 1

        if self.state_count % 200 == 0:
            self.get_logger().info(
                f'States: {self.state_count} | Commands: {self.cmd_count}',
                throttle_duration_sec=5.0
            )

    def moveit_to_isaac_callback(self, msg: JointState):
        """Forward ros2_control commands to Isaac Sim."""
        self.isaac_cmd_pub.publish(msg)
        self.cmd_count += 1

        if self.cmd_count % 100 == 0:
            self.get_logger().info(
                f'Cmd {self.cmd_count}: pos={[round(p,3) for p in msg.position[:3]]}...',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
