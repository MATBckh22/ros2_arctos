#!/usr/bin/env python3
"""
Isaac Sim ROS bridge.

Supported modes:
  sim_bridge
    /joint_states (from Isaac) -> /isaac_joint_states
    /isaac_joint_commands (from ros2_control) -> /joint_command

  visual_twin
    /joint_states (from real hardware/MoveIt) -> /joint_command (to Isaac)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IsaacSimBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')

        self.declare_parameter('mode', 'sim_bridge')

        self.declare_parameter('isaac_state_topic', '/joint_states')
        self.declare_parameter('moveit_state_topic', '/isaac_joint_states')
        self.declare_parameter('controller_command_topic', '/isaac_joint_commands')
        self.declare_parameter('isaac_command_topic', '/joint_command')

        self.declare_parameter('visual_twin_state_topic', '/joint_states')
        self.declare_parameter('command_rate', 30.0)
        self.declare_parameter('enable_filtering', True)
        self.declare_parameter('min_position_change', 0.001)

        self.mode = self.get_parameter('mode').value
        self.isaac_state_topic = self.get_parameter('isaac_state_topic').value
        self.moveit_state_topic = self.get_parameter('moveit_state_topic').value
        self.controller_command_topic = self.get_parameter('controller_command_topic').value
        self.isaac_command_topic = self.get_parameter('isaac_command_topic').value
        self.visual_twin_state_topic = self.get_parameter('visual_twin_state_topic').value
        self.command_rate = float(self.get_parameter('command_rate').value)
        self.enable_filtering = bool(self.get_parameter('enable_filtering').value)
        self.min_position_change = float(self.get_parameter('min_position_change').value)

        self.cmd_count = 0
        self.state_count = 0
        self._pending_visual_twin_msg: Optional[JointState] = None
        self._last_visual_twin_positions = None

        if self.mode == 'visual_twin':
            self._setup_visual_twin_mode()
        else:
            self._setup_sim_bridge_mode()

    def _setup_sim_bridge_mode(self):
        self.moveit_pub = self.create_publisher(
            JointState,
            self.moveit_state_topic,
            10,
        )
        self.isaac_cmd_pub = self.create_publisher(
            JointState,
            self.isaac_command_topic,
            10,
        )

        self.isaac_sub = self.create_subscription(
            JointState,
            self.isaac_state_topic,
            self.isaac_to_moveit_callback,
            10,
        )
        self.moveit_cmd_sub = self.create_subscription(
            JointState,
            self.controller_command_topic,
            self.moveit_to_isaac_callback,
            10,
        )

        self.get_logger().info('Isaac Sim Bridge started in sim_bridge mode')
        self.get_logger().info(
            f'Bridging: {self.isaac_state_topic} -> {self.moveit_state_topic}'
        )
        self.get_logger().info(
            f'Bridging: {self.controller_command_topic} -> {self.isaac_command_topic}'
        )

    def _setup_visual_twin_mode(self):
        self.isaac_cmd_pub = self.create_publisher(
            JointState,
            self.isaac_command_topic,
            10,
        )

        self.visual_twin_sub = self.create_subscription(
            JointState,
            self.visual_twin_state_topic,
            self.visual_twin_callback,
            10,
        )

        timer_period = 1.0 / max(self.command_rate, 1.0)
        self.visual_twin_timer = self.create_timer(
            timer_period,
            self.publish_visual_twin_command,
        )

        self.get_logger().info('Isaac Sim Bridge started in visual_twin mode')
        self.get_logger().info(
            f'Bridging: {self.visual_twin_state_topic} -> {self.isaac_command_topic}'
        )
        self.get_logger().info(
            f'Publish rate: {self.command_rate:.1f} Hz | '
            f'Filtering: {self.enable_filtering} | '
            f'Min change: {self.min_position_change}'
        )

    def isaac_to_moveit_callback(self, msg: JointState):
        """Forward Isaac Sim joint states to ros2_control."""
        self.moveit_pub.publish(msg)
        self.state_count += 1

        if self.state_count % 200 == 0:
            self.get_logger().info(
                f'States: {self.state_count} | Commands: {self.cmd_count}',
                throttle_duration_sec=5.0,
            )

    def moveit_to_isaac_callback(self, msg: JointState):
        """Forward ros2_control commands to Isaac Sim."""
        self.isaac_cmd_pub.publish(msg)
        self.cmd_count += 1

        if self.cmd_count % 100 == 0:
            self.get_logger().info(
                f'Cmd {self.cmd_count}: pos={[round(p, 3) for p in msg.position[:3]]}...',
                throttle_duration_sec=2.0,
            )

    def visual_twin_callback(self, msg: JointState):
        """Track the latest real-robot state for Isaac visual twin playback."""
        if not msg.position:
            return

        if self.enable_filtering and self._last_visual_twin_positions is not None:
            max_delta = max(
                abs(a - b)
                for a, b in zip(msg.position, self._last_visual_twin_positions)
            )
            if max_delta < self.min_position_change:
                return

        twin_msg = JointState()
        twin_msg.header = msg.header
        twin_msg.name = list(msg.name)
        twin_msg.position = list(msg.position)
        twin_msg.velocity = list(msg.velocity)
        twin_msg.effort = list(msg.effort)

        self._pending_visual_twin_msg = twin_msg
        self._last_visual_twin_positions = list(msg.position)
        self.state_count += 1

    def publish_visual_twin_command(self):
        """Republish the latest real-robot state to Isaac at a stable rate."""
        if self._pending_visual_twin_msg is None:
            return

        msg = self._pending_visual_twin_msg
        if not msg.header.stamp.sec and not msg.header.stamp.nanosec:
            msg.header.stamp = self.get_clock().now().to_msg()

        self.isaac_cmd_pub.publish(msg)
        self.cmd_count += 1

        if self.cmd_count % 100 == 0:
            pos_preview = [round(math.degrees(p), 1) for p in msg.position[:6]]
            self.get_logger().info(
                f'Visual twin cmd {self.cmd_count}: deg={pos_preview}',
                throttle_duration_sec=2.0,
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
