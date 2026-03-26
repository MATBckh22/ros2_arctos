#!/usr/bin/env python3
"""
Arctos ros2_control Hardware Interface.

This is a Python-based ros2_control hardware interface that allows MoveIt2
to control the Arctos robot through the standard ros2_control framework.

Note: For production, consider implementing this in C++ for better real-time
performance. This Python version is suitable for development and testing.
"""

import math
import time
import threading
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from arctos_hardware.arctos_controller import ArctosController, ArctosConfig
from arctos_hardware.can_interface import default_can_device


class ArctosHardwareInterface(Node):
    """
    ros2_control style hardware interface for Arctos robot.
    
    This node bridges ros2_controllers and the actual hardware by:
    - Subscribing to joint commands from controllers
    - Publishing joint states for feedback
    
    Topics (default namespace: /arctos):
        Subscriptions:
            ~/commands/position: Position commands (Float64MultiArray)
            ~/commands/velocity: Velocity commands (Float64MultiArray)
        Publishers:
            /joint_states: Current joint states
    """
    
    JOINT_NAMES = [
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
    ]
    
    def __init__(self):
        super().__init__('arctos_hardware_interface')
        
        # Declare parameters
        self.declare_parameter('can_device', default_can_device())
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('coupled_axis_mode', True)
        self.declare_parameter('position_mode', True)  # vs velocity mode
        self.declare_parameter('command_timeout', 0.5)  # seconds
        
        # Get parameters
        can_device = self.get_parameter('can_device').value
        can_bitrate = self.get_parameter('can_bitrate').value
        self.update_rate = self.get_parameter('update_rate').value
        coupled_mode = self.get_parameter('coupled_axis_mode').value
        self.position_mode = self.get_parameter('position_mode').value
        self.command_timeout = self.get_parameter('command_timeout').value
        
        # Create configuration
        config = ArctosConfig(
            can_device=can_device,
            can_bitrate=can_bitrate,
            coupled_axis_mode=coupled_mode
        )
        
        # Initialize controller
        self.controller = ArctosController(config)
        self._connected = False
        
        # State storage
        self._current_positions = [0.0] * 6
        self._current_velocities = [0.0] * 6
        self._command_positions = [0.0] * 6
        self._command_velocities = [0.0] * 6
        self._last_command_time = time.time()
        
        # Thread safety
        self._lock = threading.RLock()
        
        # QoS settings
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', qos_reliable
        )
        
        # Subscribers for commands
        self.create_subscription(
            Float64MultiArray,
            '/arctos/commands/position',
            self._position_command_callback,
            qos_best_effort
        )
        
        self.create_subscription(
            Float64MultiArray,
            '/arctos/commands/velocity',
            self._velocity_command_callback,
            qos_best_effort
        )
        
        # Connect to hardware
        if not self._connect():
            self.get_logger().error('Failed to connect to hardware')
        else:
            # Enable motors
            self.controller.enable_motors()
        
        # Start update loop
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self._update_callback
        )
        
        self.get_logger().info(
            f'Arctos hardware interface initialized '
            f'(device: {can_device}, rate: {self.update_rate} Hz)'
        )
    
    def _connect(self) -> bool:
        """Connect to hardware."""
        try:
            if self.controller.connect():
                self._connected = True
                self.get_logger().info('Connected to hardware')
                
                # Read initial positions
                self._current_positions = list(self.controller.read_joint_positions())
                self._command_positions = list(self._current_positions)
                
                return True
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
        
        return False
    
    def _position_command_callback(self, msg: Float64MultiArray):
        """Handle position command."""
        if len(msg.data) != 6:
            self.get_logger().warn(f'Invalid position command size: {len(msg.data)}')
            return
        
        with self._lock:
            self._command_positions = list(msg.data)
            self._last_command_time = time.time()
    
    def _velocity_command_callback(self, msg: Float64MultiArray):
        """Handle velocity command."""
        if len(msg.data) != 6:
            self.get_logger().warn(f'Invalid velocity command size: {len(msg.data)}')
            return
        
        with self._lock:
            self._command_velocities = list(msg.data)
            self._last_command_time = time.time()
    
    def _update_callback(self):
        """Main update loop - read state, write commands."""
        if not self._connected:
            return
        
        try:
            with self._lock:
                # Read current state
                self._current_positions, self._current_velocities = \
                    self.controller.get_joint_states()
                
                # Check command timeout
                if time.time() - self._last_command_time > self.command_timeout:
                    # No recent commands - don't send anything
                    pass
                else:
                    # Send commands
                    if self.position_mode:
                        self.controller.move_to_positions(self._command_positions)
                    else:
                        # Velocity mode - convert to position increments
                        dt = 1.0 / self.update_rate
                        target = [
                            pos + vel * dt
                            for pos, vel in zip(self._current_positions, self._command_velocities)
                        ]
                        self.controller.move_to_positions(target)
            
            # Publish joint state
            self._publish_joint_state()
            
        except Exception as e:
            self.get_logger().error(f'Update error: {e}')
    
    def _publish_joint_state(self):
        """Publish current joint state."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = self._current_positions
        msg.velocity = self._current_velocities
        msg.effort = [0.0] * 6
        
        self.joint_state_pub.publish(msg)
    
    def destroy_node(self):
        """Clean shutdown."""
        if self._connected:
            try:
                self.controller.stop_all()
                self.controller.disable_motors()
                self.controller.disconnect()
            except Exception:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ArctosHardwareInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
