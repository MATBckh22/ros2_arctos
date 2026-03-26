#!/usr/bin/env python3
"""
Arctos Hardware ROS2 Node — LEGACY.

This is the legacy standalone node with its own FollowJointTrajectory action
server. It bypasses ros2_control entirely.

For ros2_control + MoveIt integration, use arctos_can_bridge.py instead
(launched via real_hardware.launch.py in arctos_moveit_config).

This node is kept for backward compatibility and direct hardware testing.
"""

import math
import time
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState

from arctos_hardware.arctos_controller import ArctosController, ArctosConfig
from arctos_hardware.can_interface import CanError, default_can_device


def parse_active_motor_ids(value) -> List[int]:
    """Parse a comma-separated motor ID list."""
    if isinstance(value, int):
        return [value]
    if isinstance(value, str):
        return [int(part.strip()) for part in value.split(',') if part.strip()]
    if isinstance(value, (list, tuple)):
        return [int(part) for part in value]
    return [1, 2, 3, 4, 5, 6]


class ArctosHardwareNode(Node):
    """
    ROS2 node for Arctos robot hardware control.
    
    Publishers:
        /joint_states: Current joint positions and velocities
        /arctos/controller_state: Controller state feedback
    
    Services:
        /arctos/home_all: Execute full homing sequence
        /arctos/set_zero: Set current position as zero
        /arctos/enable_motors: Enable all motors
        /arctos/disable_motors: Disable all motors
        /arctos/emergency_stop: Emergency stop
    
    Actions:
        /arctos/follow_joint_trajectory: Execute joint trajectory
    """
    
    JOINT_NAMES = [
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
    ]
    STATE_JOINT_NAMES = JOINT_NAMES + ['jaw1', 'jaw2']
    CONTINUOUS_JOINT_INDICES = set(range(6))
    def __init__(self):
        super().__init__('arctos_hardware_node')
        
        # Declare parameters
        self.declare_parameter('can_device', default_can_device())
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('coupled_axis_mode', True)
        self.declare_parameter('auto_enable', True)
        self.declare_parameter(
            'active_motor_ids',
            '1,2,3,4,5,6',
            ParameterDescriptor(dynamic_typing=True)
        )
        
        # Get parameters
        can_device = self.get_parameter('can_device').value
        can_bitrate = self.get_parameter('can_bitrate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        coupled_mode = self.get_parameter('coupled_axis_mode').value
        auto_enable = self.get_parameter('auto_enable').value
        active_motor_ids = parse_active_motor_ids(self.get_parameter('active_motor_ids').value)
        
        # Create configuration
        config = ArctosConfig(
            can_device=can_device,
            can_bitrate=can_bitrate,
            coupled_axis_mode=coupled_mode,
            active_motor_ids=active_motor_ids,
        )
        
        # Initialize controller
        self.controller = ArctosController(config)
        self._connected = False
        self._enabled = False
        
        # Thread safety
        self._lock = threading.RLock()
        self._trajectory_lock = threading.Lock()
        self._executing_trajectory = False
        
        # Callback groups
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.action_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        
        # QoS for joint states
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', qos
        )
        
        self.controller_state_pub = self.create_publisher(
            JointTrajectoryControllerState, '/arctos/controller_state', qos
        )
        
        # Services
        self.create_service(
            Trigger, '/arctos/home_all', self.home_all_callback,
            callback_group=self.service_cb_group
        )
        
        self.create_service(
            Trigger, '/arctos/set_zero', self.set_zero_callback,
            callback_group=self.service_cb_group
        )
        
        self.create_service(
            Trigger, '/arctos/enable_motors', self.enable_motors_callback,
            callback_group=self.service_cb_group
        )
        
        self.create_service(
            Trigger, '/arctos/disable_motors', self.disable_motors_callback,
            callback_group=self.service_cb_group
        )
        
        self.create_service(
            Trigger, '/arctos/emergency_stop', self.emergency_stop_callback,
            callback_group=self.service_cb_group
        )
        
        self.create_service(
            Trigger, '/arctos/reconnect', self.reconnect_callback,
            callback_group=self.service_cb_group
        )
        
        # Action server for trajectory execution
        self.trajectory_action = ActionServer(
            self,
            FollowJointTrajectory,
            '/arctos_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_trajectory_callback,
            goal_callback=self.trajectory_goal_callback,
            cancel_callback=self.trajectory_cancel_callback,
            callback_group=self.action_cb_group
        )
        
        # Trajectory subscriber (alternative to action)
        self.create_subscription(
            JointTrajectory,
            '/arctos/joint_trajectory',
            self.trajectory_command_callback,
            10
        )
        
        # Connect to hardware
        self._connect()
        
        if auto_enable and self._connected:
            self._enable_motors()
        
        # Start publishing timer
        self.state_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_state_callback,
            callback_group=self.timer_cb_group
        )
        
        self.get_logger().info(f'Arctos hardware node initialized (device: {can_device})')
    
    def _connect(self) -> bool:
        """Connect to robot hardware."""
        try:
            if self.controller.connect():
                self._connected = True
                self.get_logger().info('Connected to Arctos hardware')
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
        
        self._connected = False
        return False
    
    def _enable_motors(self) -> bool:
        """Enable all motors."""
        if not self._connected:
            return False
        
        try:
            if self.controller.enable_motors():
                self._enabled = True
                self.get_logger().info('Motors enabled')
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to enable motors: {e}')
        
        return False
    
    # ===== Publishers =====
    
    def publish_state_callback(self):
        """Publish current joint state."""
        if not self._connected:
            return
        
        try:
            # Reduce CAN traffic while a trajectory is active: positions are
            # still needed for feedback/visualization, but motor-speed polling
            # adds another full bus pass and has been causing reply dropouts.
            if self._executing_trajectory:
                positions = self.controller.read_joint_positions()
                velocities = [0.0] * len(self.JOINT_NAMES)
            else:
                positions, velocities = self.controller.get_joint_states()
            
            # Publish JointState
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.STATE_JOINT_NAMES
            msg.position = positions + [0.0, 0.0]
            msg.velocity = velocities + [0.0, 0.0]
            msg.effort = [0.0] * len(self.STATE_JOINT_NAMES)  # Not available from hardware
            
            self.joint_state_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().warning(f'Error reading joint states: {e}')
    
    # ===== Service Callbacks =====
    
    def home_all_callback(self, request, response):
        """Handle home_all service request."""
        self.get_logger().info('Homing request received')
        
        if not self._connected:
            response.success = False
            response.message = 'Not connected to hardware'
            return response
        
        try:
            with self._lock:
                success = self.controller.home_all()
            
            response.success = success
            response.message = 'Homing complete' if success else 'Homing failed'
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Homing error: {e}')
        
        return response
    
    def set_zero_callback(self, request, response):
        """Handle set_zero service request."""
        self.get_logger().info('Set zero request received')
        
        if not self._connected:
            response.success = False
            response.message = 'Not connected to hardware'
            return response
        
        try:
            with self._lock:
                success = self.controller.set_zero_all()
            
            response.success = success
            response.message = 'Zero position set' if success else 'Set zero failed'
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Set zero error: {e}')
        
        return response
    
    def enable_motors_callback(self, request, response):
        """Handle enable_motors service request."""
        if not self._connected:
            response.success = False
            response.message = 'Not connected to hardware'
            return response
        
        response.success = self._enable_motors()
        response.message = 'Motors enabled' if response.success else 'Failed to enable motors'
        return response
    
    def disable_motors_callback(self, request, response):
        """Handle disable_motors service request."""
        if not self._connected:
            response.success = False
            response.message = 'Not connected to hardware'
            return response
        
        try:
            success = self.controller.disable_motors()
            self._enabled = not success
            response.success = success
            response.message = 'Motors disabled' if success else 'Failed to disable motors'
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response
    
    def emergency_stop_callback(self, request, response):
        """Handle emergency_stop service request."""
        self.get_logger().warn('EMERGENCY STOP requested')
        
        try:
            self.controller.emergency_stop()
            self._executing_trajectory = False
            response.success = True
            response.message = 'Emergency stop executed'
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response
    
    def reconnect_callback(self, request, response):
        """Handle reconnect service request."""
        self.get_logger().info('Reconnect requested')
        
        self.controller.disconnect()
        self._connected = False
        
        if self._connect():
            response.success = True
            response.message = 'Reconnected successfully'
        else:
            response.success = False
            response.message = 'Reconnection failed'
        
        return response
    
    # ===== Trajectory Execution =====
    
    def trajectory_goal_callback(self, goal_request):
        """Accept or reject trajectory goal."""
        if not self._connected:
            self.get_logger().warn('Rejecting trajectory: not connected')
            return GoalResponse.REJECT
        
        if not self._enabled:
            self.get_logger().warn('Rejecting trajectory: motors not enabled')
            return GoalResponse.REJECT
        
        if self._executing_trajectory:
            self.get_logger().warn('Rejecting trajectory: already executing')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def trajectory_cancel_callback(self, goal_handle):
        """Handle trajectory cancellation."""
        self.get_logger().info('Trajectory cancellation requested')
        self._executing_trajectory = False
        self.controller.stop_all()
        return CancelResponse.ACCEPT

    @staticmethod
    def _nearest_equivalent_angle(current: float, target: float) -> float:
        """Return the equivalent target angle nearest to the current angle."""
        delta = math.atan2(math.sin(target - current), math.cos(target - current))
        return current + delta

    def _unwrap_continuous_targets(
        self,
        current_positions: List[float],
        target_positions: List[float],
        joint_indices: List[int],
    ) -> List[float]:
        """Map continuous-joint targets to the nearest equivalent angles."""
        unwrapped = list(target_positions)
        for idx in joint_indices:
            if idx in self.CONTINUOUS_JOINT_INDICES:
                unwrapped[idx] = self._nearest_equivalent_angle(
                    current_positions[idx],
                    target_positions[idx],
                )
        return unwrapped

    def execute_trajectory_callback(self, goal_handle):
        """Execute joint trajectory action."""
        self.get_logger().info('Executing trajectory...')
        
        self._executing_trajectory = True
        trajectory = goal_handle.request.trajectory
        
        feedback = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()
        
        try:
            # Map joint names to indices
            joint_indices = []
            for name in trajectory.joint_names:
                if name in self.JOINT_NAMES:
                    joint_indices.append(self.JOINT_NAMES.index(name))
                else:
                    self.get_logger().error(f'Unknown joint: {name}')
                    result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                    goal_handle.abort()
                    return result
            
            # Track elapsed time against trajectory timestamps
            start_time = time.monotonic()
            previous_target_time = 0.0
            previous_positions = self.controller.read_joint_positions()
            
            # Execute each trajectory point
            for i, point in enumerate(trajectory.points):
                if goal_handle.is_cancel_requested:
                    self.controller.stop_all()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                
                if not self._executing_trajectory:
                    break
                
                # Build full position vector for the segment end and unwrap
                # continuous joints so they move along the nearest equivalent path.
                target_positions = list(previous_positions)
                
                for j, idx in enumerate(joint_indices):
                    target_positions[idx] = point.positions[j]
                target_positions = self._unwrap_continuous_targets(
                    previous_positions,
                    target_positions,
                    joint_indices,
                )

                target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                if not self.controller.move_to_positions(target_positions, validate=False):
                    self.get_logger().warn(
                        'Trajectory point command failed after retry'
                    )
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    goal_handle.abort()
                    return result

                while True:
                    remaining = target_time - (time.monotonic() - start_time)
                    if remaining <= 0.0:
                        break
                    if goal_handle.is_cancel_requested or not self._executing_trajectory:
                        self.controller.stop_all()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        return result
                    time.sleep(min(0.02, remaining))

                previous_positions = target_positions
                previous_target_time = target_time
                
                # Publish feedback
                actual = self.controller.read_joint_positions()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = trajectory.joint_names
                feedback.desired.positions = list(point.positions)
                feedback.actual.positions = [actual[idx] for idx in joint_indices]
                feedback.error.positions = [
                    feedback.desired.positions[j] - feedback.actual.positions[j]
                    for j in range(len(joint_indices))
                ]
                
                goal_handle.publish_feedback(feedback)
            
            # Give hardware a brief settle window after the planned end time
            self.controller.wait_for_idle(timeout=1.0)
            
            # Check final position
            final_positions = self.controller.read_joint_positions()
            final_errors = [
                abs(trajectory.points[-1].positions[j] - final_positions[joint_indices[j]])
                for j in range(len(joint_indices))
            ]
            
            max_error = max(final_errors) if final_errors else 0.0
            
            if max_error > 0.1:  # 0.1 rad tolerance
                self.get_logger().warn(f'Trajectory completed with error: {max_error:.4f} rad')
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            else:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            
            goal_handle.succeed()
            
        except Exception as e:
            self.get_logger().error(f'Trajectory execution error: {e}')
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            goal_handle.abort()
        
        finally:
            self._executing_trajectory = False
        
        return result
    
    def trajectory_command_callback(self, msg: JointTrajectory):
        """Handle direct trajectory command (non-action)."""
        if not self._connected or not self._enabled:
            return
        
        if len(msg.points) == 0:
            return
        
        # Execute last point immediately
        point = msg.points[-1]
        
        try:
            # Map joint names
            positions = self.controller.read_joint_positions()
            
            for i, name in enumerate(msg.joint_names):
                if name in self.JOINT_NAMES:
                    idx = self.JOINT_NAMES.index(name)
                    positions[idx] = point.positions[i]
            
            self.controller.move_to_positions(positions)
            
        except Exception as e:
            self.get_logger().error(f'Trajectory command error: {e}')
    
    # ===== Cleanup =====
    
    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down Arctos hardware node')
        
        # Stop trajectory execution
        self._executing_trajectory = False
        
        # Stop and disable motors
        if self._connected:
            try:
                self.controller.stop_all()
                self.controller.disable_motors()
            except Exception:
                pass
        
        # Disconnect
        self.controller.disconnect()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ArctosHardwareNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
