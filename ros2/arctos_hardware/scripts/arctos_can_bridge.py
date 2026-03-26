#!/usr/bin/env python3
"""
Arctos CAN Bridge — primary hardware interface for ros2_control integration.

This is the authoritative ROS 2 node for controlling the Arctos robot arm via
the CAN bus. It bridges topic_based_ros2_control topics to physical hardware
and provides all operational services (homing, calibration, safety).

Data flow:
  JointTrajectoryController → /arctos/joint_commands → [this node] → CAN bus
  CAN bus → [this node] → /arctos/joint_states → joint_state_broadcaster

Services:
  /arctos/emergency_stop  — Immediate motor stop
  /arctos/enable_motors   — Re-enable motors after e-stop
  /arctos/disable_motors  — Gracefully disable motors
  /arctos/home_all        — Execute full homing sequence (pauses control loop)
  /arctos/set_zero        — Set current position as zero reference
  /arctos/reconnect       — Reconnect to CAN bus after disconnect

Note: arctos_hardware_node.py is the legacy standalone node with its own
FollowJointTrajectory action server. This CAN bridge is the primary path
when using ros2_control + MoveIt integration.

Usage:
    ros2 run arctos_hardware arctos_can_bridge.py
"""

import math
import time
import threading
import logging

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from arctos_hardware.arctos_controller import ArctosController, ArctosConfig
from arctos_hardware.can_interface import default_can_device
from arctos_hardware.homing import ArctosHoming

logger = logging.getLogger(__name__)

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
JAW_NAMES = ['jaw1', 'jaw2']
ALL_JOINT_NAMES = JOINT_NAMES + JAW_NAMES


class ArctosCanBridge(Node):
    """
    Primary CAN bridge node for Arctos robot hardware.

    Publishers:
        /arctos/joint_states  — Encoder-based joint positions (for ros2_control)

    Subscriptions:
        /arctos/joint_commands — Position commands from ros2_control

    Services:
        /arctos/emergency_stop  — Immediate stop, disables motors
        /arctos/enable_motors   — Re-enable motors
        /arctos/disable_motors  — Graceful disable
        /arctos/home_all        — Full homing sequence (blocks control loop)
        /arctos/set_zero        — Set current positions as zero
        /arctos/reconnect       — Reconnect CAN bus
    """

    def __init__(self):
        super().__init__('arctos_can_bridge')

        self.declare_parameter('can_device', default_can_device())
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('coupled_axis_mode', True)
        self.declare_parameter('state_publish_rate', 5.0)
        self.declare_parameter('command_send_rate', 10.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('active_joints', [1, 2, 3, 4])
        self.declare_parameter('state_joint_signs', [1.0, 1.0, -1.0, 1.0, 1.0, 1.0])

        can_device = self.get_parameter('can_device').value
        can_bitrate = self.get_parameter('can_bitrate').value
        coupled_mode = self.get_parameter('coupled_axis_mode').value
        self._state_rate = self.get_parameter('state_publish_rate').value
        self._command_rate = self.get_parameter('command_send_rate').value
        self._cmd_timeout = self.get_parameter('command_timeout').value
        active_joints_param = self.get_parameter('active_joints').value
        state_joint_signs = self.get_parameter('state_joint_signs').value

        self._active_indices = [j - 1 for j in active_joints_param]
        self._state_joint_signs = [float(v) for v in state_joint_signs]

        config = ArctosConfig(
            can_device=can_device,
            can_bitrate=can_bitrate,
            coupled_axis_mode=coupled_mode,
            active_motor_ids=list(active_joints_param),
        )
        self.controller = ArctosController(config)

        self._lock = threading.Lock()
        self._last_cmd_time = 0.0
        self._last_cmd_positions = [0.0] * 6
        self._last_sent_cmd_positions = [0.0] * 6
        self._last_known_positions = [0.0] * 6  # Fallback for failed reads
        self._pending_cmd = None
        self._motors_enabled = False
        self._watchdog_triggered = False
        self._control_paused = False  # Paused during homing/reconnect
        self._warned_inactive_joints = set()

        # Callback groups: services run on a separate thread so they don't
        # block the control loop timer (and vice versa).
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()
        self._service_cb_group = MutuallyExclusiveCallbackGroup()

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._state_pub = self.create_publisher(
            JointState, '/arctos/joint_states', state_qos
        )

        self._cmd_sub = self.create_subscription(
            JointState,
            '/arctos/joint_commands',
            self._on_joint_command,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=1,
            ),
        )

        # --- Services ---
        self.create_service(
            Trigger, '/arctos/emergency_stop', self._handle_estop,
            callback_group=self._service_cb_group,
        )
        self.create_service(
            Trigger, '/arctos/enable_motors', self._handle_enable,
            callback_group=self._service_cb_group,
        )
        self.create_service(
            Trigger, '/arctos/disable_motors', self._handle_disable,
            callback_group=self._service_cb_group,
        )
        self.create_service(
            Trigger, '/arctos/home_all', self._handle_home_all,
            callback_group=self._service_cb_group,
        )
        self.create_service(
            Trigger, '/arctos/set_zero', self._handle_set_zero,
            callback_group=self._service_cb_group,
        )
        self.create_service(
            Trigger, '/arctos/reconnect', self._handle_reconnect,
            callback_group=self._service_cb_group,
        )

        self.get_logger().info(
            f'Arctos CAN Bridge starting on {can_device}, '
            f'state rate={self._state_rate} Hz, '
            f'command rate={self._command_rate} Hz, '
            f'active joints={[i+1 for i in self._active_indices]}'
        )

        if not self.controller.connect():
            self.get_logger().fatal('Failed to connect to CAN bus')
            raise RuntimeError('CAN connection failed')

        self.controller.enable_motors()
        self._motors_enabled = True
        self.get_logger().info('Motors enabled')

        positions = self.controller.read_joint_positions()
        self._last_cmd_positions = list(positions)
        self._last_sent_cmd_positions = list(positions)
        self._last_known_positions = list(positions)
        self._last_cmd_time = time.monotonic()
        self.get_logger().info(
            f'Initial positions (deg): '
            f'{[f"{math.degrees(p):.1f}" for p in positions]}'
        )

        self._state_timer = self.create_timer(
            1.0 / self._state_rate, self._state_loop,
            callback_group=self._timer_cb_group,
        )
        self._command_timer = self.create_timer(
            1.0 / self._command_rate, self._command_loop,
            callback_group=self._timer_cb_group,
        )
        self._watchdog_timer = self.create_timer(
            0.1, self._check_watchdog,
            callback_group=self._timer_cb_group,
        )

    def _on_joint_command(self, msg: JointState):
        """Store latest command without blocking (no CAN I/O here)."""
        if not self._motors_enabled:
            return

        name_to_pos = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                name_to_pos[name] = msg.position[i]

        with self._lock:
            target = list(self._last_cmd_positions)

            changed = False
            inactive_requested = []
            for idx in self._active_indices:
                jname = JOINT_NAMES[idx]
                if jname in name_to_pos:
                    new_pos = name_to_pos[jname]
                    if abs(new_pos - target[idx]) > 1e-6:
                        target[idx] = new_pos
                        changed = True

            for i, jname in enumerate(JOINT_NAMES):
                if i not in self._active_indices and jname in name_to_pos:
                    inactive_requested.append(jname)

            for jname in inactive_requested:
                if jname not in self._warned_inactive_joints:
                    self.get_logger().warn(
                        f'Ignoring command for inactive joint {jname}'
                    )
                    self._warned_inactive_joints.add(jname)

            if changed:
                if self.controller.validate_positions(target):
                    self._pending_cmd = target
                else:
                    self.get_logger().warn('Position command rejected: out of limits')

            self._last_cmd_time = time.monotonic()
            if self._watchdog_triggered:
                self.get_logger().info('Command received, re-enabling motors')
                self.controller.enable_motors()
                self._motors_enabled = True
                self._watchdog_triggered = False

    def _read_positions_sequential(self):
        """Read joint positions sequentially to avoid CAN bus collisions.

        Only reads active joints; inactive positions stay at 0.0.
        On read failure, keeps the last successfully read value so the
        RViz visualization doesn't jump.
        """
        positions = list(self._last_known_positions)
        for i in self._active_indices:
            servo = self.controller.servos[i]
            if servo is None:
                continue
            try:
                enc = servo.read_encoder_value()
                if enc is not None:
                    positions[i] = self.controller.encoder_to_angle(enc, i)
            except Exception:
                pass  # keep last known value
            time.sleep(0.005)
        self._last_known_positions = positions
        return positions

    def _positions_for_ros_state(self, positions):
        """Apply display/state-frame sign corrections without affecting commands."""
        corrected = list(positions)
        for i, sign in enumerate(self._state_joint_signs):
            if i < len(corrected):
                corrected[i] *= sign
        return corrected

    def _state_loop(self):
        """Read and publish state at a conservative CAN-safe rate."""
        if self._control_paused:
            return

        try:
            self.controller.can.flush()
        except Exception:
            pass
        time.sleep(0.005)

        try:
            positions = self._read_positions_sequential()
            ros_positions = self._positions_for_ros_state(positions)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(ALL_JOINT_NAMES)
            msg.position = list(ros_positions) + [0.0, 0.0]
            msg.velocity = [0.0] * len(ALL_JOINT_NAMES)
            msg.effort = []

            self._state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to read joint positions: {e}')

    def _command_loop(self):
        """Send the latest trajectory command at a smoother rate than state reads."""
        if self._control_paused or not self._motors_enabled:
            return

        with self._lock:
            cmd = self._pending_cmd
            self._pending_cmd = None

        if cmd is None:
            return

        if all(abs(a - b) < 1e-6 for a, b in zip(cmd, self._last_sent_cmd_positions)):
            return

        time.sleep(0.005)

        try:
            self.controller.move_to_positions_no_wait(cmd, validate=False)
            with self._lock:
                self._last_cmd_positions = cmd
                self._last_sent_cmd_positions = list(cmd)
        except Exception as e:
            self.get_logger().warn(f'Failed to send joint command: {e}')

    def _check_watchdog(self):
        """Disable motors if no command received within timeout."""
        if not self._motors_enabled or self._watchdog_triggered:
            return

        elapsed = time.monotonic() - self._last_cmd_time
        if elapsed > self._cmd_timeout:
            self.get_logger().warn(
                f'No command for {elapsed:.2f}s, watchdog holding position'
            )
            self.controller.stop_all()
            self._watchdog_triggered = True

    def _handle_estop(self, request, response):
        """Emergency stop service callback."""
        self.get_logger().warn('EMERGENCY STOP requested')
        self.controller.emergency_stop()
        self._motors_enabled = False
        self._watchdog_triggered = True
        response.success = True
        response.message = 'Emergency stop executed'
        return response

    def _handle_enable(self, request, response):
        """Re-enable motors after e-stop."""
        self.get_logger().info('Motor enable requested')
        self.controller.enable_motors()
        self._motors_enabled = True
        self._watchdog_triggered = False
        self._last_cmd_time = time.monotonic()
        response.success = True
        response.message = 'Motors enabled'
        return response

    def _handle_disable(self, request, response):
        """Gracefully disable motors."""
        self.get_logger().info('Motor disable requested')
        try:
            self.controller.stop_all()
            self.controller.disable_motors()
            self._motors_enabled = False
            response.success = True
            response.message = 'Motors disabled'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_home_all(self, request, response):
        """Execute homing sequence for active joints only.

        When active_joints=[1,2,3,4] (the default), this homes only the
        standard joints and skips the differential wrist (J5-6).  Full
        home_all() is only called when all six joints are active.
        """
        self.get_logger().info('Homing request received — pausing control loop')

        if not self.controller.is_connected:
            response.success = False
            response.message = 'Not connected to hardware'
            return response

        # Pause the control loop so homing has exclusive CAN bus access.
        self._control_paused = True
        self._motors_enabled = False

        try:
            homing = ArctosHoming(self.controller.can)

            wrist_active = 4 in self._active_indices and 5 in self._active_indices
            if wrist_active:
                success = homing.home_all()
            else:
                results = homing.home_standard_joints()
                success = all(r.success for r in results)
                if success:
                    homing.verify_zero_positions()

            if success:
                # Re-read positions after homing so state is accurate.
                positions = self.controller.read_joint_positions()
                with self._lock:
                    self._last_cmd_positions = list(positions)
                    self._pending_cmd = None
                self.get_logger().info(
                    f'Post-home positions (deg): '
                    f'{[f"{math.degrees(p):.1f}" for p in positions]}'
                )

            response.success = success
            response.message = 'Homing complete' if success else 'Homing failed'
        except Exception as e:
            self.get_logger().error(f'Homing error: {e}')
            response.success = False
            response.message = str(e)
        finally:
            # Re-enable control loop regardless of outcome.
            self.controller.enable_motors()
            self._motors_enabled = True
            self._watchdog_triggered = False
            self._last_cmd_time = time.monotonic()
            self._control_paused = False
            self.get_logger().info('Control loop resumed')

        return response

    def _handle_set_zero(self, request, response):
        """Set current motor positions as the zero reference."""
        self.get_logger().info('Set zero requested')

        if not self.controller.is_connected:
            response.success = False
            response.message = 'Not connected to hardware'
            return response

        # Briefly pause control loop to avoid CAN contention during zeroing.
        self._control_paused = True
        try:
            success = self.controller.set_zero_all()
            if success:
                with self._lock:
                    self._last_cmd_positions = [0.0] * 6
                    self._pending_cmd = None
            response.success = success
            response.message = 'Zero position set' if success else 'Set zero failed'
        except Exception as e:
            self.get_logger().error(f'Set zero error: {e}')
            response.success = False
            response.message = str(e)
        finally:
            self._control_paused = False

        return response

    def _handle_reconnect(self, request, response):
        """Reconnect to CAN bus after a disconnect."""
        self.get_logger().info('Reconnect requested')

        self._control_paused = True
        self._motors_enabled = False

        try:
            self.controller.disconnect()

            if self.controller.connect():
                self.controller.enable_motors()
                self._motors_enabled = True
                self._last_cmd_time = time.monotonic()
                positions = self.controller.read_joint_positions()
                with self._lock:
                    self._last_cmd_positions = list(positions)
                    self._pending_cmd = None
                response.success = True
                response.message = 'Reconnected successfully'
            else:
                response.success = False
                response.message = 'Reconnection failed'
        except Exception as e:
            self.get_logger().error(f'Reconnect error: {e}')
            response.success = False
            response.message = str(e)
        finally:
            self._control_paused = False
            self._watchdog_triggered = False

        return response

    def destroy_node(self):
        self.get_logger().info('Shutting down CAN bridge')
        try:
            self.controller.stop_all()
            self.controller.disconnect()
        except Exception as e:
            self.get_logger().warn(f'Error during shutdown: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArctosCanBridge()

    executor = MultiThreadedExecutor(num_threads=3)
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
