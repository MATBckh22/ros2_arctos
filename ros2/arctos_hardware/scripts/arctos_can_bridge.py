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
from arctos_hardware.can_interface import default_can_device, query_socketcan_state
from arctos_hardware.homing import ArctosHoming, ArctosHomingConfig
from arctos_hardware.mks_servo import EndStopLevel

logger = logging.getLogger(__name__)

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
ALL_JOINT_NAMES = JOINT_NAMES


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
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('active_joints', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('state_joint_signs', [1.0, 1.0, -1.0, 1.0, -1.0, 1.0])
        self.declare_parameter('command_joint_signs', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('enforce_limit_switch_stop', True)
        self.declare_parameter('limit_poll_rate', 2.0)
        self.declare_parameter('validate_position_commands', False)
        self.declare_parameter('enforce_stall_stop', True)
        self.declare_parameter('stall_error_deg', 4.0)
        self.declare_parameter('stall_progress_deg', 0.25)
        self.declare_parameter('stall_timeout_s', 0.8)
        self.declare_parameter('bus_diag_rate', 1.0)

        can_device = self.get_parameter('can_device').value
        can_bitrate = self.get_parameter('can_bitrate').value
        coupled_mode = self.get_parameter('coupled_axis_mode').value
        self._state_rate = self.get_parameter('state_publish_rate').value
        self._cmd_timeout = self.get_parameter('command_timeout').value
        active_joints_param = self.get_parameter('active_joints').value
        state_joint_signs = self.get_parameter('state_joint_signs').value
        command_joint_signs = self.get_parameter('command_joint_signs').value
        self._enforce_limit_switch_stop = self.get_parameter('enforce_limit_switch_stop').value
        self._limit_poll_rate = float(self.get_parameter('limit_poll_rate').value)
        self._validate_position_commands = self.get_parameter('validate_position_commands').value
        self._enforce_stall_stop = self.get_parameter('enforce_stall_stop').value
        self._stall_error_rad = math.radians(self.get_parameter('stall_error_deg').value)
        self._stall_progress_rad = math.radians(self.get_parameter('stall_progress_deg').value)
        self._stall_timeout_s = self.get_parameter('stall_timeout_s').value
        self._bus_diag_rate = float(self.get_parameter('bus_diag_rate').value)
        self._can_device = can_device

        self._active_indices = [j - 1 for j in active_joints_param]
        self._state_joint_signs = [float(v) for v in state_joint_signs]
        self._command_joint_signs = [float(v) for v in command_joint_signs]
        self._homing_config = ArctosHomingConfig()
        self._limit_motor_ids = list(active_joints_param)

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
        self._motors_enabled = False
        self._watchdog_triggered = False
        self._limit_tripped = False
        self._stall_tripped = False
        self._control_paused = False  # Paused during homing/reconnect
        self._warned_inactive_joints = set()
        self._last_limit_states = {motor_id: None for motor_id in self._limit_motor_ids}
        self._last_limit_warn_time = 0.0
        self._last_command_log_time = 0.0
        self._last_rx_log_time = 0.0
        self._rx_count_since_log = 0
        self._last_rx_summary_time = 0.0
        self._last_invalid_cmd_warn_time = 0.0
        self._last_state_positions = None
        self._stall_started_at = None
        self._last_bus_state_str = None
        self._last_bus_restarts = 0
        self._last_bus_errors = 0
        # Track consecutive read failures so we can back off on a motor that
        # keeps timing out instead of blocking the bus every cycle.
        self._read_failures = [0] * 6
        self._read_skip_counter = [0] * 6
        # Command worker: subscription drops the latest target into this slot
        # and signals the event; the worker thread fires CAN writes without
        # blocking the ROS executor.
        self._cmd_cv = threading.Condition()
        self._pending_target = None
        self._worker_should_stop = False

        # Keep read/write timers on separate executor lanes. Actual CAN access
        # is serialized with the controller lock so command streaming doesn't
        # get starved behind the slower state-read loop.
        self._state_cb_group = MutuallyExclusiveCallbackGroup()
        self._watchdog_cb_group = MutuallyExclusiveCallbackGroup()
        self._service_cb_group = MutuallyExclusiveCallbackGroup()
        self._limit_cb_group = MutuallyExclusiveCallbackGroup()
        self._diag_cb_group = MutuallyExclusiveCallbackGroup()

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
            f'commands forwarded directly from /arctos/joint_commands, '
            f'active joints={[i+1 for i in self._active_indices]}, '
            f'state_signs={self._state_joint_signs}, '
            f'command_signs={self._command_joint_signs}'
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
            callback_group=self._state_cb_group,
        )
        self._watchdog_timer = self.create_timer(
            0.1, self._check_watchdog,
            callback_group=self._watchdog_cb_group,
        )

        if self._enforce_limit_switch_stop and self._limit_poll_rate > 0:
            self._limit_timer = self.create_timer(
                1.0 / self._limit_poll_rate, self._limit_loop,
                callback_group=self._limit_cb_group,
            )

        if self._bus_diag_rate > 0:
            self._bus_diag_timer = self.create_timer(
                1.0 / self._bus_diag_rate, self._bus_diag_loop,
                callback_group=self._diag_cb_group,
            )

        self._command_thread = threading.Thread(
            target=self._command_worker,
            name='arctos_command_worker',
            daemon=True,
        )
        self._command_thread.start()

    def _on_joint_command(self, msg: JointState):
        """Handoff incoming targets to the CAN worker without blocking.

        The subscription runs on the ROS executor; doing CAN I/O inline
        would make it share a lock with the state-read loop and stall for
        hundreds of milliseconds on every motor read timeout. Instead we
        parse the target, stash it in a single-slot buffer, and wake the
        worker thread that owns the CAN bus.
        """
        self._rx_count_since_log += 1
        now = time.monotonic()
        if now - self._last_rx_log_time > 2.0:
            self.get_logger().info(
                f'rx: {self._rx_count_since_log} cmd(s) in last '
                f'{now - self._last_rx_log_time:.1f}s '
                f'[enabled={self._motors_enabled} wd={self._watchdog_triggered} '
                f'limit={self._limit_tripped} stall={self._stall_tripped} '
                f'paused={self._control_paused}]'
            )
            self._last_rx_log_time = now
            self._rx_count_since_log = 0

        if self._control_paused:
            return

        limit_latched = self._enforce_limit_switch_stop and self._limit_tripped
        stall_latched = self._enforce_stall_stop and self._stall_tripped
        if limit_latched or stall_latched:
            if now - self._last_limit_warn_time > 1.0:
                self.get_logger().warn(
                    'Ignoring command while a safety stop is latched; '
                    'inspect the robot and call /arctos/enable_motors'
                )
                self._last_limit_warn_time = now
            return

        name_to_pos = {}
        invalid_names = []
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                pos = msg.position[i]
                if math.isfinite(pos):
                    name_to_pos[name] = pos
                else:
                    invalid_names.append(name)

        if invalid_names and now - self._last_invalid_cmd_warn_time > 1.0:
            self.get_logger().warn(
                'Ignoring non-finite command position(s) for '
                + ', '.join(sorted(set(invalid_names)))
            )
            self._last_invalid_cmd_warn_time = now

        inactive_requested = []
        with self._lock:
            previous_target = list(self._last_cmd_positions)
            target = list(self._last_cmd_positions)
            updated_active = False
            for idx in self._active_indices:
                jname = JOINT_NAMES[idx]
                if jname in name_to_pos:
                    # ros2_control and RViz operate in the same joint frame as
                    # the published /arctos/joint_states topic. Convert that
                    # back into the controller's internal hardware frame here.
                    target[idx] = name_to_pos[jname] * self._command_joint_signs[idx]
                    updated_active = True
            for i, jname in enumerate(JOINT_NAMES):
                if i not in self._active_indices and jname in name_to_pos:
                    inactive_requested.append(jname)

            if not updated_active:
                return

            self._last_cmd_time = now
            self._last_cmd_positions = list(target)

        for jname in inactive_requested:
            if jname not in self._warned_inactive_joints:
                self.get_logger().warn(f'Ignoring command for inactive joint {jname}')
                self._warned_inactive_joints.add(jname)

        if self._validate_position_commands and not self.controller.validate_positions(target):
            self.get_logger().warn('Position command rejected: out of limits')
            return

        changed = [
            f'{JOINT_NAMES[i]}={math.degrees(target[i]):+6.2f}°'
            for i in self._active_indices
            if abs(target[i] - previous_target[i]) > 1e-5
        ]
        if changed and now - self._last_rx_summary_time > 0.25:
            self.get_logger().info('rx target: ' + ' '.join(changed))
            self._last_rx_summary_time = now

        with self._cmd_cv:
            self._pending_target = target
            self._cmd_cv.notify()

    def _command_worker(self):
        """Drain the latest target into fire-and-forget CAN writes.

        Using a dedicated thread keeps CAN I/O completely off the ROS
        executor: the subscription stays responsive, and a slow state read
        on one motor can no longer throttle command forwarding.
        """
        while True:
            with self._cmd_cv:
                while (
                    self._pending_target is None
                    and not self._worker_should_stop
                ):
                    self._cmd_cv.wait()
                if self._worker_should_stop:
                    return
                target = self._pending_target
                self._pending_target = None

            if self._control_paused:
                continue

            if not self._motors_enabled:
                if self._watchdog_triggered:
                    try:
                        self.controller.enable_motors()
                    except Exception as e:
                        self.get_logger().warn(f'Auto-reenable failed: {e}')
                        continue
                    self._motors_enabled = True
                    self._watchdog_triggered = False
                    self.get_logger().info('Command received, re-enabled motors')
                else:
                    continue

            if all(abs(a - b) < 1e-5 for a, b in zip(target, self._last_sent_cmd_positions)):
                continue

            if not all(math.isfinite(v) for v in target):
                self.get_logger().warn('Skipping non-finite target before CAN send')
                continue

            try:
                self.controller.move_to_positions_no_wait(target, validate=False)
            except Exception as e:
                self.get_logger().warn(f'Failed to send joint command: {e}')
                continue

            with self._lock:
                self._last_sent_cmd_positions = list(target)

            now = time.monotonic()
            if now - self._last_command_log_time > 1.0:
                enc5 = self.controller.angle_to_encoder(target[4], 4)
                enc6 = self.controller.angle_to_encoder(target[5], 5)
                self.get_logger().info(
                    'cmd: '
                    + ' '.join(
                        f'{JOINT_NAMES[i]}={math.degrees(target[i]):+6.2f}°'
                        for i in range(6)
                    )
                    + f' [WRIST enc5={enc5} enc6={enc6}]'
                )
                self._last_command_log_time = now

    def _read_positions_sequential(self):
        """Read joint positions sequentially to avoid CAN bus collisions.

        Only reads active joints; inactive positions stay at 0.0.
        A motor that keeps timing out is skipped on the next few sweeps so
        one flaky joint can't monopolize the bus and starve command writes.
        """
        positions = list(self._last_known_positions)
        encoder_values = [None] * len(JOINT_NAMES)

        for i in self._active_indices:
            servo = self.controller.servos[i]
            if servo is None:
                continue

            if self._read_skip_counter[i] > 0:
                self._read_skip_counter[i] -= 1
                continue

            try:
                enc = servo.read_encoder_value()
            except Exception:
                enc = None

            if enc is not None:
                encoder_values[i] = enc
                self._read_failures[i] = 0
            else:
                self._read_failures[i] += 1
                # 3 consecutive misses → skip this motor for 5 sweeps so we
                # don't burn a full timeout on it every state cycle.
                if self._read_failures[i] >= 3:
                    self._read_skip_counter[i] = 5

        self._last_encoder_reads = list(encoder_values)
        self._last_read_had_fresh_data = any(v is not None for v in encoder_values)

        for i in range(4):
            if encoder_values[i] is not None:
                positions[i] = self.controller.encoder_to_angle(encoder_values[i], i)

        wrist_coupled = (
            self.controller.config.coupled_axis_mode and
            4 in self._active_indices and
            5 in self._active_indices
        )
        if wrist_coupled:
            wrist_b = encoder_values[4]
            wrist_c = encoder_values[5]
            if wrist_b is not None and wrist_c is not None:
                positions[4], positions[5] = self.controller.wrist.motors_to_joints(
                    wrist_b, wrist_c
                )
        else:
            for i in (4, 5):
                if encoder_values[i] is not None:
                    positions[i] = self.controller.encoder_to_angle(encoder_values[i], i)

        self._last_known_positions = positions
        return positions

    def _positions_for_ros_state(self, positions):
        """Apply display/state-frame sign corrections without affecting commands."""
        corrected = list(positions)
        for i, sign in enumerate(self._state_joint_signs):
            if i < len(corrected):
                corrected[i] *= sign
        return corrected

    def _get_limit_config(self, motor_id):
        """Return homing/limit config for a motor ID."""
        if motor_id in self._homing_config.joint_configs:
            return self._homing_config.joint_configs[motor_id]
        if motor_id == self._homing_config.wrist_b_config.motor_id:
            return self._homing_config.wrist_b_config
        if motor_id == self._homing_config.wrist_c_config.motor_id:
            return self._homing_config.wrist_c_config
        return None

    def _limit_active_from_io(self, motor_id, io_status):
        """Decode the configured limit input/polarity from an IO status byte."""
        cfg = self._get_limit_config(motor_id)
        if cfg is None or io_status is None:
            return None

        if cfg.limit_switch_input not in (1, 2):
            return None

        input_mask = 1 << (cfg.limit_switch_input - 1)
        input_active = (io_status & input_mask) != 0
        return (not input_active) if cfg.trigger_level == EndStopLevel.LOW else input_active

    def _read_limit_states_sequential(self):
        """Read all configured limit switches sequentially to avoid CAN collisions."""
        states = dict(self._last_limit_states)
        for motor_id in self._limit_motor_ids:
            servo = self.controller.servos[motor_id - 1]
            if servo is None:
                continue
            try:
                io_status = servo.read_io_status()
                states[motor_id] = self._limit_active_from_io(motor_id, io_status)
            except Exception:
                states[motor_id] = None
        self._last_limit_states = states
        return states

    def _handle_limit_trip(self, limit_states):
        """Stop motion and latch the bridge if any configured limit is active."""
        active_limits = [motor_id for motor_id, active in limit_states.items() if active is True]
        if not active_limits:
            return
        if self._limit_tripped:
            return

        self.get_logger().error(
            f'Limit switch triggered on motor(s) {active_limits}; stopping motion'
        )
        self.controller.stop_all()
        with self._lock:
            self._last_sent_cmd_positions = list(self._last_cmd_positions)
        self._motors_enabled = False
        self._limit_tripped = True
        self._stall_started_at = None

    def _handle_motion_stall(self, positions):
        """Latch the bridge if commands continue but the robot stops making progress."""
        if not self._enforce_stall_stop or self._stall_tripped or self._limit_tripped:
            self._last_state_positions = list(positions)
            return

        # If the last state read returned no fresh encoder data (e.g. the bus
        # is in bus-off and the kernel is auto-recovering), we cannot tell
        # whether the robot is moving or not — do not latch a false stall.
        if not getattr(self, '_last_read_had_fresh_data', True):
            self._stall_started_at = None
            return

        with self._lock:
            commanded = list(self._last_cmd_positions)
            last_cmd_time = self._last_cmd_time

        if not all(math.isfinite(v) for v in commanded) or not all(math.isfinite(v) for v in positions):
            self._stall_started_at = None
            self._last_state_positions = list(positions)
            return

        now = time.monotonic()
        if now - last_cmd_time > self._cmd_timeout:
            self._stall_started_at = None
            self._last_state_positions = list(positions)
            return

        max_error = max(
            abs(commanded[i] - positions[i]) for i in self._active_indices
        )
        if max_error < self._stall_error_rad:
            self._stall_started_at = None
            self._last_state_positions = list(positions)
            return

        if self._last_state_positions is None:
            self._last_state_positions = list(positions)
            return

        max_progress = max(
            abs(positions[i] - self._last_state_positions[i]) for i in self._active_indices
        )
        self._last_state_positions = list(positions)

        if max_progress > self._stall_progress_rad:
            self._stall_started_at = None
            return

        if self._stall_started_at is None:
            self._stall_started_at = now
            return

        if now - self._stall_started_at < self._stall_timeout_s:
            return

        max_error_deg = math.degrees(max_error)
        self.get_logger().error(
            f'Motion stall detected (max error {max_error_deg:.2f} deg); stopping motion'
        )
        self.controller.stop_all()
        with self._lock:
            self._last_sent_cmd_positions = list(self._last_cmd_positions)
        self._motors_enabled = False
        self._stall_tripped = True
        self._stall_started_at = None

    def _state_loop(self):
        """Read and publish state at a conservative CAN-safe rate."""
        if self._control_paused:
            return

        try:
            positions = self._read_positions_sequential()
            self._handle_motion_stall(positions)
            ros_positions = self._positions_for_ros_state(positions)

            if not hasattr(self, '_wrist_log_counter'):
                self._wrist_log_counter = 0
            self._wrist_log_counter += 1
            if self._wrist_log_counter % 25 == 1:
                raw_b = self._last_encoder_reads[4] if hasattr(self, '_last_encoder_reads') else '?'
                raw_c = self._last_encoder_reads[5] if hasattr(self, '_last_encoder_reads') else '?'
                self.get_logger().info(
                    f'[WRIST_STATE] raw B={raw_b} C={raw_c} | '
                    f'internal j5={math.degrees(positions[4]):.2f}° '
                    f'j6={math.degrees(positions[5]):.2f}° | '
                    f'ros j5={math.degrees(ros_positions[4]):.2f}° '
                    f'j6={math.degrees(ros_positions[5]):.2f}°'
                )

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(ALL_JOINT_NAMES)
            msg.position = list(ros_positions)
            msg.velocity = [0.0] * len(ALL_JOINT_NAMES)
            msg.effort = []

            self._state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to read joint positions: {e}')

    def _limit_loop(self):
        """Poll limit switches at a lower rate than encoders to cut bus load."""
        if self._control_paused or not self._enforce_limit_switch_stop:
            return
        if self._limit_tripped or self._stall_tripped:
            return
        try:
            limit_states = self._read_limit_states_sequential()
            self._handle_limit_trip(limit_states)
        except Exception as e:
            self.get_logger().warn(f'Limit poll failed: {e}')

    def _bus_diag_loop(self):
        """Log kernel-side CAN state changes and climbing error counters."""
        state = query_socketcan_state(self._can_device)
        if state is None:
            return

        state_changed = state.state != self._last_bus_state_str
        errors_changed = (
            state.restarts != self._last_bus_restarts
            or state.bus_errors != self._last_bus_errors
        )

        if state_changed or errors_changed:
            level = self.get_logger().error if state.is_bus_off else (
                self.get_logger().warn if state.is_degraded else self.get_logger().info
            )
            level(
                f'[BUS] state={state.state} restarts={state.restarts} '
                f'bus_errors={state.bus_errors} tx_err={state.tx_errors} '
                f'rx_err={state.rx_errors}'
            )
            self._last_bus_state_str = state.state
            self._last_bus_restarts = state.restarts
            self._last_bus_errors = state.bus_errors

    def _check_watchdog(self):
        """Disable motors if no command received within timeout."""
        if not self._motors_enabled or self._watchdog_triggered or self._limit_tripped or self._stall_tripped:
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
        if self._enforce_limit_switch_stop:
            limit_states = self._read_limit_states_sequential()
            active_limits = [motor_id for motor_id, active in limit_states.items() if active is True]
            if active_limits:
                response.success = False
                response.message = f'Limit switch still active on motor(s) {active_limits}'
                return response
        self.controller.enable_motors()
        self._motors_enabled = True
        self._watchdog_triggered = False
        self._limit_tripped = False
        self._stall_tripped = False
        self._stall_started_at = None
        self._last_cmd_time = time.monotonic()
        try:
            positions = self.controller.read_joint_positions()
            with self._lock:
                self._last_cmd_positions = list(positions)
                self._last_sent_cmd_positions = list(positions)
        except Exception as e:
            self.get_logger().warn(f'Failed to refresh positions on enable: {e}')
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

        When all six joints are active, this executes full standard+wrist
        homing. If a reduced joint set is configured, the wrist homing is
        skipped automatically.
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
                    self._last_sent_cmd_positions = list(positions)
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
            self._limit_tripped = False
            self._stall_tripped = False
            self._stall_started_at = None
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
                    self._last_sent_cmd_positions = [0.0] * 6
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
                self._limit_tripped = False
                self._stall_tripped = False
                self._stall_started_at = None
                self._last_cmd_time = time.monotonic()
                positions = self.controller.read_joint_positions()
                with self._lock:
                    self._last_cmd_positions = list(positions)
                    self._last_sent_cmd_positions = list(positions)
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
        with self._cmd_cv:
            self._worker_should_stop = True
            self._cmd_cv.notify_all()
        try:
            if self._command_thread.is_alive():
                self._command_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.controller.stop_all()
            self.controller.disconnect()
        except Exception as e:
            self.get_logger().warn(f'Error during shutdown: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArctosCanBridge()

    executor = MultiThreadedExecutor(num_threads=5)
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
