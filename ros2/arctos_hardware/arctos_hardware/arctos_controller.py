"""
Arctos Robot Controller.

High-level controller that integrates CAN communication, motor control,
differential wrist kinematics, and coordinate transformation.

This is the main interface for controlling the Arctos robot from ROS2.
"""

import math
import time
import logging
import threading
from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass, field

from .can_interface import CanInterface, CanError, default_can_device
from .mks_servo import MksServo, MotorStatus
from .differential_wrist import DifferentialWrist, WristConfig
from .homing import ArctosHoming, ArctosHomingConfig

logger = logging.getLogger(__name__)


@dataclass
class JointConfig:
    """Configuration for a single joint."""
    motor_id: int
    gear_ratio: float
    direction: int = 1  # 1 or -1
    offset: float = 0.0  # Homing offset in radians
    min_limit: float = -math.pi  # Radians
    max_limit: float = math.pi   # Radians
    max_velocity: float = 2.0    # rad/s
    max_acceleration: float = 5.0  # rad/s^2


@dataclass
class ArctosConfig:
    """Complete configuration for Arctos robot."""
    can_device: str = field(default_factory=default_can_device)
    can_bitrate: int = 500000
    active_motor_ids: List[int] = field(default_factory=lambda: [1, 2, 3, 4, 5, 6])
    
    encoder_resolution: int = 16384
    
    # Joint configurations (index 0-5 for joints 1-6)
    joints: List[JointConfig] = field(default_factory=lambda: [
        JointConfig(motor_id=1, gear_ratio=13.5, direction=1,
                   min_limit=-math.pi, max_limit=math.pi),
        JointConfig(motor_id=2, gear_ratio=150.0, direction=-1,
                   min_limit=-math.pi/2, max_limit=math.pi/2),
        JointConfig(motor_id=3, gear_ratio=150.0, direction=1,
                   min_limit=-math.pi*3/4, max_limit=math.pi*3/4),
        JointConfig(motor_id=4, gear_ratio=48.0, direction=1,
                   min_limit=-math.pi, max_limit=math.pi),
        JointConfig(motor_id=5, gear_ratio=67.82, direction=1,
                   min_limit=-math.pi/2, max_limit=math.pi/2),
        JointConfig(motor_id=6, gear_ratio=67.82, direction=1,
                   min_limit=-math.pi, max_limit=math.pi),
    ])
    
    # Default speeds
    default_speed_rpm: int = 500
    default_acceleration: int = 150
    default_joint_speeds_rpm: List[int] = field(
        default_factory=lambda: [120, 500, 500, 500, 80, 80]
    )
    default_joint_accelerations: List[int] = field(
        default_factory=lambda: [40, 150, 150, 150, 30, 30]
    )
    command_spacing_s: float = 0.02
    command_spacing_no_ack_s: float = 0.002
    command_retry_delay_s: float = 0.05
    read_spacing_s: float = 0.005
    
    # Enable coupled axis mode for differential wrist
    coupled_axis_mode: bool = True
    
    # Wrist configuration
    wrist_joint_ratio: float = 210000.0


class ArctosController:
    """
    Main controller for Arctos robot arm.
    
    Provides:
    - Joint angle reading and commanding
    - Automatic differential wrist transformation
    - Thread-safe parallel motor control
    - Integration with homing system
    
    Attributes:
        config: ArctosConfig instance
        can: CanInterface instance
        servos: List of MksServo instances
        wrist: DifferentialWrist instance
        homing: ArctosHoming instance
    """
    
    NUM_JOINTS = 6
    
    def __init__(self, config: ArctosConfig = None):
        """
        Initialize Arctos controller.
        
        Args:
            config: Robot configuration
        """
        self.config = config or ArctosConfig()
        self._lock = threading.RLock()
        self._connected = False
        
        # CAN interface
        self.can: Optional[CanInterface] = None
        
        # Servo instances
        self.servos: List[Optional[MksServo]] = []
        
        # Differential wrist handler
        wrist_config = WristConfig(
            gear_ratio_j5=self.config.joints[4].gear_ratio,
            gear_ratio_j6=self.config.joints[5].gear_ratio,
            encoder_resolution=self.config.encoder_resolution,
            joint5_direction=self.config.joints[4].direction,
            joint6_direction=self.config.joints[5].direction,
            joint_ratio=self.config.wrist_joint_ratio
        )
        self.wrist = DifferentialWrist(wrist_config)
        
        # Homing system
        self.homing: Optional[ArctosHoming] = None
        
        # Current commanded positions (for tracking)
        self._commanded_positions: List[float] = [0.0] * self.NUM_JOINTS
    
    def connect(self) -> bool:
        """
        Connect to robot hardware.
        
        Returns:
            True if connection successful
        """
        with self._lock:
            if self._connected:
                return True
            
            try:
                # Initialize CAN
                self.can = CanInterface(
                    device=self.config.can_device,
                    bitrate=self.config.can_bitrate
                )
                self.can.connect()
                
                # Create servo instances
                self.servos = []
                active_motor_ids = set(self.config.active_motor_ids)
                for joint in self.config.joints:
                    if joint.motor_id in active_motor_ids:
                        servo = MksServo(self.can, joint.motor_id)
                    else:
                        servo = None
                    self.servos.append(servo)

                # Match ArctosGuiPython startup behavior: enable limit port
                # remap on servos 3-6 so the alternate limit input is available
                # during serial/CAN-controlled homing.
                for joint_index, servo in enumerate(self.servos, start=1):
                    if servo is None or joint_index < 3:
                        continue
                    try:
                        servo.set_limit_port_remap(True)
                        time.sleep(0.1)
                    except Exception as e:
                        logger.warning(
                            f"Failed to enable limit port remap on motor {joint_index}: {e}"
                        )

                # Let the MKS drivers smooth axis-mode moves themselves rather
                # than emulating dense interpolation in Python. The reference
                # stack uses interpolation with 64 subdivisions.
                for joint_index, servo in enumerate(self.servos, start=1):
                    if servo is None:
                        continue
                    try:
                        servo.set_interpolation(True)
                        servo.set_subdivisions(64)
                        time.sleep(0.05)
                    except Exception as e:
                        logger.warning(
                            f"Failed to enable interpolation on motor {joint_index}: {e}"
                        )
                
                # Create homing system
                self.homing = ArctosHoming(self.can) if active_motor_ids == {1, 2, 3, 4, 5, 6} else None
                
                self._connected = True
                logger.info("Arctos controller connected")
                return True
                
            except Exception as e:
                logger.error(f"Failed to connect: {e}")
                self.disconnect()
                return False
    
    def disconnect(self) -> None:
        """Disconnect from robot hardware."""
        with self._lock:
            if self.can:
                try:
                    self.can.disconnect()
                except Exception as e:
                    logger.warning(f"Error disconnecting CAN: {e}")
                finally:
                    self.can = None
            
            self.servos = []
            self.homing = None
            self._connected = False
            logger.info("Arctos controller disconnected")
    
    @property
    def is_connected(self) -> bool:
        """Check if connected to hardware."""
        return self._connected and self.can is not None and self.can.is_connected
    
    def _check_connected(self):
        """Raise error if not connected."""
        if not self.is_connected:
            raise RuntimeError("Not connected to robot hardware")
    
    # ===== Coordinate Transformation =====
    
    def angle_to_encoder(self, angle_rad: float, joint_index: int) -> int:
        """
        Convert joint angle to encoder value.
        
        Args:
            angle_rad: Joint angle in radians
            joint_index: Joint index (0-5)
            
        Returns:
            Encoder value
        """
        joint = self.config.joints[joint_index]
        
        # Apply direction and offset
        angle_corrected = (angle_rad - joint.offset) * joint.direction
        
        # Convert to encoder
        encoder = int(
            (angle_corrected / (2 * math.pi)) *
            self.config.encoder_resolution *
            joint.gear_ratio
        )
        
        return encoder
    
    def encoder_to_angle(self, encoder_value: int, joint_index: int) -> float:
        """
        Convert encoder value to joint angle.
        
        Args:
            encoder_value: Encoder value
            joint_index: Joint index (0-5)
            
        Returns:
            Joint angle in radians
        """
        joint = self.config.joints[joint_index]
        
        # Convert from encoder
        angle_corrected = (
            (encoder_value / (self.config.encoder_resolution * joint.gear_ratio)) *
            2 * math.pi
        )
        
        # Apply inverse direction and offset
        angle_rad = (angle_corrected / joint.direction) + joint.offset
        
        return angle_rad
    
    # ===== Reading =====
    
    def read_joint_positions(self) -> List[float]:
        """
        Read current joint positions.
        
        Handles differential wrist transformation automatically.
        
        Returns:
            List of 6 joint angles in radians
        """
        self._check_connected()
        
        positions = [0.0] * self.NUM_JOINTS
        
        with self._lock:
            encoder_values = [0] * 6
            active_indices = [
                idx for idx, servo in enumerate(self.servos)
                if servo is not None
            ]

            for idx in active_indices:
                value = self.servos[idx].read_encoder_value()
                encoder_values[idx] = value or 0
                time.sleep(self.config.read_spacing_s)
            
            # Convert joints 1-4 directly
            for i in range(4):
                if self.servos[i] is not None:
                    positions[i] = self.encoder_to_angle(encoder_values[i], i)
            
            # Handle differential wrist
            if self.config.coupled_axis_mode and self.servos[4] is not None and self.servos[5] is not None:
                positions[4], positions[5] = self.wrist.motors_to_joints(
                    encoder_values[4], encoder_values[5]
                )
            else:
                if self.servos[4] is not None:
                    positions[4] = self.encoder_to_angle(encoder_values[4], 4)
                if self.servos[5] is not None:
                    positions[5] = self.encoder_to_angle(encoder_values[5], 5)
        
        return positions
    
    def read_joint_velocities(self) -> List[float]:
        """
        Read current joint velocities.
        
        Returns:
            List of 6 joint velocities in rad/s
        """
        self._check_connected()
        
        velocities = [0.0] * self.NUM_JOINTS
        
        with self._lock:
            for i in range(6):
                servo = self.servos[i]
                if servo is None:
                    continue
                rpm = servo.read_motor_speed()
                if rpm is not None:
                    # Convert RPM to rad/s
                    joint = self.config.joints[i]
                    velocities[i] = (rpm / 60.0) * 2 * math.pi / joint.gear_ratio
        
        return velocities
    
    def get_joint_states(self) -> Tuple[List[float], List[float]]:
        """
        Get current joint positions and velocities.
        
        Returns:
            Tuple of (positions, velocities)
        """
        return self.read_joint_positions(), self.read_joint_velocities()
    
    # ===== Commanding =====
    
    def validate_positions(self, positions: List[float]) -> bool:
        """
        Validate joint positions are within limits.
        
        Args:
            positions: List of 6 joint angles in radians
            
        Returns:
            True if all within limits
        """
        if len(positions) != self.NUM_JOINTS:
            return False
        
        for i, pos in enumerate(positions):
            joint = self.config.joints[i]
            if not (joint.min_limit <= pos <= joint.max_limit):
                logger.warning(
                    f"Joint {i+1} position {math.degrees(pos):.2f}° "
                    f"out of limits [{math.degrees(joint.min_limit):.2f}, "
                    f"{math.degrees(joint.max_limit):.2f}]"
                )
                return False
        
        return True
    
    def move_to_positions(
        self,
        positions: List[float],
        speeds: Optional[List[int]] = None,
        accelerations: Optional[List[int]] = None,
        validate: bool = True
    ) -> bool:
        """
        Move all joints to specified positions.
        
        Handles differential wrist transformation automatically.
        
        Args:
            positions: List of 6 joint angles in radians
            speeds: Optional list of 6 speeds in RPM
            accelerations: Optional list of 6 accelerations
            validate: Whether to validate limits
            
        Returns:
            True if commands accepted
        """
        self._check_connected()
        
        if len(positions) != self.NUM_JOINTS:
            raise ValueError(f"Expected {self.NUM_JOINTS} positions, got {len(positions)}")
        
        if validate and not self.validate_positions(positions):
            logger.error("Position validation failed")
            return False
        
        # Default speeds and accelerations
        if speeds is None:
            speeds = list(self.config.default_joint_speeds_rpm)
        if accelerations is None:
            accelerations = list(self.config.default_joint_accelerations)
        
        with self._lock:
            # Calculate encoder targets
            encoder_targets = [0] * 6
            
            # Joints 1-4: direct conversion
            for i in range(4):
                encoder_targets[i] = self.angle_to_encoder(positions[i], i)
            
            # Joints 5-6: differential wrist
            if self.config.coupled_axis_mode:
                encoder_targets[4], encoder_targets[5] = self.wrist.joints_to_motors(
                    positions[4], positions[5]
                )
            else:
                encoder_targets[4] = self.angle_to_encoder(positions[4], 4)
                encoder_targets[5] = self.angle_to_encoder(positions[5], 5)
            
            # Send commands sequentially with a small spacing. The MKS drivers
            # are sensitive to bursts of request/response traffic during motion,
            # and parallel submissions were causing intermittent missed replies.
            results = []
            for i in range(6):
                servo = self.servos[i]
                if servo is None:
                    continue

                success = servo.move_to_position(
                    encoder_targets[i],
                    speeds[i],
                    accelerations[i]
                )

                if not success:
                    logger.warning(
                        f"Motor {i + 1}: move command missed, retrying once"
                    )
                    time.sleep(self.config.command_retry_delay_s)
                    success = servo.move_to_position(
                        encoder_targets[i],
                        speeds[i],
                        accelerations[i]
                    )

                results.append(success)
                time.sleep(self.config.command_spacing_s)
            
            if all(results):
                self._commanded_positions = list(positions)
                return True
            else:
                logger.error("Some motor commands failed")
                return False
    
    def move_to_positions_no_wait(
        self,
        positions: List[float],
        speeds: Optional[List[int]] = None,
        accelerations: Optional[List[int]] = None,
        validate: bool = True
    ) -> None:
        """
        Fire-and-forget version of move_to_positions.

        Sends absolute position commands without waiting for motor ACKs,
        keeping CAN bus time minimal so the control loop can spend most of
        its budget on state reads.
        """
        self._check_connected()

        if len(positions) != self.NUM_JOINTS:
            raise ValueError(f"Expected {self.NUM_JOINTS} positions, got {len(positions)}")

        if validate and not self.validate_positions(positions):
            logger.error("Position validation failed")
            return

        if speeds is None:
            speeds = list(self.config.default_joint_speeds_rpm)
        if accelerations is None:
            accelerations = list(self.config.default_joint_accelerations)

        with self._lock:
            encoder_targets = [0] * 6

            for i in range(4):
                encoder_targets[i] = self.angle_to_encoder(positions[i], i)

            if self.config.coupled_axis_mode:
                encoder_targets[4], encoder_targets[5] = self.wrist.joints_to_motors(
                    positions[4], positions[5]
                )
            else:
                encoder_targets[4] = self.angle_to_encoder(positions[4], 4)
                encoder_targets[5] = self.angle_to_encoder(positions[5], 5)

            for i in range(6):
                servo = self.servos[i]
                if servo is None:
                    continue
                servo.move_to_position(
                    encoder_targets[i],
                    speeds[i],
                    accelerations[i],
                    wait_for_ack=False,
                )
                time.sleep(self.config.command_spacing_no_ack_s)

            self._commanded_positions = list(positions)

    def stop_all(self, deceleration: int = 255) -> None:
        """
        Stop all motors gracefully.
        
        Args:
            deceleration: Deceleration rate
        """
        if not self.is_connected:
            return
        
        with self._lock:
            for servo in self.servos:
                if servo is None:
                    continue
                try:
                    servo.stop(deceleration)
                except Exception as e:
                    logger.error(f"Error stopping motor: {e}")
    
    def emergency_stop(self) -> None:
        """Emergency stop all motors immediately."""
        if not self.is_connected:
            return
        
        logger.warning("EMERGENCY STOP")
        
        with self._lock:
            for servo in self.servos:
                if servo is None:
                    continue
                try:
                    servo.emergency_stop()
                except Exception as e:
                    logger.error(f"Error in emergency stop: {e}")
    
    # ===== Motor Control =====
    
    def enable_motors(self) -> bool:
        """Enable all motors."""
        self._check_connected()
        
        success = True
        for servo in self.servos:
            if servo is None:
                continue
            if not servo.enable():
                success = False
        
        return success
    
    def disable_motors(self) -> bool:
        """Disable all motors."""
        self._check_connected()
        
        success = True
        for servo in self.servos:
            if servo is None:
                continue
            if not servo.disable():
                success = False
        
        return success
    
    def is_moving(self) -> bool:
        """Check if any motor is currently moving."""
        if not self.is_connected:
            return False
        
        for servo in self.servos:
            if servo is None:
                continue
            if servo.is_running():
                return True
        
        return False
    
    def wait_for_idle(self, timeout: float = 30.0) -> bool:
        """
        Wait for all motors to stop moving.
        
        Args:
            timeout: Maximum time to wait
            
        Returns:
            True if all stopped, False on timeout
        """
        start = time.time()
        
        while time.time() - start < timeout:
            if not self.is_moving():
                return True
            time.sleep(0.05)
        
        return False
    
    # ===== Homing =====
    
    def home_all(self) -> bool:
        """
        Home all joints.
        
        Returns:
            True if homing successful
        """
        self._check_connected()
        
        if self.homing is None:
            logger.warning("Homing is only available when all six motors are active")
            return False
        
        return self.homing.home_all()
    
    def set_zero_all(self) -> bool:
        """
        Set current positions as zero for all joints.
        
        Returns:
            True if successful
        """
        self._check_connected()
        
        if self.homing is not None:
            return self.homing.set_zero_all()

        success = True
        for servo in self.servos:
            if servo is None:
                continue
            if not servo.set_zero():
                success = False

        return success
    
    # ===== Context Manager =====
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False
    
    def __del__(self):
        """Destructor."""
        self.disconnect()


# Convenience function for quick testing
def test_connection(can_device: str = default_can_device()) -> bool:
    """
    Test connection to Arctos robot.
    
    Args:
        can_device: CAN device path
        
    Returns:
        True if connection successful
    """
    config = ArctosConfig(can_device=can_device)
    
    with ArctosController(config) as controller:
        if controller.is_connected:
            positions = controller.read_joint_positions()
            print(f"Current positions (degrees): {[f'{math.degrees(p):.2f}' for p in positions]}")
            return True
    
    return False


if __name__ == '__main__':
    import sys
    
    logging.basicConfig(level=logging.INFO)
    
    device = sys.argv[1] if len(sys.argv) > 1 else default_can_device()
    success = test_connection(device)
    
    sys.exit(0 if success else 1)
