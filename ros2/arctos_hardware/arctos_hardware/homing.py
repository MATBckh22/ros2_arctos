"""
Homing and Calibration System for Arctos Robot.

Provides comprehensive homing procedures including:
- Pre-homing limit switch check (handles "already at limit" case)
- Standard joint homing using limit switches (0x91)
- Set current position as zero (0x92)
- Synchronized differential wrist homing

The differential wrist requires special handling because both motors (B and C)
affect both joints (5 and 6). The homing sequence must synchronize the zero
positions of both motors.
"""

import time
import logging
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Optional, Callable, Dict
from concurrent.futures import ThreadPoolExecutor, as_completed

from .can_interface import CanInterface
from .mks_servo import MksServo, Direction, EndStopLevel, HomingStatus, MotorStatus

logger = logging.getLogger(__name__)


class HomingState(Enum):
    """State of the homing procedure."""
    IDLE = auto()
    CHECKING_LIMITS = auto()
    BACKING_UP = auto()
    IN_PROGRESS = auto()
    SUCCESS = auto()
    FAILED = auto()
    ABORTED = auto()


@dataclass
class HomingParameters:
    """
    MKS SERVO homing parameters.
    
    Based on MKS SERVO42&57D_CAN User Manual V1.0.6.
    """
    # Motor driver settings
    microsteps: int = 16                    # Microstepping subdivision
    encoder_resolution: int = 16384         # Encoder pulses per revolution
    
    # Homing speeds (RPM)
    homing_speed: int = 300                 # Speed when searching for limit
    backup_speed: int = 100                 # Speed when backing off limit
    
    # Backup distance (degrees) - used if already at limit
    backup_distance_deg: float = 5.0        # Degrees to back up
    
    # Timeouts
    homing_timeout: float = 30.0            # Max time per joint (seconds)
    backup_timeout: float = 5.0             # Max time for backup move
    
    # Acceleration (0-255, higher = faster accel/decel)
    acceleration: int = 50                  # Moderate acceleration for safety
    
    @property
    def backup_distance_pulses(self) -> int:
        """Convert backup distance to encoder pulses."""
        pulses_per_degree = self.encoder_resolution / 360.0
        return int(self.backup_distance_deg * pulses_per_degree)


@dataclass
class JointHomingConfig:
    """Configuration for homing a single joint."""
    motor_id: int
    trigger_level: EndStopLevel = EndStopLevel.LOW
    limit_switch_input: int = 1            # 1 = IN1, 2 = IN2
    backup_delta_sign: int = 0             # -1 or +1 to force backup sign, 0 = infer
    direction: Direction = Direction.CW
    speed: int = 300                        # RPM for homing
    backup_speed: int = 100                 # RPM for backup
    zero_offset_speed: int = 0              # RPM for post-home move (0 = use homing speed)
    enable_limit: bool = True
    timeout: float = 30.0
    backup_distance_deg: float = 5.0        # Degrees to backup if at limit
    zero_offset_axis: int = 0               # Post-home relative move in motor-axis counts
    
    @property
    def opposite_direction(self) -> Direction:
        """Get opposite direction for backup movement."""
        return Direction.CCW if self.direction == Direction.CW else Direction.CW


@dataclass 
class HomingResult:
    """Result of a homing operation."""
    motor_id: int
    success: bool
    state: HomingState
    message: str = ""
    duration: float = 0.0
    was_at_limit: bool = False              # Was motor already at limit switch?


@dataclass
class ArctosHomingConfig:
    """Complete homing configuration for Arctos robot."""
    # Default homing parameters
    params: HomingParameters = field(default_factory=HomingParameters)

    # Gear ratios used to convert joint backup angles into motor encoder counts.
    gear_ratios: Dict[int, float] = field(default_factory=lambda: {
        1: 13.5,
        2: 150.0,
        3: 150.0,
        4: 48.0,
        5: 67.82,
        6: 67.82,
    })
    
    # Standard joints (1-4) configurations
    joint_configs: Dict[int, JointHomingConfig] = field(default_factory=lambda: {
        1: JointHomingConfig(
            motor_id=1,
            limit_switch_input=1,
            direction=Direction.CW,
            speed=60,
            backup_speed=20,
            timeout=90.0,
            backup_distance_deg=5.0,
            zero_offset_axis=-106005
        ),
        2: JointHomingConfig(
            motor_id=2,
            limit_switch_input=1,
            direction=Direction.CW,
            speed=80,
            backup_speed=30,
            backup_distance_deg=5.0,
            zero_offset_axis=-409600
        ),
        3: JointHomingConfig(
            motor_id=3,
            limit_switch_input=1,
            backup_delta_sign=-1,
            direction=Direction.CCW,
            speed=80,
            backup_speed=30,
            backup_distance_deg=5.0,
            zero_offset_axis=239200
        ),
        4: JointHomingConfig(
            motor_id=4,
            trigger_level=EndStopLevel.LOW,
            limit_switch_input=1,
            direction=Direction.CW,
            speed=60,
            backup_speed=20,
            zero_offset_speed=90,
            backup_distance_deg=5.0,
            zero_offset_axis=-145932
        ),
    })
    
    # Differential wrist motors (5, 6 = B, C)
    wrist_b_config: JointHomingConfig = field(
        default_factory=lambda: JointHomingConfig(
            motor_id=5,
            trigger_level=EndStopLevel.LOW,
            limit_switch_input=1,
            direction=Direction.CCW,
            speed=80,
            backup_speed=30,
            zero_offset_speed=60,
            backup_distance_deg=3.0,         # Smaller backup for wrist
            zero_offset_axis=121972
        )
    )
    wrist_c_config: JointHomingConfig = field(
        default_factory=lambda: JointHomingConfig(
            motor_id=6,
            trigger_level=EndStopLevel.LOW,
            limit_switch_input=1,
            direction=Direction.CCW,
            speed=80,
            backup_speed=30,
            zero_offset_speed=60,
            backup_distance_deg=3.0,
            zero_offset_axis=-79420
        )
    )
    
    # Homing order (reverse order to avoid collisions: wrist→elbow→shoulder→base)
    homing_order: List[int] = field(default_factory=lambda: [4, 3, 2, 1])
    
    # Number of differential wrist homing iterations (for backlash compensation)
    wrist_iterations: int = 2
    
    # Position verification tolerance (encoder counts)
    zero_tolerance: int = 100
    
    # Enable pre-homing limit check
    check_limits_before_homing: bool = True


class ArctosHoming:
    """
    Homing and calibration system for Arctos robot.
    
    Features:
    - Pre-homing limit switch check to handle "already at limit" case
    - Automatic backup if motor is at limit before homing
    - Standard joints (1-4) homing to limit switches
    - Differential wrist synchronized homing
    - Zero position verification
    
    The differential wrist homing follows the community-recommended procedure:
    - Home B motor, simultaneously zero C motor
    - Home C motor, simultaneously zero B motor
    - Repeat for precision (backlash compensation)
    """
    
    ENCODER_RESOLUTION = 16384              # Pulses per revolution
    
    def __init__(
        self,
        can_interface: CanInterface,
        config: Optional[ArctosHomingConfig] = None,
        progress_callback: Optional[Callable[[str, float], None]] = None
    ):
        """
        Initialize homing system.
        
        Args:
            can_interface: CAN interface for communication
            config: Homing configuration
            progress_callback: Optional callback for progress updates
                              Signature: callback(message: str, progress: float)
        """
        self.can = can_interface
        self.config = config or ArctosHomingConfig()
        self.progress_callback = progress_callback
        self.state = HomingState.IDLE
        self._abort_requested = False
        
        # Create servo instances
        self.servos: Dict[int, MksServo] = {}
        for motor_id in range(1, 7):
            self.servos[motor_id] = MksServo(can_interface, motor_id)
    
    def _report_progress(self, message: str, progress: float = 0.0):
        """Report progress through callback if available."""
        logger.info(message)
        if self.progress_callback:
            self.progress_callback(message, progress)
    
    def abort(self):
        """Request abortion of current homing operation."""
        self._abort_requested = True
        logger.warning("Homing abort requested")
        # Stop all motors immediately
        self.emergency_stop_all()
    
    def check_limit_switch(self, motor_id: int) -> Optional[bool]:
        """
        Check if motor is currently at limit switch.
        
        Uses READ_IO_STATUS (0x34) command to read limit switch state.
        
        Args:
            motor_id: Motor CAN ID
            
        Returns:
            True if at limit, False if not, None if read failed
        """
        servo = self.servos[motor_id]
        
        try:
            io_status = servo.read_io_status()
            if io_status is None:
                logger.warning(f"Motor {motor_id}: Failed to read IO status")
                return None
            
            if motor_id in self.config.joint_configs:
                trigger_level = self.config.joint_configs[motor_id].trigger_level
            elif motor_id == self.config.wrist_b_config.motor_id:
                trigger_level = self.config.wrist_b_config.trigger_level
            elif motor_id == self.config.wrist_c_config.motor_id:
                trigger_level = self.config.wrist_c_config.trigger_level
            else:
                trigger_level = EndStopLevel.LOW

            if motor_id in self.config.joint_configs:
                limit_switch_input = self.config.joint_configs[motor_id].limit_switch_input
            elif motor_id == self.config.wrist_b_config.motor_id:
                limit_switch_input = self.config.wrist_b_config.limit_switch_input
            elif motor_id == self.config.wrist_c_config.motor_id:
                limit_switch_input = self.config.wrist_c_config.limit_switch_input
            else:
                limit_switch_input = 1

            if limit_switch_input not in (1, 2):
                raise ValueError(f"Unsupported limit switch input {limit_switch_input}")

            input_mask = 1 << (limit_switch_input - 1)
            input_active = (io_status & input_mask) != 0
            at_limit = (not input_active) if trigger_level == EndStopLevel.LOW else input_active

            logger.debug(
                f"Motor {motor_id}: IO status=0x{io_status:02X}, "
                f"input=IN{limit_switch_input}, at_limit={at_limit}"
            )
            return at_limit
            
        except Exception as e:
            logger.error(f"Motor {motor_id}: Error checking limit switch: {e}")
            return None
    
    def check_all_limit_switches(self) -> Dict[int, Optional[bool]]:
        """
        Check limit switch status for all motors.
        
        Returns:
            Dictionary mapping motor_id to limit switch state
        """
        results = {}
        for motor_id in range(1, 7):
            results[motor_id] = self.check_limit_switch(motor_id)
        return results
    
    def backup_from_limit(
        self,
        motor_id: int,
        config: JointHomingConfig
    ) -> bool:
        """
        Back up motor from limit switch.
        
        Moves motor in opposite direction to clear the limit switch.
        
        Args:
            motor_id: Motor CAN ID
            config: Joint homing configuration
            
        Returns:
            True if backup successful
        """
        servo = self.servos[motor_id]
        
        # Calculate backup distance in pulses. READ/relative axis commands use the
        # motor encoder axis, so convert the requested joint-space backup angle
        # through the configured reduction ratio.
        gear_ratio = self.config.gear_ratios.get(motor_id, 1.0)
        pulses_per_degree = (self.ENCODER_RESOLUTION * gear_ratio) / 360.0
        backup_pulses = int(config.backup_distance_deg * pulses_per_degree)

        logger.info(
            f"Motor {motor_id}: Backing up {config.backup_distance_deg}° "
            f"with gear ratio {gear_ratio} ({backup_pulses} pulses) at {config.backup_speed} RPM"
        )
        
        try:
            if config.backup_delta_sign in (-1, 1):
                deltas = (config.backup_delta_sign * backup_pulses,)
            else:
                preferred_delta = -backup_pulses if config.direction == Direction.CW else backup_pulses
                deltas = (preferred_delta, -preferred_delta)

            for attempt, delta in enumerate(deltas, start=1):
                logger.info(
                    f"Motor {motor_id}: Backup attempt {attempt}/{len(deltas)} with delta {delta}"
                )

                success = servo.move_relative(
                    delta=delta,
                    speed=config.backup_speed,
                    acceleration=50
                )

                if not success:
                    logger.error(f"Motor {motor_id}: Failed to start backup movement")
                    continue

                timeout = 5.0
                start_time = time.time()

                while time.time() - start_time < timeout:
                    at_limit = self.check_limit_switch(motor_id)

                    if at_limit is False:
                        logger.info(f"Motor {motor_id}: Backup complete, limit switch clear")
                        time.sleep(0.2)
                        return True

                    if not servo.is_running():
                        break

                    time.sleep(0.1)

                at_limit = self.check_limit_switch(motor_id)
                if at_limit is False:
                    logger.info(f"Motor {motor_id}: Backup complete")
                    return True

                servo.stop()
                time.sleep(0.1)

            logger.error(f"Motor {motor_id}: Still at limit after backup")
            return False
                
        except Exception as e:
            logger.error(f"Motor {motor_id}: Backup error: {e}")
            return False
    
    def home_single_joint(
        self,
        motor_id: int,
        config: Optional[JointHomingConfig] = None,
        check_limit_first: bool = True
    ) -> HomingResult:
        """
        Home a single joint with optional pre-homing limit check.
        
        If the motor is already at the limit switch, it will back up first
        before attempting to home.
        
        Args:
            motor_id: Motor CAN ID to home
            config: Homing configuration for this joint
            check_limit_first: Check and handle "already at limit" case
            
        Returns:
            HomingResult with success status
        """
        config = config or self.config.joint_configs.get(
            motor_id,
            JointHomingConfig(motor_id=motor_id)
        )
        
        servo = self.servos[motor_id]
        start_time = time.time()
        was_at_limit = False
        
        try:
            # Step 0: Clear any stale homing/motion state from a previous run.
            status = servo.query_status()
            if status in (
                MotorStatus.HOMING,
                MotorStatus.SPEED_UP,
                MotorStatus.SPEED_DOWN,
                MotorStatus.FULL_SPEED,
            ):
                logger.warning(f"Motor {motor_id}: Motor is in state {status}, stopping first...")
                servo.emergency_stop()
                time.sleep(0.3)
                servo.enable()
                time.sleep(0.2)
                status = servo.query_status()
                logger.info(f"Motor {motor_id}: After stop, status={status}")

            # Step 1: Check if already at limit switch
            if check_limit_first and self.config.check_limits_before_homing:
                self.state = HomingState.CHECKING_LIMITS
                logger.info(f"Motor {motor_id}: Checking limit switch status...")
                
                at_limit = self.check_limit_switch(motor_id)
                
                if at_limit is True:
                    was_at_limit = True
                    logger.warning(
                        f"Motor {motor_id}: Already at limit switch. "
                        "Using MKS go-home behavior to back off automatically."
                    )
                    self._report_progress(
                        f"Motor {motor_id}: At limit - driver will back off automatically...",
                        0.0
                    )
                elif at_limit is None:
                    logger.warning(
                        f"Motor {motor_id}: Could not read limit switch, "
                        f"proceeding with caution..."
                    )
            
            # Step 2: Configure homing parameters
            self.state = HomingState.IN_PROGRESS
            logger.debug(f"Motor {motor_id}: Configuring homing parameters")
            logger.info(
                f"Motor {motor_id}: Homing at {config.speed} RPM, "
                f"direction={'CW' if config.direction == Direction.CW else 'CCW'}"
            )

            if was_at_limit:
                logger.info(f"Motor {motor_id}: Backing off active home switch before homing...")
                servo.emergency_stop()
                time.sleep(0.3)

                servo.configure_home(
                    trigger_level=config.trigger_level,
                    direction=config.direction,
                    speed=config.speed,
                    enable_limit=False
                )
                time.sleep(0.1)

                servo.enable()
                time.sleep(0.2)

                if not self.backup_from_limit(motor_id, config):
                    return HomingResult(
                        motor_id=motor_id,
                        success=False,
                        state=HomingState.FAILED,
                        message="Failed to back off from active home switch",
                        was_at_limit=was_at_limit
                    )
            
            if not servo.configure_home(
                trigger_level=config.trigger_level,
                direction=config.direction,
                speed=config.speed,
                enable_limit=config.enable_limit
            ):
                return HomingResult(
                    motor_id=motor_id,
                    success=False,
                    state=HomingState.FAILED,
                    message="Failed to configure homing parameters",
                    was_at_limit=was_at_limit
                )
            
            time.sleep(0.1)
            
            # Step 3: Execute homing. The reference ArctosGuiPython flow is:
            # start go-home, wait until the home switch is hit, then move to the
            # configured zero offset and set zero there. That is more reliable on
            # this firmware than trusting the 0x3B status alone.
            logger.info(f"Motor {motor_id}: Starting homing...")
            if not servo.go_home(blocking=False):
                return HomingResult(
                    motor_id=motor_id,
                    success=False,
                    state=HomingState.FAILED,
                    message="Failed to start homing",
                    was_at_limit=was_at_limit
                )

            homed = False
            switch_seen_time: Optional[float] = None
            home_start_time = time.time()
            while time.time() - home_start_time < config.timeout:
                if self._abort_requested:
                    return HomingResult(
                        motor_id=motor_id,
                        success=False,
                        state=HomingState.ABORTED,
                        message="Aborted by user",
                        was_at_limit=was_at_limit
                    )

                at_limit = self.check_limit_switch(motor_id)
                if at_limit is True:
                    if switch_seen_time is None:
                        switch_seen_time = time.time()

                    motor_status = servo.query_status()
                    homing_status = servo.read_homing_status()
                    logger.info(
                        f"Motor {motor_id}: Home switch active, waiting for homing state to settle "
                        f"(status={motor_status}, homing_status={homing_status})"
                    )

                    if homing_status == HomingStatus.SUCCESS or (
                        motor_status == MotorStatus.STOPPED and time.time() - switch_seen_time >= 0.3
                    ):
                        logger.info(
                            f"Motor {motor_id}: Home switch reached, clearing homing state..."
                        )
                        servo.emergency_stop()
                        time.sleep(0.3)
                        servo.enable()
                        time.sleep(0.2)
                        logger.info(
                            f"Motor {motor_id}: After clearing homing state, status={servo.query_status()}"
                        )
                        homed = True
                        break
                else:
                    switch_seen_time = None

                status = servo.query_status()
                if status == MotorStatus.STOPPED and at_limit is False:
                    break

                time.sleep(0.1)

            if not homed:
                return HomingResult(
                    motor_id=motor_id,
                    success=False,
                    state=HomingState.FAILED,
                    message="Homing failed or timed out",
                    was_at_limit=was_at_limit
                )

            # The homing-state clear above should already have stopped the
            # driver. Keep this as a defensive second check only.
            status = servo.query_status()
            if status == MotorStatus.HOMING:
                logger.info(f"Motor {motor_id}: Stopping motor to exit homing state...")
                servo.emergency_stop()
                time.sleep(0.3)
                servo.enable()
                time.sleep(0.2)

            if config.zero_offset_axis != 0:
                zero_offset_speed = config.zero_offset_speed or config.speed
                servo.configure_home(
                    trigger_level=config.trigger_level,
                    direction=config.direction,
                    speed=config.speed,
                    enable_limit=False
                )
                time.sleep(0.1)

                logger.info(
                    f"Motor {motor_id}: Moving from limit switch to zero offset "
                    f"{config.zero_offset_axis} counts at {zero_offset_speed} RPM"
                )
                start_encoder = servo.read_encoder_value() or 0
                if not servo.move_relative(
                    delta=config.zero_offset_axis,
                    speed=zero_offset_speed,
                    acceleration=50,
                ):
                    return HomingResult(
                        motor_id=motor_id,
                        success=False,
                        state=HomingState.FAILED,
                        message="Failed to move to configured zero offset",
                        was_at_limit=was_at_limit,
                    )

                moved_off_switch = False
                reached_offset_motion = False
                move_start_time = time.time()
                while time.time() - move_start_time < config.timeout:
                    current_encoder = servo.read_encoder_value()
                    at_limit = self.check_limit_switch(motor_id)

                    if current_encoder is not None and abs(current_encoder - start_encoder) > 1000:
                        reached_offset_motion = True

                    if at_limit is False:
                        moved_off_switch = True

                    if moved_off_switch and reached_offset_motion and not servo.is_running():
                        break

                    time.sleep(0.1)

                if not moved_off_switch:
                    return HomingResult(
                        motor_id=motor_id,
                        success=False,
                        state=HomingState.FAILED,
                        message="Configured zero offset move never cleared the home switch",
                        was_at_limit=was_at_limit,
                    )

                if not reached_offset_motion:
                    return HomingResult(
                        motor_id=motor_id,
                        success=False,
                        state=HomingState.FAILED,
                        message="Configured zero offset move did not change encoder position",
                        was_at_limit=was_at_limit,
                    )

                if not servo.set_zero():
                    return HomingResult(
                        motor_id=motor_id,
                        success=False,
                        state=HomingState.FAILED,
                        message="Failed to set zero after configured offset move",
                        was_at_limit=was_at_limit,
                    )
             
            duration = time.time() - start_time
            logger.info(f"Motor {motor_id}: Homing complete in {duration:.1f}s")
            
            return HomingResult(
                motor_id=motor_id,
                success=True,
                state=HomingState.SUCCESS,
                message="Homing successful",
                duration=duration,
                was_at_limit=was_at_limit
            )
            
        except Exception as e:
            logger.error(f"Motor {motor_id}: Homing error: {e}")
            return HomingResult(
                motor_id=motor_id,
                success=False,
                state=HomingState.FAILED,
                message=str(e),
                was_at_limit=was_at_limit
            )
    
    def set_zero_single(self, motor_id: int) -> bool:
        """
        Set current position as zero for a single motor.
        
        Args:
            motor_id: Motor CAN ID
            
        Returns:
            True if successful
        """
        return self.servos[motor_id].set_zero()
    
    def set_zero_all(self) -> bool:
        """
        Set current position as zero for all motors.
        
        Useful when robot is manually positioned at home.
        
        Returns:
            True if all motors zeroed successfully
        """
        self._report_progress("Setting all motors to zero at current position...", 0.0)
        
        all_success = True
        
        for i, motor_id in enumerate(range(1, 7)):
            self._report_progress(f"Zeroing motor {motor_id}...", i / 6.0)
            
            if not self.set_zero_single(motor_id):
                logger.error(f"Motor {motor_id}: Failed to set zero")
                all_success = False
            else:
                logger.info(f"Motor {motor_id}: Zero set")
            
            time.sleep(0.1)
        
        if all_success:
            self._report_progress("All motors zeroed successfully", 1.0)
        else:
            self._report_progress("Some motors failed to zero", 1.0)
        
        return all_success
    
    def home_standard_joints(self) -> List[HomingResult]:
        """
        Home standard joints (1-4) in sequence.
        
        Homes in reverse order (4→1) to avoid collisions.
        Checks for limit switch status before each joint.
        
        Returns:
            List of HomingResult for each joint
        """
        results = []
        total = len(self.config.homing_order)
        
        for i, motor_id in enumerate(self.config.homing_order):
            if self._abort_requested:
                results.append(HomingResult(
                    motor_id=motor_id,
                    success=False,
                    state=HomingState.ABORTED,
                    message="Aborted by user"
                ))
                break
            
            progress = i / total
            self._report_progress(f"Homing joint {motor_id}...", progress)
            
            config = self.config.joint_configs.get(motor_id) or JointHomingConfig(motor_id=motor_id)
            result = self.home_single_joint(motor_id, config)
            results.append(result)
            
            if result.was_at_limit:
                self._report_progress(
                    f"Joint {motor_id}: Was at limit, backed up first",
                    progress
                )
            
            if not result.success:
                logger.error(f"Joint {motor_id} homing failed, aborting sequence")
                break
            
            time.sleep(0.5)
        
        return results
    
    def home_differential_wrist(self) -> bool:
        """
        Home the differential wrist with synchronized zeroing.
        
        Procedure (repeated for precision):
        1. Check if B motor at limit, backup if needed
        2. Home B motor
        3. When B reaches home, zero C motor
        4. Check if C motor at limit, backup if needed
        5. Home C motor
        6. When C reaches home, zero B motor
        
        Returns:
            True if successful
        """
        servo_b = self.servos[5]
        servo_c = self.servos[6]
        
        self._report_progress("Homing differential wrist...", 0.0)
        
        for iteration in range(self.config.wrist_iterations):
            if self._abort_requested:
                return False
                
            self._report_progress(
                f"Wrist homing iteration {iteration + 1}/{self.config.wrist_iterations}",
                iteration / self.config.wrist_iterations
            )
            
            # === Home B Motor ===
            logger.info(f"[Iteration {iteration + 1}] Checking B motor (ID 5) limit switch...")
            
            # Check if B is at limit
            at_limit_b = self.check_limit_switch(5)
            if at_limit_b is True:
                logger.warning("B motor at limit, backing up...")
                if not self.backup_from_limit(5, self.config.wrist_b_config):
                    logger.error("B motor backup failed")
                    return False
                time.sleep(0.2)
            
            # Configure and home B motor
            logger.info("Configuring B motor (ID 5) for homing...")
            servo_b.configure_home(
                trigger_level=self.config.wrist_b_config.trigger_level,
                direction=self.config.wrist_b_config.direction,
                speed=self.config.wrist_b_config.speed,
                enable_limit=self.config.wrist_b_config.enable_limit
            )
            time.sleep(0.1)
            
            logger.info("Homing B motor...")
            if not servo_b.go_home(blocking=True, timeout=self.config.wrist_b_config.timeout):
                logger.error("B motor homing failed")
                return False
            
            # B reached home - zero C motor
            logger.info("B motor homed, zeroing C motor...")
            if not servo_c.set_zero():
                logger.error("Failed to zero C motor")
                return False
            
            time.sleep(0.3)
            
            # === Home C Motor ===
            logger.info(f"[Iteration {iteration + 1}] Checking C motor (ID 6) limit switch...")
            
            # Check if C is at limit
            at_limit_c = self.check_limit_switch(6)
            if at_limit_c is True:
                logger.warning("C motor at limit, backing up...")
                if not self.backup_from_limit(6, self.config.wrist_c_config):
                    logger.error("C motor backup failed")
                    return False
                time.sleep(0.2)
            
            # Configure and home C motor
            logger.info("Configuring C motor (ID 6) for homing...")
            servo_c.configure_home(
                trigger_level=self.config.wrist_c_config.trigger_level,
                direction=self.config.wrist_c_config.direction,
                speed=self.config.wrist_c_config.speed,
                enable_limit=self.config.wrist_c_config.enable_limit
            )
            time.sleep(0.1)
            
            # Start C motor homing
            logger.info("Homing C motor...")
            if not servo_c.go_home(blocking=True, timeout=self.config.wrist_c_config.timeout):
                logger.error("C motor homing failed")
                return False
            
            # C reached home - zero B motor
            logger.info("C motor homed, zeroing B motor...")
            if not servo_b.set_zero():
                logger.error("Failed to zero B motor")
                return False
            
            time.sleep(0.3)

        # Apply coupled B/C zero offsets from the GUI's coupled-axis mode.
        # These are raw motor-axis offsets for B and C, not joint-space J5/J6 offsets.
        offset_b = self.config.wrist_b_config.zero_offset_axis
        offset_c = self.config.wrist_c_config.zero_offset_axis
        if offset_b != 0 or offset_c != 0:
            logger.info(
                "Applying coupled wrist zero offsets: "
                f"B={offset_b}, C={offset_c}"
            )

            servo_b.configure_home(
                trigger_level=self.config.wrist_b_config.trigger_level,
                direction=self.config.wrist_b_config.direction,
                speed=self.config.wrist_b_config.speed,
                enable_limit=False,
            )
            servo_c.configure_home(
                trigger_level=self.config.wrist_c_config.trigger_level,
                direction=self.config.wrist_c_config.direction,
                speed=self.config.wrist_c_config.speed,
                enable_limit=False,
            )
            time.sleep(0.1)

            move_jobs = []
            if offset_b != 0:
                move_jobs.append(
                    (
                        "B",
                        servo_b,
                        self.config.wrist_b_config.zero_offset_speed or self.config.wrist_b_config.speed,
                        offset_b,
                    )
                )
            if offset_c != 0:
                move_jobs.append(
                    (
                        "C",
                        servo_c,
                        self.config.wrist_c_config.zero_offset_speed or self.config.wrist_c_config.speed,
                        offset_c,
                    )
                )

            with ThreadPoolExecutor(max_workers=2) as executor:
                futures = {
                    executor.submit(
                        servo.move_relative,
                        delta=delta,
                        speed=speed,
                        acceleration=50,
                    ): axis_name
                    for axis_name, servo, speed, delta in move_jobs
                }

                for future in as_completed(futures):
                    axis_name = futures[future]
                    if not future.result():
                        logger.error(f"Failed to start wrist {axis_name} offset move")
                        return False

            move_start_time = time.time()
            while time.time() - move_start_time < max(
                self.config.wrist_b_config.timeout,
                self.config.wrist_c_config.timeout,
            ):
                running_b = offset_b != 0 and servo_b.is_running()
                running_c = offset_c != 0 and servo_c.is_running()
                if not running_b and not running_c:
                    break
                time.sleep(0.1)
            else:
                logger.error("Timed out while applying coupled wrist offsets")
                return False

            if not servo_b.set_zero():
                logger.error("Failed to set zero on B motor after coupled wrist offset move")
                return False
            if not servo_c.set_zero():
                logger.error("Failed to set zero on C motor after coupled wrist offset move")
                return False
        
        self._report_progress("Differential wrist homing complete", 1.0)
        return True
    
    def verify_zero_positions(self) -> bool:
        """
        Verify all motors are at or near zero position.
        
        Returns:
            True if all positions within tolerance
        """
        logger.info("Verifying zero positions...")
        all_ok = True
        
        for motor_id in range(1, 7):
            encoder = self.servos[motor_id].read_encoder_value()
            
            if encoder is None:
                logger.warning(f"Motor {motor_id}: Failed to read encoder")
                continue
            
            if abs(encoder) > self.config.zero_tolerance:
                logger.warning(
                    f"Motor {motor_id}: Encoder={encoder} exceeds tolerance "
                    f"(±{self.config.zero_tolerance})"
                )
                # Warning only, don't fail
            else:
                logger.debug(f"Motor {motor_id}: Encoder={encoder} OK")
        
        return all_ok
    
    def home_all(self) -> bool:
        """
        Execute complete homing sequence for all joints.
        
        Sequence:
        1. Check all limit switches
        2. Home standard joints 4→1
        3. Home differential wrist
        4. Verify zero positions
        
        Returns:
            True if all homing successful
        """
        self.state = HomingState.IN_PROGRESS
        self._abort_requested = False
        
        self._report_progress("=" * 50, 0.0)
        self._report_progress("ARCTOS ROBOT HOMING SEQUENCE", 0.0)
        self._report_progress("=" * 50, 0.0)
        self._report_progress(
            f"Homing speed: {self.config.params.homing_speed} RPM, "
            f"Backup speed: {self.config.params.backup_speed} RPM",
            0.0
        )
        
        try:
            # Step 0: Check all limit switches
            if self.config.check_limits_before_homing:
                self._report_progress("\n[0/4] Checking limit switches...", 0.0)
                limits = self.check_all_limit_switches()
                for motor_id, at_limit in limits.items():
                    status = "AT LIMIT" if at_limit else "clear" if at_limit is False else "unknown"
                    self._report_progress(f"  Motor {motor_id}: {status}", 0.0)
            
            # Step 1: Home standard joints
            self._report_progress("\n[1/3] Homing standard joints...", 0.0)
            joint_results = self.home_standard_joints()
            
            if not all(r.success for r in joint_results):
                self.state = HomingState.FAILED
                self._report_progress("Standard joint homing failed", 0.33)
                return False
            
            if self._abort_requested:
                self.state = HomingState.ABORTED
                return False
            
            # Step 2: Home differential wrist
            self._report_progress("\n[2/3] Homing differential wrist...", 0.33)
            if not self.home_differential_wrist():
                self.state = HomingState.FAILED
                self._report_progress("Differential wrist homing failed", 0.66)
                return False
            
            if self._abort_requested:
                self.state = HomingState.ABORTED
                return False
            
            # Step 3: Verify positions
            self._report_progress("\n[3/3] Verifying zero positions...", 0.66)
            self.verify_zero_positions()
            
            self.state = HomingState.SUCCESS
            self._report_progress("\n" + "=" * 50, 1.0)
            self._report_progress("HOMING COMPLETE", 1.0)
            self._report_progress("=" * 50, 1.0)
            
            return True
            
        except Exception as e:
            logger.exception(f"Homing error: {e}")
            self.state = HomingState.FAILED
            return False
    
    def enable_all_motors(self) -> bool:
        """Enable all motors."""
        success = True
        for motor_id in range(1, 7):
            if not self.servos[motor_id].enable():
                logger.error(f"Failed to enable motor {motor_id}")
                success = False
        return success
    
    def disable_all_motors(self) -> bool:
        """Disable all motors."""
        success = True
        for motor_id in range(1, 7):
            if not self.servos[motor_id].disable():
                logger.error(f"Failed to disable motor {motor_id}")
                success = False
        return success
    
    def emergency_stop_all(self) -> None:
        """Emergency stop all motors."""
        logger.warning("EMERGENCY STOP - All motors")
        for motor_id in range(1, 7):
            try:
                self.servos[motor_id].emergency_stop()
            except Exception as e:
                logger.error(f"Motor {motor_id}: Emergency stop failed: {e}")


# Convenience function for standalone use
def home_robot(
    can_device: str = '/dev/ttyACM0',
    set_zero_only: bool = False,
    homing_speed: int = 300
) -> bool:
    """
    Convenience function to home the Arctos robot.
    
    Args:
        can_device: CAN device path
        set_zero_only: If True, just set current position as zero
        homing_speed: Homing speed in RPM
        
    Returns:
        True if successful
    """
    from .can_interface import CanInterface
    
    # Create config with custom speed
    config = ArctosHomingConfig()
    config.params.homing_speed = homing_speed
    
    # Update all joint configs with new speed
    for joint_config in config.joint_configs.values():
        joint_config.speed = homing_speed
    config.wrist_b_config.speed = homing_speed
    config.wrist_c_config.speed = homing_speed
    
    with CanInterface(can_device) as can:
        homing = ArctosHoming(can, config)
        
        if set_zero_only:
            return homing.set_zero_all()
        else:
            return homing.home_all()


# CLI entry point
if __name__ == '__main__':
    import sys
    import argparse
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    parser = argparse.ArgumentParser(description='Arctos Robot Homing')
    parser.add_argument('--device', default='/dev/ttyACM0', help='CAN device path')
    parser.add_argument('--zero', action='store_true', help='Set current position as zero')
    parser.add_argument('--speed', type=int, default=300, help='Homing speed in RPM')
    parser.add_argument('--check-limits', action='store_true', help='Only check limit switches')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    try:
        if args.check_limits:
            # Just check limit switches
            from .can_interface import CanInterface
            with CanInterface(args.device) as can:
                homing = ArctosHoming(can)
                limits = homing.check_all_limit_switches()
                print("\nLimit Switch Status:")
                for motor_id, at_limit in limits.items():
                    status = "AT LIMIT!" if at_limit else "Clear" if at_limit is False else "Unknown"
                    print(f"  Motor {motor_id}: {status}")
            sys.exit(0)
        
        success = home_robot(
            args.device,
            set_zero_only=args.zero,
            homing_speed=args.speed
        )
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\nHoming interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
