"""
MKS SERVO CAN Motor Driver.

Provides high-level interface to MKS SERVO42D and SERVO57D motor drivers
using the MKS CAN protocol (based on MKS SERVO42&57D_CAN User Manual V1.0.6).
"""

import time
import logging
from typing import Optional, Tuple
from enum import IntEnum
from dataclasses import dataclass

from .can_interface import CanInterface, CanError, CanTimeoutError

logger = logging.getLogger(__name__)


class MksCommand(IntEnum):
    """MKS SERVO CAN command codes."""
    # Read commands (0x30-0x3F)
    READ_ENCODER_CARRY = 0x30
    READ_ENCODER_ADDITION = 0x31
    READ_MOTOR_SPEED = 0x32
    READ_PULSES_RECEIVED = 0x33
    READ_IO_STATUS = 0x34
    READ_SHAFT_ERROR = 0x39
    READ_EN_STATUS = 0x3A
    READ_HOMING_STATUS = 0x3B
    RELEASE_PROTECTION = 0x3D
    READ_PROTECTION_STATE = 0x3E
    FACTORY_RESET = 0x3F
    
    # Configuration commands (0x80-0x9F)
    CALIBRATE_ENCODER = 0x80
    SET_WORK_MODE = 0x82
    SET_CURRENT = 0x83
    SET_SUBDIVISIONS = 0x84
    SET_EN_PIN = 0x85
    SET_DIRECTION = 0x86
    SET_AUTO_SCREEN_OFF = 0x87
    SET_STALL_PROTECTION = 0x88
    SET_INTERPOLATION = 0x89
    SET_CAN_BITRATE = 0x8A
    SET_CAN_ID = 0x8B
    SET_RESPONSE_MODE = 0x8C
    SET_GROUP_ID = 0x8D
    SET_KEY_LOCK = 0x8F
    SET_HOME = 0x90
    GO_HOME = 0x91
    SET_ZERO = 0x92
    SET_NO_LIMIT_HOME = 0x94
    SET_MODE0 = 0x9A
    SET_HOLDING_CURRENT = 0x9B
    SET_LIMIT_REMAP = 0x9E
    
    # Motor control commands (0xF0-0xFF)
    QUERY_STATUS = 0xF1
    ENABLE_MOTOR = 0xF3
    RELATIVE_MOTION_AXIS = 0xF4
    ABSOLUTE_MOTION_AXIS = 0xF5
    SPEED_MODE = 0xF6
    EMERGENCY_STOP = 0xF7
    RELATIVE_MOTION_PULSES = 0xFD
    ABSOLUTE_MOTION_PULSES = 0xFE
    SAVE_SPEED_PARAMS = 0xFF


class MotorStatus(IntEnum):
    """Motor status codes."""
    QUERY_FAILED = 0
    STOPPED = 1
    SPEED_UP = 2
    SPEED_DOWN = 3
    FULL_SPEED = 4
    HOMING = 5
    CALIBRATING = 6
    UNKNOWN = 255


class HomingStatus(IntEnum):
    """Homing status codes."""
    IN_PROGRESS = 0
    SUCCESS = 1
    FAILED = 2
    UNKNOWN = 255


class Direction(IntEnum):
    """Motor direction."""
    CCW = 0
    CW = 1


class EndStopLevel(IntEnum):
    """End stop trigger level."""
    LOW = 0
    HIGH = 1


@dataclass
class MotorState:
    """Current state of a motor."""
    encoder_value: int = 0
    speed_rpm: int = 0
    status: MotorStatus = MotorStatus.STOPPED
    enabled: bool = False
    position_error: int = 0


class MksServo:
    """
    MKS SERVO motor driver interface.
    
    Provides methods for controlling MKS SERVO42D and SERVO57D motors
    via CAN bus communication.
    
    Attributes:
        can: CAN interface instance
        motor_id: Motor CAN ID (1-6 for Arctos)
        timeout: Command timeout in seconds
    """
    
    MAX_SPEED = 3000  # RPM
    MAX_ACCELERATION = 255
    MAX_POSITION = 0x7FFFFF  # 24-bit signed max
    MIN_POSITION = -0x800000  # 24-bit signed min
    ENCODER_RESOLUTION = 16384  # Counts per revolution
    
    def __init__(
        self,
        can_interface: CanInterface,
        motor_id: int,
        timeout: float = 0.5
    ):
        """
        Initialize MKS SERVO driver.
        
        Args:
            can_interface: CAN interface for communication
            motor_id: Motor CAN ID (1-6)
            timeout: Command timeout in seconds
        """
        if not 1 <= motor_id <= 255:
            raise ValueError(f"Invalid motor ID: {motor_id}")
            
        self.can = can_interface
        self.motor_id = motor_id
        self.timeout = timeout
        self._enabled = False
        self._homing_status = HomingStatus.UNKNOWN
        
    def _send_command(
        self,
        opcode: int,
        data: list = None,
        expect_response: bool = True
    ) -> Optional[bytes]:
        """
        Send command to motor and optionally wait for response.
        
        Args:
            opcode: Command opcode
            data: Additional data bytes
            expect_response: Whether to wait for response
            
        Returns:
            Response bytes if expect_response, else None
        """
        cmd = [opcode]
        if data:
            cmd.extend(data)
            
        if expect_response:
            return self.can.send_and_receive(
                self.motor_id, cmd, timeout=self.timeout
            )
        else:
            self.can.send(self.motor_id, cmd)
            return None
    
    def _validate_speed(self, speed: int) -> int:
        """Clamp speed to valid range."""
        return max(0, min(speed, self.MAX_SPEED))
    
    def _validate_acceleration(self, accel: int) -> int:
        """Clamp acceleration to valid range."""
        return max(0, min(accel, self.MAX_ACCELERATION))
    
    def _validate_position(self, pos: int) -> int:
        """Clamp position to valid range."""
        return max(self.MIN_POSITION, min(pos, self.MAX_POSITION))
    
    # ===== Read Commands =====
    
    def read_encoder_value(self) -> Optional[int]:
        """
        Read encoder value in addition mode (absolute position).
        
        Returns:
            Encoder value (48-bit signed), or None on error
        """
        response = self._send_command(MksCommand.READ_ENCODER_ADDITION)
        
        if response and len(response) >= 7:
            # Extract 48-bit value from bytes 1-6
            value = int.from_bytes(response[1:7], byteorder='big', signed=True)
            return value
            
    
    def read_io_status(self) -> Optional[int]:
        """
        Read IO port status (limit switches, etc.).
        
        Returns:
            IO status byte where:
            - Bit 0: IN1 (limit switch 1) state
            - Bit 1: IN2 (limit switch 2) state
            Returns None on error.
        """
        response = self._send_command(MksCommand.READ_IO_STATUS)
        
        if response and len(response) >= 2:
            return response[1]
            
        return None

    def read_motor_speed(self) -> Optional[int]:
        """
        Read current motor speed in RPM.

        Returns:
            Signed motor speed in RPM, or None on error.
        """
        response = self._send_command(MksCommand.READ_MOTOR_SPEED)

        if response and len(response) >= 3:
            return int.from_bytes(response[1:3], byteorder='big', signed=True)

        return None
    
    def is_limit_switch_active(self, switch: int = 1) -> Optional[bool]:
        """
        Check if a limit switch is currently triggered.
        
        Args:
            switch: Limit switch number (1 or 2)
            
        Returns:
            True if limit switch is active, False if not, None on error
        """
        io_status = self.read_io_status()
        
        if io_status is None:
            return None
            
        # Limit switches are active low (triggered = 0)
        if switch == 1:
            return (io_status & 0x01) == 0
        elif switch == 2:
            return (io_status & 0x02) == 0
        else:
            return None

    def read_homing_status(self) -> HomingStatus:
        """
        Read go-back-to-zero status.
         
        Returns:
            Current homing status
        """
        response = self._send_command(MksCommand.READ_HOMING_STATUS)
        
        if response and len(response) >= 2:
            try:
                return HomingStatus(response[1])
            except ValueError:
                return HomingStatus.UNKNOWN

        return HomingStatus.UNKNOWN

    def query_status(self) -> MotorStatus:
        """
        Query current motor run state.

        Returns:
            MotorStatus enum value.
        """
        response = self._send_command(MksCommand.QUERY_STATUS)

        if response and len(response) >= 2:
            try:
                return MotorStatus(response[1])
            except ValueError:
                return MotorStatus.UNKNOWN

        return MotorStatus.UNKNOWN

    def is_running(self) -> bool:
        """Return True while the motor reports a motion state."""
        status = self.query_status()
        return status not in (MotorStatus.QUERY_FAILED, MotorStatus.STOPPED, MotorStatus.UNKNOWN)

    def read_position_error(self) -> Optional[int]:
        """
        Read shaft/position error if reported by the driver.

        Returns:
            Signed error value, or None on error.
        """
        response = self._send_command(MksCommand.READ_SHAFT_ERROR)

        if response and len(response) >= 3:
            return int.from_bytes(response[1:3], byteorder='big', signed=True)

        return None

    def get_state(self) -> MotorState:
        """
        Get comprehensive motor state.
        
        Returns:
            MotorState with current values
        """
        return MotorState(
            encoder_value=self.read_encoder_value() or 0,
            speed_rpm=self.read_motor_speed() or 0,
            status=self.query_status(),
            enabled=self._enabled,
            position_error=self.read_position_error() or 0
        )
    
    # ===== Control Commands =====
    
    def enable(self, enabled: bool = True) -> bool:
        """
        Enable or disable motor.
        
        Args:
            enabled: True to enable, False to disable
            
        Returns:
            True if command successful
        """
        response = self._send_command(
            MksCommand.ENABLE_MOTOR,
            [0x01 if enabled else 0x00]
        )
        
        if response and len(response) >= 2:
            success = response[1] == 1
            if success:
                self._enabled = enabled
            return success
            
        return False
    
    def disable(self) -> bool:
        """Disable motor."""
        return self.enable(False)
    
    def emergency_stop(self) -> bool:
        """
        Execute emergency stop.
        
        Warning: If motor is running above 1000 RPM, this may cause
        mechanical stress. Use decelerate() for graceful stopping.
        
        Returns:
            True if command successful
        """
        response = self._send_command(MksCommand.EMERGENCY_STOP)
        return response is not None
    
    def move_to_position(
        self,
        position: int,
        speed: int = 500,
        acceleration: int = 150,
        wait_for_ack: bool = True
    ) -> bool:
        """
        Move to absolute encoder position.
        
        This is the primary position control command using encoder axis units.
        
        Args:
            position: Target encoder position (24-bit signed)
            speed: Movement speed in RPM (0-3000)
            acceleration: Acceleration (0-255)
            wait_for_ack: If False, send command without waiting for motor ACK.
                          Useful in tight control loops where latency matters.
            
        Returns:
            True if command accepted (always True when wait_for_ack=False)
        """
        speed = self._validate_speed(speed)
        acceleration = self._validate_acceleration(acceleration)
        position = self._validate_position(position)
        
        # Build command: [speed_h, speed_l, accel, pos_h, pos_m, pos_l]
        cmd = [
            (speed >> 8) & 0xFF,
            speed & 0xFF,
            acceleration,
            (position >> 16) & 0xFF,
            (position >> 8) & 0xFF,
            position & 0xFF
        ]
        
        if not wait_for_ack:
            self._send_command(MksCommand.ABSOLUTE_MOTION_AXIS, cmd,
                               expect_response=False)
            return True

        response = self._send_command(MksCommand.ABSOLUTE_MOTION_AXIS, cmd)
        
        if response and len(response) >= 2:
            # 0 = running, 1 = complete, 2 = failed
            return response[1] in (0, 1)
            
        return False
    
    def move_relative(
        self,
        delta: int,
        speed: int = 500,
        acceleration: int = 150
    ) -> bool:
        """
        Move relative to current position.
        
        Args:
            delta: Relative movement in encoder counts (24-bit signed)
            speed: Movement speed in RPM
            acceleration: Acceleration
            
        Returns:
            True if command accepted
        """
        speed = self._validate_speed(speed)
        acceleration = self._validate_acceleration(acceleration)
        delta = self._validate_position(delta)
        
        cmd = [
            (speed >> 8) & 0xFF,
            speed & 0xFF,
            acceleration,
            (delta >> 16) & 0xFF,
            (delta >> 8) & 0xFF,
            delta & 0xFF
        ]
        
        response = self._send_command(MksCommand.RELATIVE_MOTION_AXIS, cmd)
        
        if response and len(response) >= 2:
            return response[1] in (0, 1)
            
        return False
    
    def run_speed(
        self,
        direction: Direction,
        speed: int,
        acceleration: int = 150
    ) -> bool:
        """
        Run motor in speed mode (continuous rotation).
        
        Args:
            direction: Rotation direction
            speed: Speed in RPM (0-3000)
            acceleration: Acceleration (0-255)
            
        Returns:
            True if command accepted
        """
        speed = self._validate_speed(speed)
        acceleration = self._validate_acceleration(acceleration)
        
        # Direction is encoded in high bit of speed high byte
        dir_bit = 0x80 if direction == Direction.CW else 0x00
        
        cmd = [
            dir_bit | ((speed >> 8) & 0x0F),
            speed & 0xFF,
            acceleration
        ]
        
        response = self._send_command(MksCommand.SPEED_MODE, cmd)
        return response is not None
    
    def stop(self, acceleration: int = 255) -> bool:
        """
        Gracefully stop motor with deceleration.
        
        Args:
            acceleration: Deceleration rate (higher = faster stop)
            
        Returns:
            True if command accepted
        """
        return self.run_speed(Direction.CCW, 0, acceleration)
    
    # ===== Homing Commands =====
    
    def configure_home(
        self,
        trigger_level: EndStopLevel = EndStopLevel.LOW,
        direction: Direction = Direction.CW,
        speed: int = 200,
        enable_limit: bool = True,
        home_mode: int = 0,
    ) -> bool:
        """
        Configure homing parameters.
        
        Args:
            trigger_level: End stop trigger level
            direction: Homing direction
            speed: Homing speed in RPM
            enable_limit: Enable limit switch
            home_mode: 0 = use limit switch, 1 = no-limit homing
            
        Returns:
            True if command successful
        """
        speed = self._validate_speed(speed)

        # MKS homing direction encoding differs from the speed-mode direction
        # bit used elsewhere: 0 = CW, 1 = CCW for SET_HOME.
        home_direction = 0x00 if direction == Direction.CW else 0x01
        
        cmd = [
            trigger_level.value,
            home_direction,
            (speed >> 8) & 0xFF,
            speed & 0xFF,
            0x01 if enable_limit else 0x00,
            home_mode,
        ]
        
        response = self._send_command(MksCommand.SET_HOME, cmd)
        
        if response and len(response) >= 2:
            return response[1] == 1
            
        return False
    
    def go_home(
        self,
        blocking: bool = False,
        timeout: float = 60.0,
        limit_switch_input: int = 1
    ) -> bool:
        """
        Execute homing sequence.
        
        Args:
            blocking: If True, wait for homing to complete
            timeout: Maximum time to wait if blocking
            
        Returns:
            True if homing started (non-blocking) or completed (blocking)
        """
        response = self._send_command(MksCommand.GO_HOME)
        
        if not response or len(response) < 2:
            return False
            
        status = response[1]
        if status == HomingStatus.IN_PROGRESS.value:
            self._homing_status = HomingStatus.IN_PROGRESS
            if not blocking:
                return True
        elif status == HomingStatus.SUCCESS.value:
            self._homing_status = HomingStatus.SUCCESS
            return True
        elif status == HomingStatus.FAILED.value:
            self._homing_status = HomingStatus.FAILED
            return False
        else:
            self._homing_status = HomingStatus.UNKNOWN
            logger.warning(
                f"Motor {self.motor_id}: Unexpected go_home status {status}"
            )
            return False
        
        # Wait for completion
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            homing_status = self.read_homing_status()

            motor_status = self.query_status()
            limit_active = self.is_limit_switch_active(limit_switch_input)

            if homing_status == HomingStatus.SUCCESS:
                self._homing_status = HomingStatus.SUCCESS
                return True

            # On this hardware/firmware, the 0x3B status can report failure even
            # though the axis physically reached the limit switch. Prefer the
            # observable end condition: motor stopped while the home switch is active.
            if motor_status == MotorStatus.STOPPED:
                if limit_active is True:
                    self._homing_status = HomingStatus.SUCCESS
                    return True
                if homing_status == HomingStatus.FAILED or limit_active is False:
                    self._homing_status = HomingStatus.FAILED
                    return False

            if homing_status == HomingStatus.FAILED and limit_active is False:
                self._homing_status = HomingStatus.FAILED
                return False
                
            time.sleep(0.1)
        
        logger.error(f"Motor {self.motor_id}: Homing timeout after {timeout}s")
        return False
    
    def set_zero(self) -> bool:
        """
        Set current position as zero.
        
        This sets the current encoder position as the zero reference
        without any motor movement.
        
        Returns:
            True if command successful
        """
        response = self._send_command(MksCommand.SET_ZERO)
        
        if response and len(response) >= 2:
            success = response[1] == 1
            if success:
                logger.info(f"Motor {self.motor_id}: Zero position set")
            return success
            
        return False
    
    def release_stall_protection(self) -> bool:
        """
        Release locked rotor protection.
        
        Call this after a stall condition has been resolved.
        
        Returns:
            True if command successful
        """
        response = self._send_command(MksCommand.RELEASE_PROTECTION)
        
        if response and len(response) >= 2:
            return response[1] == 1
            
        return False
    
    # ===== Configuration Commands =====
    
    def set_limit_port_remap(self, enabled: bool = True) -> bool:
        """
        Enable/disable limit port remapping.
        
        For 42D motors with single limit port, this allows using
        EN and DIR pins as additional limit inputs.
        
        Args:
            enabled: True to enable remapping
            
        Returns:
            True if command successful
        """
        response = self._send_command(
            MksCommand.SET_LIMIT_REMAP,
            [0x01 if enabled else 0x00]
        )
        
        if response and len(response) >= 2:
            return response[1] == 1
            
        return False

    def set_subdivisions(self, subdivisions: int) -> bool:
        """
        Set motor microstep subdivisions.

        The MKS absolute/relative axis commands are noticeably smoother with
        higher subdivision settings; the reference implementation suggests 64.
        """
        if subdivisions <= 0 or subdivisions > 255:
            raise ValueError("subdivisions must be in range 1..255")

        response = self._send_command(MksCommand.SET_SUBDIVISIONS, [subdivisions])
        if response and len(response) >= 2:
            return response[1] == 1
        return False

    def set_interpolation(self, enabled: bool = True) -> bool:
        """
        Enable/disable the driver's subdivision interpolation function.
        """
        response = self._send_command(
            MksCommand.SET_INTERPOLATION,
            [0x01 if enabled else 0x00]
        )
        if response and len(response) >= 2:
            return response[1] == 1
        return False
    
    def wait_for_idle(self, timeout: float = 30.0) -> bool:
        """
        Wait for motor to stop moving.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if motor stopped, False on timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if not self.is_running():
                return True
            time.sleep(0.05)
            
        return False
    
    def __repr__(self) -> str:
        return f"MksServo(id={self.motor_id}, enabled={self._enabled})"
