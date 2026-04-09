"""
Differential Wrist Kinematics for Arctos Robot.

Handles the coupled kinematics of joints 5 and 6 which use a differential
mechanism with two motors (B-axis and C-axis).

Mathematical Model:
    Forward (motors → joints):
        joint5 = (B + C) / 2
        joint6 = (B - C) / 2
    
    Inverse (joints → motors):
        B = joint5 + joint6
        C = joint5 - joint6

Note: Based on community feedback, the actual implementation uses:
    B = -joint6 + joint5
    C = -joint6 - joint5
This accounts for the specific mechanical configuration of the Arctos wrist.
"""

import math
import logging
from dataclasses import dataclass
from typing import Tuple

logger = logging.getLogger(__name__)


@dataclass
class WristConfig:
    """Configuration for differential wrist."""
    gear_ratio_j5: float = 67.82    # Gear ratio for joint 5 / B axis
    gear_ratio_j6: float = 67.82    # Gear ratio for joint 6 / C axis
    encoder_resolution: int = 16384  # Encoder counts per motor revolution
    joint5_direction: int = 1        # Direction multiplier for joint 5
    joint6_direction: int = 1        # Direction multiplier for joint 6
    
    # Legacy shared rad->encoder constant used in older experiments.
    # Runtime conversion now uses the per-motor encoder factors derived
    # from encoder_resolution * gear_ratio / (2*pi).
    joint_ratio: float = 210000.0


class DifferentialWrist:
    """
    Differential wrist kinematics handler.
    
    The Arctos robot uses a differential mechanism for the wrist where
    two motors (B and C) are coupled to drive joints 5 (pitch) and 6 (roll).
    
    This class handles the conversion between joint space and motor space
    for both position commands and encoder feedback.
    
    Attributes:
        config: WristConfig with gear ratios and parameters
    """
    
    def __init__(self, config: WristConfig = None):
        """
        Initialize differential wrist handler.
        
        Args:
            config: WristConfig instance, or None for defaults
        """
        self.config = config or WristConfig()
        
        # Cache computed values
        self._j5_factor = (
            self.config.encoder_resolution * 
            self.config.gear_ratio_j5 / 
            (2 * math.pi)
        )
        self._j6_factor = (
            self.config.encoder_resolution * 
            self.config.gear_ratio_j6 / 
            (2 * math.pi)
        )
        
        logger.debug(
            f"DifferentialWrist initialized: J5 factor={self._j5_factor:.2f}, "
            f"J6 factor={self._j6_factor:.2f}"
        )
    
    def joints_to_motors(
        self,
        joint5_rad: float,
        joint6_rad: float
    ) -> Tuple[int, int]:
        """
        Convert joint angles to motor encoder positions.
        
        Uses the differential inverse kinematics:
            B = -joint6 + joint5
            C = -joint6 - joint5
        
        (Based on actual Arctos implementation from community)
        
        Args:
            joint5_rad: Joint 5 angle in radians
            joint6_rad: Joint 6 angle in radians
            
        Returns:
            Tuple of (motor_b_encoder, motor_c_encoder)
        """
        # Apply direction multipliers
        j5 = joint5_rad * self.config.joint5_direction
        j6 = joint6_rad * self.config.joint6_direction
        
        # Differential inverse kinematics with per-motor scaling.
        #
        # Preserve the established Arctos sign convention:
        #   B = -joint6 + joint5
        #   C = -joint6 - joint5
        #
        # but convert each joint contribution with the motor that actually
        # produces it instead of a shared hardcoded 210000 counts/rad factor.
        motor_b_encoder = int((j5 * self._j5_factor) - (j6 * self._j6_factor))
        motor_c_encoder = int((-j5 * self._j5_factor) - (j6 * self._j6_factor))
        
        logger.debug(
            f"joints_to_motors: J5={math.degrees(joint5_rad):.2f}°, "
            f"J6={math.degrees(joint6_rad):.2f}° → B={motor_b_encoder}, C={motor_c_encoder}"
        )
        
        return motor_b_encoder, motor_c_encoder
    
    def motors_to_joints(
        self,
        motor_b_encoder: int,
        motor_c_encoder: int
    ) -> Tuple[float, float]:
        """
        Convert motor encoder positions to joint angles.
        
        Uses the differential forward kinematics:
            joint5 = (B - C) / 2
            joint6 = -(B + C) / 2
        
        (Inverse of the joints_to_motors transformation)
        
        Args:
            motor_b_encoder: Motor B encoder value
            motor_c_encoder: Motor C encoder value
            
        Returns:
            Tuple of (joint5_rad, joint6_rad)
        """
        # Differential forward kinematics with per-motor scaling.
        #
        # From:
        #   B_enc =  j5 * j5_factor - j6 * j6_factor
        #   C_enc = -j5 * j5_factor - j6 * j6_factor
        #
        # Therefore:
        #   j5 = (B_enc - C_enc) / (2 * j5_factor)
        #   j6 = -(B_enc + C_enc) / (2 * j6_factor)
        j5 = (motor_b_encoder - motor_c_encoder) / (2 * self._j5_factor)
        j6 = -(motor_b_encoder + motor_c_encoder) / (2 * self._j6_factor)
        
        # Apply inverse direction multipliers
        joint5_rad = j5 / self.config.joint5_direction
        joint6_rad = j6 / self.config.joint6_direction
        
        logger.debug(
            f"motors_to_joints: B={motor_b_encoder}, C={motor_c_encoder} → "
            f"J5={math.degrees(joint5_rad):.2f}°, J6={math.degrees(joint6_rad):.2f}°"
        )
        
        return joint5_rad, joint6_rad
    
    def joint_velocities_to_motor_velocities(
        self,
        joint5_vel: float,
        joint6_vel: float
    ) -> Tuple[float, float]:
        """
        Convert joint velocities to motor velocities.
        
        The velocity transformation uses the same Jacobian as position.
        
        Args:
            joint5_vel: Joint 5 velocity in rad/s
            joint6_vel: Joint 6 velocity in rad/s
            
        Returns:
            Tuple of (motor_b_vel, motor_c_vel) in rad/s
        """
        j5 = joint5_vel * self.config.joint5_direction
        j6 = joint6_vel * self.config.joint6_direction
        
        motor_b_vel = (j5 * self.config.gear_ratio_j5) - (j6 * self.config.gear_ratio_j6)
        motor_c_vel = (-j5 * self.config.gear_ratio_j5) - (j6 * self.config.gear_ratio_j6)
        
        return motor_b_vel, motor_c_vel
    
    def motor_velocities_to_joint_velocities(
        self,
        motor_b_vel: float,
        motor_c_vel: float
    ) -> Tuple[float, float]:
        """
        Convert motor velocities to joint velocities.
        
        Args:
            motor_b_vel: Motor B velocity in rad/s
            motor_c_vel: Motor C velocity in rad/s
            
        Returns:
            Tuple of (joint5_vel, joint6_vel) in rad/s
        """
        j5 = (motor_b_vel - motor_c_vel) / (2 * self.config.gear_ratio_j5)
        j6 = -(motor_b_vel + motor_c_vel) / (2 * self.config.gear_ratio_j6)
        
        joint5_vel = j5 / self.config.joint5_direction
        joint6_vel = j6 / self.config.joint6_direction
        
        return joint5_vel, joint6_vel
    
    def calculate_coordinated_speeds(
        self,
        target_j5: float,
        target_j6: float,
        current_j5: float,
        current_j6: float,
        max_speed: int = 500
    ) -> Tuple[int, int]:
        """
        Calculate coordinated motor speeds for smooth wrist motion.
        
        Ensures both motors arrive at target at the same time by
        scaling speeds proportionally to distance.
        
        Args:
            target_j5: Target joint 5 angle in radians
            target_j6: Target joint 6 angle in radians
            current_j5: Current joint 5 angle in radians
            current_j6: Current joint 6 angle in radians
            max_speed: Maximum motor speed in RPM
            
        Returns:
            Tuple of (motor_b_speed, motor_c_speed) in RPM
        """
        # Calculate motor positions
        target_b, target_c = self.joints_to_motors(target_j5, target_j6)
        current_b, current_c = self.joints_to_motors(current_j5, current_j6)
        
        # Calculate deltas
        delta_b = abs(target_b - current_b)
        delta_c = abs(target_c - current_c)
        
        # Scale speeds proportionally
        max_delta = max(delta_b, delta_c)
        
        if max_delta > 0:
            scale_b = delta_b / max_delta
            scale_c = delta_c / max_delta
        else:
            scale_b = scale_c = 1.0
        
        speed_b = max(1, int(max_speed * scale_b))
        speed_c = max(1, int(max_speed * scale_c))
        
        return speed_b, speed_c
    
    def validate_joint_limits(
        self,
        joint5_rad: float,
        joint6_rad: float,
        j5_limits: Tuple[float, float] = (-math.pi/2, math.pi/2),
        j6_limits: Tuple[float, float] = (-math.pi, math.pi)
    ) -> bool:
        """
        Validate joint angles are within limits.
        
        Args:
            joint5_rad: Joint 5 angle in radians
            joint6_rad: Joint 6 angle in radians
            j5_limits: Tuple of (min, max) for joint 5
            j6_limits: Tuple of (min, max) for joint 6
            
        Returns:
            True if within limits
        """
        if not (j5_limits[0] <= joint5_rad <= j5_limits[1]):
            logger.warning(
                f"Joint 5 out of limits: {math.degrees(joint5_rad):.2f}° "
                f"not in [{math.degrees(j5_limits[0]):.2f}, {math.degrees(j5_limits[1]):.2f}]"
            )
            return False
            
        if not (j6_limits[0] <= joint6_rad <= j6_limits[1]):
            logger.warning(
                f"Joint 6 out of limits: {math.degrees(joint6_rad):.2f}° "
                f"not in [{math.degrees(j6_limits[0]):.2f}, {math.degrees(j6_limits[1]):.2f}]"
            )
            return False
            
        return True


# Convenience function for simple usage
def create_default_wrist() -> DifferentialWrist:
    """Create a DifferentialWrist with default Arctos configuration."""
    return DifferentialWrist(WristConfig())


# Self-test
if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    
    wrist = create_default_wrist()
    
    # Test round-trip conversion
    print("Testing differential wrist kinematics...")
    
    test_cases = [
        (0.0, 0.0),
        (math.radians(45), 0.0),
        (0.0, math.radians(45)),
        (math.radians(45), math.radians(30)),
        (math.radians(-30), math.radians(60)),
    ]
    
    for j5, j6 in test_cases:
        # Forward
        b, c = wrist.joints_to_motors(j5, j6)
        
        # Inverse
        j5_back, j6_back = wrist.motors_to_joints(b, c)
        
        # Check error
        j5_err = abs(j5 - j5_back)
        j6_err = abs(j6 - j6_back)
        
        status = "✓" if max(j5_err, j6_err) < 1e-10 else "✗"
        
        print(f"{status} J5={math.degrees(j5):6.1f}°, J6={math.degrees(j6):6.1f}° → "
              f"B={b:8d}, C={c:8d} → "
              f"J5'={math.degrees(j5_back):6.1f}°, J6'={math.degrees(j6_back):6.1f}°")
    
    print("\nDifferential wrist test complete.")
