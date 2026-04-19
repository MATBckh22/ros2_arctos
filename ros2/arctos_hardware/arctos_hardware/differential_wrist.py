"""
Differential wrist kinematics for the Arctos robot.

Joints 5 and 6 are produced by two coupled wrist motors:
    Motor B (CAN ID 5)
    Motor C (CAN ID 6)

The live hardware stack uses the legacy Arctos wrist convention observed on the
real robot:

    B = J5 - J6
    C = -(J5 + J6)

with an effective per-joint coupling ratio of 33.91. This is half of the
physical motor gearbox ratio (67.82), because each logical wrist joint is
reconstructed from the average / difference of the two motor motions.

Inverse kinematics (joints -> motor encoders):
    B_enc = (+R5 * j5 - R6 * j6) * (encoder_resolution / 2pi) * dir_B
    C_enc = (-R5 * j5 - R6 * j6) * (encoder_resolution / 2pi) * dir_C

Forward kinematics (motor encoders -> joints):
    b_rad = B_enc / ((encoder_resolution / 2pi) * dir_B)
    c_rad = C_enc / ((encoder_resolution / 2pi) * dir_C)
    j5 = (b_rad - c_rad) / (2 * R5)
    j6 = -(b_rad + c_rad) / (2 * R6)
"""

import math
import logging
from dataclasses import dataclass
from typing import Tuple

logger = logging.getLogger(__name__)


@dataclass
class WristConfig:
    """Configuration for differential wrist.

    gear_ratio_j5 / gear_ratio_j6 are the effective coupling ratios for the
    logical wrist joints. For this robot, the calibrated values used by the
    reference ROS2 hardware notes are 27.3375 / 10.0, but the live wrist
    has needed a slightly larger joint6 command scale to match physical
    motion on this robot.

    motor_b_direction / motor_c_direction flip the sign of the encoder
    target sent to each motor to account for physical mounting orientation.
    """
    gear_ratio_j5: float = 27.3375
    gear_ratio_j6: float = 15.0
    encoder_resolution: int = 16384
    motor_b_direction: int = 1
    motor_c_direction: int = 1


class DifferentialWrist:
    """Differential wrist kinematics handler.

    Converts between joint-space (j5 pitch, j6 roll) and motor-space
    (motor B encoder, motor C encoder) using the real-hardware Arctos
    wrist convention.
    """

    def __init__(self, config: WristConfig = None):
        self.config = config or WristConfig()

        self._enc_per_rad = self.config.encoder_resolution / (2 * math.pi)

        logger.debug(
            f"DifferentialWrist initialized: R5={self.config.gear_ratio_j5}, "
            f"R6={self.config.gear_ratio_j6}, "
            f"dir_B={self.config.motor_b_direction}, "
            f"dir_C={self.config.motor_c_direction}"
        )

    def joints_to_motors(
        self,
        joint5_rad: float,
        joint6_rad: float
    ) -> Tuple[int, int]:
        """Convert joint angles to motor encoder positions.

        B_enc = (+R5 * j5 - R6 * j6) * enc_per_rad * dir_B
        C_enc = (-R5 * j5 - R6 * j6) * enc_per_rad * dir_C
        """
        j5_geared = joint5_rad * self.config.gear_ratio_j5
        j6_geared = joint6_rad * self.config.gear_ratio_j6

        motor_b_raw = (+j5_geared - j6_geared) * self._enc_per_rad
        motor_c_raw = (-j5_geared - j6_geared) * self._enc_per_rad

        motor_b_encoder = int(motor_b_raw * self.config.motor_b_direction)
        motor_c_encoder = int(motor_c_raw * self.config.motor_c_direction)

        logger.debug(
            f"joints_to_motors: J5={math.degrees(joint5_rad):.2f}°, "
            f"J6={math.degrees(joint6_rad):.2f}° -> B={motor_b_encoder}, C={motor_c_encoder}"
        )

        return motor_b_encoder, motor_c_encoder

    def motors_to_joints(
        self,
        motor_b_encoder: int,
        motor_c_encoder: int
    ) -> Tuple[float, float]:
        """Convert motor encoder positions to joint angles.

        b_rad = enc_B / (enc_per_rad * dir_B)
        c_rad = enc_C / (enc_per_rad * dir_C)
        j5 = (b_rad - c_rad) / (2 * R5)
        j6 = -(b_rad + c_rad) / (2 * R6)
        """
        b_rad = motor_b_encoder / (self._enc_per_rad * self.config.motor_b_direction)
        c_rad = motor_c_encoder / (self._enc_per_rad * self.config.motor_c_direction)

        joint5_rad = (b_rad - c_rad) / (2 * self.config.gear_ratio_j5)
        joint6_rad = -(b_rad + c_rad) / (2 * self.config.gear_ratio_j6)

        logger.debug(
            f"motors_to_joints: B={motor_b_encoder}, C={motor_c_encoder} -> "
            f"J5={math.degrees(joint5_rad):.2f}°, J6={math.degrees(joint6_rad):.2f}°"
        )

        return joint5_rad, joint6_rad

    def joint_velocities_to_motor_velocities(
        self,
        joint5_vel: float,
        joint6_vel: float
    ) -> Tuple[float, float]:
        """Convert joint velocities to motor velocities (rad/s)."""
        j5g = joint5_vel * self.config.gear_ratio_j5
        j6g = joint6_vel * self.config.gear_ratio_j6

        motor_b_vel = (+j5g - j6g) * self.config.motor_b_direction
        motor_c_vel = (-j5g - j6g) * self.config.motor_c_direction

        return motor_b_vel, motor_c_vel

    def motor_velocities_to_joint_velocities(
        self,
        motor_b_vel: float,
        motor_c_vel: float
    ) -> Tuple[float, float]:
        """Convert motor velocities to joint velocities (rad/s)."""
        b = motor_b_vel / self.config.motor_b_direction
        c = motor_c_vel / self.config.motor_c_direction

        joint5_vel = (b - c) / (2 * self.config.gear_ratio_j5)
        joint6_vel = -(b + c) / (2 * self.config.gear_ratio_j6)

        return joint5_vel, joint6_vel

    def calculate_coordinated_speeds(
        self,
        target_j5: float,
        target_j6: float,
        current_j5: float,
        current_j6: float,
        max_speed: int = 500
    ) -> Tuple[int, int]:
        """Calculate coordinated motor speeds for smooth wrist motion."""
        target_b, target_c = self.joints_to_motors(target_j5, target_j6)
        current_b, current_c = self.joints_to_motors(current_j5, current_j6)

        delta_b = abs(target_b - current_b)
        delta_c = abs(target_c - current_c)
        max_delta = max(delta_b, delta_c)

        if max_delta > 0:
            scale_b = delta_b / max_delta
            scale_c = delta_c / max_delta
        else:
            scale_b = scale_c = 1.0

        return max(1, int(max_speed * scale_b)), max(1, int(max_speed * scale_c))

    def validate_joint_limits(
        self,
        joint5_rad: float,
        joint6_rad: float,
        j5_limits: Tuple[float, float] = (-math.pi/2, math.pi/2),
        j6_limits: Tuple[float, float] = (-math.pi, math.pi)
    ) -> bool:
        """Validate joint angles are within limits."""
        if not (j5_limits[0] <= joint5_rad <= j5_limits[1]):
            logger.warning(
                f"Joint 5 out of limits: {math.degrees(joint5_rad):.2f}deg "
                f"not in [{math.degrees(j5_limits[0]):.2f}, {math.degrees(j5_limits[1]):.2f}]"
            )
            return False
        if not (j6_limits[0] <= joint6_rad <= j6_limits[1]):
            logger.warning(
                f"Joint 6 out of limits: {math.degrees(joint6_rad):.2f}deg "
                f"not in [{math.degrees(j6_limits[0]):.2f}, {math.degrees(j6_limits[1]):.2f}]"
            )
            return False
        return True


def create_default_wrist() -> DifferentialWrist:
    """Create a DifferentialWrist with default Arctos configuration."""
    return DifferentialWrist(WristConfig())


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    wrist = create_default_wrist()
    print("Testing differential wrist kinematics (B=j5-j6, C=-(j5+j6))...")

    test_cases = [
        (0.0, 0.0),
        (math.radians(45), 0.0),
        (0.0, math.radians(45)),
        (math.radians(45), math.radians(30)),
        (math.radians(-30), math.radians(60)),
    ]

    for j5, j6 in test_cases:
        b, c = wrist.joints_to_motors(j5, j6)
        j5_back, j6_back = wrist.motors_to_joints(b, c)
        j5_err = abs(j5 - j5_back)
        j6_err = abs(j6 - j6_back)
        status = "OK" if max(j5_err, j6_err) < 0.001 else "FAIL"

        print(f"  {status} J5={math.degrees(j5):7.1f}deg, J6={math.degrees(j6):7.1f}deg "
              f"-> B={b:8d}, C={c:8d} "
              f"-> J5'={math.degrees(j5_back):7.1f}deg, J6'={math.degrees(j6_back):7.1f}deg")

    print("\nDone.")
