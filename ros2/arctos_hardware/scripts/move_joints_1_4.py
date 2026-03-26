#!/usr/bin/env python3
"""Move joints 1-4 by a fixed relative angle.

This is a simple hardware test helper. It reads the current joint positions,
adds the requested delta to joints 1-4, leaves joints 5-6 unchanged, and sends
one position command through the normal Arctos controller.
"""

import argparse
import logging
import math
import sys
import time

from arctos_hardware.arctos_controller import ArctosConfig, ArctosController
from arctos_hardware.can_interface import default_can_device


LOGGER = logging.getLogger("move_joints_1_4")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move joints 1-4 by a fixed delta.")
    parser.add_argument(
        "--degrees",
        type=float,
        default=30.0,
        help="Relative motion to apply to joints 1-4 in degrees",
    )
    parser.add_argument(
        "--rpm",
        type=int,
        default=50,
        help="Motor speed in RPM for joints 1-4",
    )
    parser.add_argument(
        "--acceleration",
        type=int,
        default=30,
        help="Motor acceleration for joints 1-4",
    )
    parser.add_argument(
        "--device",
        default=default_can_device(),
        help="CAN device, e.g. can0",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=500000,
        help="CAN bitrate",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=30.0,
        help="How long to wait for joints to stop",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s - %(message)s")

    config = ArctosConfig(
        can_device=args.device,
        can_bitrate=args.bitrate,
        active_motor_ids=[1, 2, 3, 4],
    )
    controller = ArctosController(config)

    if not controller.connect():
        LOGGER.error("Failed to connect to controller")
        return 1

    try:
        if not controller.enable_motors():
            LOGGER.error("Failed to enable motors")
            return 1

        current_positions = controller.read_joint_positions()
        target_positions = list(current_positions)
        delta_rad = math.radians(args.degrees)

        for joint_index in range(4):
            target_positions[joint_index] += delta_rad

        speeds = [args.rpm, args.rpm, args.rpm, args.rpm, 0, 0]
        accelerations = [args.acceleration, args.acceleration, args.acceleration, args.acceleration, 0, 0]

        LOGGER.info(
            "Current joints 1-4 (deg): %s",
            [round(math.degrees(current_positions[i]), 2) for i in range(4)],
        )
        LOGGER.info(
            "Target joints 1-4 (deg): %s",
            [round(math.degrees(target_positions[i]), 2) for i in range(4)],
        )
        LOGGER.info(
            "Commanding joints 1-4 by %+0.2f deg at %d RPM",
            args.degrees,
            args.rpm,
        )

        if not controller.move_to_positions(
            target_positions,
            speeds=speeds,
            accelerations=accelerations,
        ):
            LOGGER.error("Move command failed")
            return 1

        start_time = time.time()
        while time.time() - start_time < args.wait_timeout:
            if not any(
                servo is not None and servo.is_running()
                for servo in controller.servos[:4]
            ):
                break
            time.sleep(0.1)
        else:
            LOGGER.warning("Timeout waiting for joints 1-4 to stop")

        final_positions = controller.read_joint_positions()
        LOGGER.info(
            "Final joints 1-4 (deg): %s",
            [round(math.degrees(final_positions[i]), 2) for i in range(4)],
        )

    finally:
        controller.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
