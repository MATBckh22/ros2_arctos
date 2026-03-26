#!/usr/bin/env python3
"""Manual recovery jog for a single MKS joint.

Use this when a joint needs to be rotated back slowly after a bad homing run.
It does not run any homing logic. It simply enables the motor, starts speed
mode in the requested direction, and stops when interrupted or when an optional
duration elapses.
"""

import argparse
import logging
import signal
import sys
import time

from arctos_hardware.can_interface import CanInterface, default_can_device
from arctos_hardware.mks_servo import Direction, MksServo


LOGGER = logging.getLogger("jog_joint_recovery")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Jog a joint slowly for recovery.")
    parser.add_argument("--motor-id", type=int, default=4, help="Motor CAN ID to jog")
    parser.add_argument(
        "--direction",
        choices=("cw", "ccw"),
        default="ccw",
        help="Motor direction for the recovery jog",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=10,
        help="Jog speed in RPM",
    )
    parser.add_argument(
        "--acceleration",
        type=int,
        default=50,
        help="Acceleration for speed mode",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional jog duration in seconds. If omitted, run until Ctrl-C.",
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
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s - %(message)s")

    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True
        LOGGER.info("Stop requested, stopping motor...")

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    direction = Direction.CW if args.direction == "cw" else Direction.CCW

    with CanInterface(args.device, args.bitrate) as can:
        servo = MksServo(can, args.motor_id)

        before = servo.read_encoder_value()
        LOGGER.info(
            "Motor %s encoder before jog: %s",
            args.motor_id,
            before,
        )

        if not servo.enable():
            LOGGER.error("Motor %s did not acknowledge enable", args.motor_id)
            return 1

        time.sleep(0.1)

        if not servo.run_speed(direction, args.speed, args.acceleration):
            LOGGER.error("Motor %s failed to start speed mode", args.motor_id)
            return 1

        LOGGER.info(
            "Jogging motor %s %s at %s RPM%s",
            args.motor_id,
            args.direction.upper(),
            args.speed,
            f" for {args.duration:.1f}s" if args.duration is not None else " until Ctrl-C",
        )

        start_time = time.time()
        while not stop_requested:
            if args.duration is not None and (time.time() - start_time) >= args.duration:
                break
            time.sleep(0.05)

        servo.stop(acceleration=args.acceleration)
        time.sleep(0.2)

        if servo.is_running():
            LOGGER.warning("Motor still reports running, issuing emergency stop")
            servo.emergency_stop()
            time.sleep(0.2)

        after = servo.read_encoder_value()
        LOGGER.info(
            "Motor %s encoder after jog: %s",
            args.motor_id,
            after,
        )
        if before is not None and after is not None:
            LOGGER.info("Encoder delta: %s", after - before)

    return 0


if __name__ == "__main__":
    sys.exit(main())
