#!/usr/bin/env python3
"""
CAN Timing Audit for Arctos Robot.

Measures CAN round-trip latencies for encoder reads and position writes
across all motors to determine the safe ros2_control update rate.

Usage:
    python3 can_timing_audit.py [--cycles 100] [--device can0]
"""

import sys
import time
import argparse
import statistics
import logging

sys.path.insert(0, '/home/pc/ros2_ws/src/arctos_hardware')

from arctos_hardware.can_interface import CanInterface, default_can_device
from arctos_hardware.mks_servo import MksServo

logging.basicConfig(level=logging.WARNING)


def measure_read_latencies(can: CanInterface, motor_ids: list[int], cycles: int):
    """Measure per-motor and full-cycle encoder read latencies."""
    servos = {mid: MksServo(can, mid) for mid in motor_ids}

    per_motor: dict[int, list[float]] = {mid: [] for mid in motor_ids}
    full_cycle: list[float] = []
    failures: dict[int, int] = {mid: 0 for mid in motor_ids}

    print(f"\n--- Encoder Read Latency ({cycles} cycles, {len(motor_ids)} motors) ---\n")

    for c in range(cycles):
        cycle_start = time.perf_counter()

        for mid in motor_ids:
            t0 = time.perf_counter()
            result = servos[mid].read_encoder_value()
            t1 = time.perf_counter()

            dt_ms = (t1 - t0) * 1000.0
            if result is not None:
                per_motor[mid].append(dt_ms)
            else:
                failures[mid] += 1

        cycle_end = time.perf_counter()
        full_cycle.append((cycle_end - cycle_start) * 1000.0)

        if (c + 1) % 20 == 0:
            print(f"  cycle {c + 1}/{cycles}: last full cycle = {full_cycle[-1]:.2f} ms")

    print("\n=== Per-Motor Encoder Read Results ===\n")
    for mid in motor_ids:
        times = per_motor[mid]
        if not times:
            print(f"  Motor {mid}: ALL FAILED ({failures[mid]} failures)")
            continue
        print(
            f"  Motor {mid}: "
            f"min={min(times):.2f} ms, "
            f"median={statistics.median(times):.2f} ms, "
            f"mean={statistics.mean(times):.2f} ms, "
            f"max={max(times):.2f} ms, "
            f"stdev={statistics.stdev(times):.2f} ms, "
            f"failures={failures[mid]}"
        )

    print("\n=== Full Cycle (all motors sequential) ===\n")
    print(
        f"  min={min(full_cycle):.2f} ms, "
        f"median={statistics.median(full_cycle):.2f} ms, "
        f"mean={statistics.mean(full_cycle):.2f} ms, "
        f"max={max(full_cycle):.2f} ms, "
        f"stdev={statistics.stdev(full_cycle):.2f} ms"
    )

    return per_motor, full_cycle


def measure_write_latencies(can: CanInterface, motor_ids: list[int], cycles: int):
    """Measure position command (read current + write same) latencies."""
    servos = {mid: MksServo(can, mid) for mid in motor_ids}

    current_pos = {}
    for mid in motor_ids:
        pos = servos[mid].read_encoder_value()
        current_pos[mid] = pos if pos is not None else 0

    full_cycle: list[float] = []

    print(f"\n--- Read+Write Cycle Latency ({cycles} cycles) ---\n")

    for c in range(cycles):
        cycle_start = time.perf_counter()

        for mid in motor_ids:
            servos[mid].read_encoder_value()

        for mid in motor_ids:
            servos[mid].move_to_position(
                current_pos[mid], speed=100, acceleration=50
            )

        cycle_end = time.perf_counter()
        full_cycle.append((cycle_end - cycle_start) * 1000.0)

        if (c + 1) % 20 == 0:
            print(f"  cycle {c + 1}/{cycles}: last = {full_cycle[-1]:.2f} ms")

    print("\n=== Read+Write Full Cycle ===\n")
    print(
        f"  min={min(full_cycle):.2f} ms, "
        f"median={statistics.median(full_cycle):.2f} ms, "
        f"mean={statistics.mean(full_cycle):.2f} ms, "
        f"max={max(full_cycle):.2f} ms, "
        f"stdev={statistics.stdev(full_cycle):.2f} ms"
    )

    return full_cycle


def recommend_rate(read_write_cycle: list[float]):
    """Recommend a safe controller update rate based on measured timings."""
    p95 = sorted(read_write_cycle)[int(len(read_write_cycle) * 0.95)]
    p99 = sorted(read_write_cycle)[int(len(read_write_cycle) * 0.99)]
    worst = max(read_write_cycle)

    safe_period_ms = p95 * 1.5
    safe_hz = 1000.0 / safe_period_ms

    conservative_period_ms = worst * 1.5
    conservative_hz = 1000.0 / conservative_period_ms

    print("\n=== Recommended controller_manager.update_rate ===\n")
    print(f"  P95 cycle time:        {p95:.2f} ms")
    print(f"  P99 cycle time:        {p99:.2f} ms")
    print(f"  Worst cycle time:      {worst:.2f} ms")
    print(f"  Safe rate (1.5x P95):  {safe_hz:.1f} Hz  ({safe_period_ms:.1f} ms period)")
    print(f"  Conservative (1.5x worst): {conservative_hz:.1f} Hz  ({conservative_period_ms:.1f} ms period)")
    print(f"\n  Suggested update_rate: {int(conservative_hz)} Hz")
    print()

    return int(conservative_hz)


def main():
    parser = argparse.ArgumentParser(description="CAN timing audit for Arctos robot")
    parser.add_argument("--cycles", type=int, default=100, help="Number of measurement cycles")
    parser.add_argument("--device", type=str, default=default_can_device(), help="CAN device")
    parser.add_argument("--motors", type=str, default="1,2,3,4,5,6", help="Comma-separated motor IDs")
    args = parser.parse_args()

    motor_ids = [int(x) for x in args.motors.split(",")]

    print(f"CAN Timing Audit")
    print(f"  Device: {args.device}")
    print(f"  Motors: {motor_ids}")
    print(f"  Cycles: {args.cycles}")

    can = CanInterface(device=args.device)
    can.connect()

    try:
        measure_read_latencies(can, motor_ids, args.cycles)
        rw_cycle = measure_write_latencies(can, motor_ids, args.cycles)
        recommend_rate(rw_cycle)
    finally:
        can.disconnect()


if __name__ == "__main__":
    main()
