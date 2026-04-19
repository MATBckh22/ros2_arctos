#!/usr/bin/env python3
"""Evaluate detection error against CSV ground truth."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np

from maripad_perception.perception.book_detector import BookDetector
from maripad_perception.perception.geometry import circular_distance_deg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_dir", type=Path, help="Root directory of saved frame folders")
    parser.add_argument("ground_truth_csv", type=Path, help="CSV with frame_id,x_mm,y_mm,z_mm,yaw_deg")
    parser.add_argument("--config", type=Path, default=None, help="Path to perception_config.yaml")
    return parser.parse_args()


def load_frame_pair(frame_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    rgb_bgr = cv2.imread(str(frame_dir / "rgb.png"), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(frame_dir / "depth.png"), cv2.IMREAD_UNCHANGED)
    if rgb_bgr is None or depth is None:
        raise FileNotFoundError(f"Incomplete frame pair in {frame_dir}")
    return cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB), depth


def main() -> None:
    args = parse_args()
    detector = BookDetector(config_path=args.config)

    rows = []
    with args.ground_truth_csv.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)

    if not rows:
        raise RuntimeError("Ground truth CSV is empty")

    translation_errors_mm = []
    yaw_errors_deg = []

    for row in rows:
        frame_dir = args.dataset_dir / row["frame_id"]
        rgb, depth = load_frame_pair(frame_dir)
        result = detector.detect_book_pose(rgb, depth)
        if result is None:
            print(f"{row['frame_id']}: detection failed")
            continue

        gt_position_m = np.array(
            [
                float(row["x_mm"]) / 1000.0,
                float(row["y_mm"]) / 1000.0,
                float(row["z_mm"]) / 1000.0,
            ],
            dtype=np.float64,
        )
        pred_position_m = np.array(
            [result.book_pose.x, result.book_pose.y, result.book_pose.z],
            dtype=np.float64,
        )
        translation_error_mm = float(
            np.linalg.norm(pred_position_m - gt_position_m) * 1000.0
        )
        yaw_error_deg = float(
            circular_distance_deg(result.book_pose.yaw, float(row["yaw_deg"]))
        )
        translation_errors_mm.append(translation_error_mm)
        yaw_errors_deg.append(yaw_error_deg)
        print(
            f"{row['frame_id']}: translation_error_mm={translation_error_mm:.3f} "
            f"yaw_error_deg={yaw_error_deg:.3f}"
        )

    if not translation_errors_mm:
        raise RuntimeError("No successful detections for evaluation")

    mean_translation = float(np.mean(translation_errors_mm))
    mean_yaw = float(np.mean(yaw_errors_deg))
    max_translation = float(np.max(translation_errors_mm))
    max_yaw = float(np.max(yaw_errors_deg))

    print("---")
    print(f"mean_translation_error_mm={mean_translation:.3f}")
    print(f"mean_yaw_error_deg={mean_yaw:.3f}")
    print(f"max_translation_error_mm={max_translation:.3f}")
    print(f"max_yaw_error_deg={max_yaw:.3f}")
    if mean_translation >= 5.0 or mean_yaw >= 2.0:
        raise SystemExit("Evaluation failed target thresholds")


if __name__ == "__main__":
    main()
