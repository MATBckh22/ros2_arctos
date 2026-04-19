#!/usr/bin/env python3
"""Visualize book detection overlays for a saved frame pair."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from maripad_perception.perception.book_detector import BookDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame_dir", type=Path, help="Directory containing rgb.png and depth.png")
    parser.add_argument("--config", type=Path, default=None, help="Path to perception_config.yaml")
    parser.add_argument("--output", type=Path, default=None, help="Output image path")
    return parser.parse_args()


def load_frame_pair(frame_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    rgb_bgr = cv2.imread(str(frame_dir / "rgb.png"), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(frame_dir / "depth.png"), cv2.IMREAD_UNCHANGED)
    if rgb_bgr is None:
        raise FileNotFoundError(f"Missing RGB image at {frame_dir / 'rgb.png'}")
    if depth is None:
        raise FileNotFoundError(f"Missing depth image at {frame_dir / 'depth.png'}")
    rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
    return rgb, depth


def main() -> None:
    args = parse_args()
    detector = BookDetector(config_path=args.config)
    rgb, depth = load_frame_pair(args.frame_dir)
    result, artifacts = detector.process_frame(rgb, depth, return_artifacts=True)

    overlay = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if artifacts is None:
        raise RuntimeError("Expected detection artifacts for visualization")

    polygon = detector.valid_zone_polygon.astype(np.int32)
    cv2.polylines(overlay, [polygon], isClosed=True, color=(255, 255, 0), thickness=2)

    if artifacts.mask is not None:
        mask_overlay = np.zeros_like(overlay)
        mask_overlay[..., 1] = artifacts.mask
        overlay = cv2.addWeighted(overlay, 1.0, mask_overlay, 0.3, 0.0)

    if artifacts.contour is not None:
        cv2.drawContours(overlay, [artifacts.contour], -1, (0, 255, 0), 2)

    if artifacts.rect is not None:
        box = cv2.boxPoints(artifacts.rect).astype(np.int32)
        cv2.polylines(overlay, [box], isClosed=True, color=(0, 0, 255), thickness=2)

    if artifacts.center_px is not None:
        center = tuple(int(round(value)) for value in artifacts.center_px)
        cv2.circle(overlay, center, 5, (255, 0, 0), -1)

    if result is not None:
        label = (
            f"yaw={result.book_pose.yaw:.2f} deg "
            f"conf={result.confidence:.2f}"
        )
        cv2.putText(
            overlay,
            label,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        grip_vector = np.array(
            [
                result.grip_target.x - result.book_pose.x,
                result.grip_target.y - result.book_pose.y,
            ],
            dtype=np.float64,
        )
        image_scale_px = 200.0
        if artifacts.center_px is not None:
            grip_px = (
                int(round(artifacts.center_px[0] + (grip_vector[0] * image_scale_px))),
                int(round(artifacts.center_px[1] + (grip_vector[1] * image_scale_px))),
            )
            cv2.drawMarker(
                overlay,
                grip_px,
                (0, 255, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=18,
                thickness=2,
            )
            artifacts.grip_target_px = grip_px
    else:
        cv2.putText(
            overlay,
            f"Detection rejected: {artifacts.reason or 'unknown'}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    output_path = args.output or (args.frame_dir / "visualization.png")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), overlay)
    print(f"Saved overlay to {output_path}")


if __name__ == "__main__":
    main()
