#!/usr/bin/env python3
"""Capture RGB-D frame pairs from an Intel RealSense camera."""

from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path

import cv2
import yaml

from maripad_perception.perception.book_detector import BookDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to perception_config.yaml",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Dataset output directory. Defaults to config value.",
    )
    parser.add_argument(
        "--frame-count",
        type=int,
        default=None,
        help="Number of frames to capture. Defaults to config value.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    detector = BookDetector(config_path=args.config)
    capture_config = detector.config["capture"]

    output_dir = args.output_dir or Path(
        capture_config["default_output_dir"]
    ).expanduser()
    frame_count = int(args.frame_count or capture_config["default_frame_count"])
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        import pyrealsense2 as rs
    except ImportError as exc:  # pragma: no cover - hardware dependent
        raise RuntimeError(
            "pyrealsense2 is required for live capture. Install it and retry."
        ) from exc

    pipeline = rs.pipeline()
    config = rs.config()
    intrinsics = detector.config["capture"]["stream"]
    width = int(intrinsics["width"])
    height = int(intrinsics["height"])
    fps = int(intrinsics["fps"])
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    pipeline.start(config)

    print(f"Capturing {frame_count} frame(s) into {output_dir}")
    print("Press Enter before each capture to save a frame pair.")
    try:
        for frame_index in range(frame_count):
            input(f"[{frame_index + 1}/{frame_count}] Position the book and press Enter...")
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                raise RuntimeError("Failed to acquire a synchronized RGB-D frame")

            frame_id = f"frame_{frame_index + 1:03d}"
            frame_dir = output_dir / frame_id
            frame_dir.mkdir(parents=True, exist_ok=True)

            rgb = cv2.cvtColor(
                cv2.UMat(color_frame.get_data()).get(), cv2.COLOR_BGR2RGB
            )
            depth = cv2.UMat(depth_frame.get_data()).get()

            rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(frame_dir / "rgb.png"), rgb_bgr)
            cv2.imwrite(str(frame_dir / "depth.png"), depth)

            metadata = {
                "frame_id": frame_id,
                "captured_at": dt.datetime.now(tz=dt.timezone.utc).isoformat(),
                "rgb_path": "rgb.png",
                "depth_path": "depth.png",
                "notes": "",
            }
            with (frame_dir / "frame_meta.yaml").open("w", encoding="utf-8") as handle:
                yaml.safe_dump(metadata, handle, sort_keys=False)
            print(f"Saved {frame_id}")
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
