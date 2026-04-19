from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml

from maripad_perception.perception.book_detector import BookDetector
from maripad_perception.perception.geometry import (
    CameraIntrinsics,
    RigidTransform,
    circular_median_deg,
    circular_distance_deg,
    normalize_book_yaw,
)


def _make_config(tmp_path: Path) -> Path:
    intrinsics_path = tmp_path / "camera_intrinsics.yaml"
    extrinsics_path = tmp_path / "overhead_extrinsics.yaml"
    intrinsics_data = {
        "placeholder": False,
        "width": 640,
        "height": 480,
        "fx": 600.0,
        "fy": 600.0,
        "cx": 320.0,
        "cy": 240.0,
        "depth_unit": "mm",
    }
    extrinsics_data = {
        "placeholder": False,
        "rotation_matrix": [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        "translation_m": [0.0, 0.0, 0.0],
    }
    with intrinsics_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(intrinsics_data, handle)
    with extrinsics_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(extrinsics_data, handle)

    config = {
        "detector": {
            "valid_zone_polygon_px": [[0, 0], [639, 0], [639, 479], [0, 479]],
            "morphology_kernel_size": 5,
            "center_depth_window": 5,
            "invalid_depth_fraction_threshold": 0.5,
            "expected_book_area_px": 7000.0,
            "expected_book_area_tolerance_ratio": 0.3,
            "book_size_mm": {"width_mm": 210.0, "length_mm": 297.0},
            "grip_target": {
                "inset_x_mm": 8.0,
                "inset_y_mm": 8.0,
                "z_offset_mm": 0.0,
            },
            "confidence": {
                "area_weight": 0.35,
                "fill_weight": 0.25,
                "depth_weight": 0.2,
                "zone_weight": 0.2,
                "zone_margin_normalizer_px": 40.0,
                "minimum_confidence": 0.4,
            },
            "temporal": {
                "frame_count": 10,
                "min_valid_detections": 2,
                "max_translation_spread_mm": 3.0,
                "max_yaw_spread_deg": 2.0,
            },
            "isolation": {
                "primary_method": "depth",
                "depth": {
                    "median_kernel_size": 5,
                    "min_depth_mm": 980,
                    "max_depth_mm": 990,
                },
                "hsv": {
                    "lower": [10, 30, 30],
                    "upper": [40, 255, 255],
                },
                "background": {
                    "reference_rgb_path": "missing_rgb.png",
                    "reference_depth_path": "missing_depth.png",
                    "rgb_threshold": 25,
                    "depth_threshold_mm": 5,
                },
            },
        },
        "calibration": {
            "intrinsics_path": str(intrinsics_path),
            "extrinsics_path": str(extrinsics_path),
        },
        "logging": {
            "log_file": str(tmp_path / "book_detector.log"),
            "max_bytes": 1048576,
            "backup_count": 2,
        },
    }
    config_path = tmp_path / "perception_config.yaml"
    with config_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, sort_keys=False)
    return config_path


def _make_synthetic_book_frame(
    width: int = 640,
    height: int = 480,
    center: tuple[int, int] = (320, 240),
    size: tuple[int, int] = (100, 70),
    angle_deg: float = 0.0,
    depth_book_mm: int = 985,
    depth_background_mm: int = 1000,
) -> tuple[np.ndarray, np.ndarray]:
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), depth_background_mm, dtype=np.uint16)

    rect = (center, size, angle_deg)
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.fillConvexPoly(rgb, box, color=(180, 120, 50))
    cv2.fillConvexPoly(depth, box, color=int(depth_book_mm))
    return rgb, depth


def test_normalize_book_yaw_zero(tmp_path: Path) -> None:
    intrinsics = CameraIntrinsics(640, 480, 600.0, 600.0, 320.0, 240.0)
    extrinsics = RigidTransform(rotation=np.eye(3), translation=np.zeros(3))
    rect = ((320.0, 240.0), (60.0, 100.0), 0.0)
    yaw = normalize_book_yaw(rect, 985.0, intrinsics, extrinsics)
    assert circular_distance_deg(yaw, 0.0) < 1e-6


def test_circular_median_wraparound() -> None:
    angles = [179.0, -179.0, 178.0]
    median = circular_median_deg(angles)
    assert circular_distance_deg(median, 179.0) < 1.1


def test_detect_book_pose_success(tmp_path: Path) -> None:
    config_path = _make_config(tmp_path)
    detector = BookDetector(config_path=config_path)
    rgb, depth = _make_synthetic_book_frame()
    result = detector.detect_book_pose(rgb, depth)
    assert result is not None
    assert result.confidence >= 0.4
    assert "confidence_sub_scores" in result.debug


def test_detect_book_pose_rejects_depth_holes(tmp_path: Path) -> None:
    config_path = _make_config(tmp_path)
    detector = BookDetector(config_path=config_path)
    rgb, depth = _make_synthetic_book_frame()
    center_y, center_x = 240, 320
    depth[center_y - 2 : center_y + 3, center_x - 2 : center_x + 3] = 0
    depth[center_y, center_x] = 985
    result, artifacts = detector.process_frame(rgb, depth, return_artifacts=True)
    assert result is None
    assert artifacts is not None
    assert "invalid depth" in artifacts.reason


def test_temporal_detector_rejects_yaw_spread(tmp_path: Path) -> None:
    config_path = _make_config(tmp_path)
    detector = BookDetector(config_path=config_path)
    frames = [
        _make_synthetic_book_frame(angle_deg=0.0),
        _make_synthetic_book_frame(angle_deg=10.0),
        _make_synthetic_book_frame(angle_deg=-10.0),
    ]
    result = detector.detect_book_pose_temporal(frames)
    assert result is None


def test_contour_outside_valid_zone_is_rejected(tmp_path: Path) -> None:
    config_path = _make_config(tmp_path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    config["detector"]["valid_zone_polygon_px"] = [[0, 0], [100, 0], [100, 100], [0, 100]]
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")

    detector = BookDetector(config_path=config_path)
    rgb, depth = _make_synthetic_book_frame(center=(500, 350))
    result = detector.detect_book_pose(rgb, depth)
    assert result is None
