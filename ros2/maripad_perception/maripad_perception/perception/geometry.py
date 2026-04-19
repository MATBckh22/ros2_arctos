"""Geometry helpers for MARIPAD book pose detection.

Yaw convention:
    - returned in degrees
    - relative to the workbench long axis (+Y in the robot base frame)
    - positive means counterclockwise viewed from above

This module centralizes the OpenCV `minAreaRect` angle handling because its
reported angle flips meaning depending on the rectangle width/height ordering.
The helper functions here convert the rectangle into an explicit long-axis
vector before any frame transforms or yaw normalization are applied.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for depth deprojection."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_unit: str = "mm"


@dataclass(frozen=True)
class RigidTransform:
    """Rigid transform from camera frame to base frame."""

    rotation: np.ndarray
    translation: np.ndarray


def load_yaml(path: Path) -> dict:
    """Load a YAML document from disk."""
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in YAML file: {path}")
    return data


def load_intrinsics(
    path: Path, allow_placeholder: bool = False
) -> tuple[CameraIntrinsics, bool]:
    """Load camera intrinsics from YAML."""
    data = load_yaml(path)
    placeholder = bool(data.get("placeholder", False))
    if placeholder and not allow_placeholder:
        raise ValueError(
            f"Camera intrinsics file {path} still contains placeholder values."
        )
    return (
        CameraIntrinsics(
            width=int(data["width"]),
            height=int(data["height"]),
            fx=float(data["fx"]),
            fy=float(data["fy"]),
            cx=float(data["cx"]),
            cy=float(data["cy"]),
            depth_unit=str(data.get("depth_unit", "mm")),
        ),
        placeholder,
    )


def load_extrinsics(
    path: Path, allow_placeholder: bool = False
) -> tuple[RigidTransform, bool]:
    """Load camera-to-base extrinsics from YAML."""
    data = load_yaml(path)
    placeholder = bool(data.get("placeholder", False))
    if placeholder and not allow_placeholder:
        raise ValueError(
            f"Extrinsics file {path} still contains placeholder values."
        )

    if "rotation_matrix" in data and "translation_m" in data:
        rotation = np.asarray(data["rotation_matrix"], dtype=np.float64)
        translation = np.asarray(data["translation_m"], dtype=np.float64)
    elif "transform_matrix" in data:
        matrix = np.asarray(data["transform_matrix"], dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError(f"Expected 4x4 transform_matrix in {path}")
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]
    else:
        raise ValueError(
            f"Extrinsics file {path} must contain rotation_matrix/translation_m "
            "or transform_matrix"
        )

    if rotation.shape != (3, 3):
        raise ValueError(f"Expected 3x3 rotation matrix in {path}")
    if translation.shape != (3,):
        raise ValueError(f"Expected 3-vector translation in {path}")
    return RigidTransform(rotation=rotation, translation=translation), placeholder


def deproject_pixel_to_camera(
    u_px: float, v_px: float, depth_mm: float, intrinsics: CameraIntrinsics
) -> np.ndarray:
    """Deproject a pixel into the camera frame in meters."""
    if depth_mm <= 0.0:
        raise ValueError("Depth must be positive for deprojection")

    depth_m = depth_mm / 1000.0
    x_m = (u_px - intrinsics.cx) * depth_m / intrinsics.fx
    y_m = (v_px - intrinsics.cy) * depth_m / intrinsics.fy
    return np.array([x_m, y_m, depth_m], dtype=np.float64)


def transform_point(point_camera_m: np.ndarray, extrinsics: RigidTransform) -> np.ndarray:
    """Transform a camera-frame point into the robot base frame."""
    point = np.asarray(point_camera_m, dtype=np.float64).reshape(3)
    return extrinsics.rotation @ point + extrinsics.translation


def transform_points(points_camera_m: np.ndarray, extrinsics: RigidTransform) -> np.ndarray:
    """Transform multiple camera-frame points into the robot base frame."""
    points = np.asarray(points_camera_m, dtype=np.float64)
    return (extrinsics.rotation @ points.T).T + extrinsics.translation


def rect_long_axis_endpoints(rect: tuple) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return image-space endpoints of the minAreaRect long axis.

    OpenCV's `minAreaRect` returns an angle whose interpretation depends on
    whether width or height is longer. To avoid subtle sign flips, this helper
    derives the long-axis endpoints from the actual box corners instead.
    """
    box = cv2.boxPoints(rect).astype(np.float64)
    edges = []
    for index in range(4):
        p0 = box[index]
        p1 = box[(index + 1) % 4]
        edges.append((float(np.linalg.norm(p1 - p0)), p0, p1))
    _, p0, p1 = max(edges, key=lambda item: item[0])
    return (float(p0[0]), float(p0[1])), (float(p1[0]), float(p1[1]))


def yaw_from_vector_base(direction_xy: np.ndarray) -> float:
    """Return yaw in degrees relative to +Y in base frame, CCW viewed from above."""
    dx = float(direction_xy[0])
    dy = float(direction_xy[1])
    if math.isclose(dx, 0.0, abs_tol=1e-9) and math.isclose(dy, 0.0, abs_tol=1e-9):
        raise ValueError("Cannot compute yaw from a zero-length vector")

    # Relative to +Y, positive CCW viewed from above.
    yaw = math.degrees(math.atan2(-dx, dy))
    return wrap_angle_deg(yaw)


def normalize_book_yaw(
    rect: tuple,
    depth_mm: float,
    intrinsics: CameraIntrinsics,
    extrinsics: RigidTransform,
) -> float:
    """Normalize a book yaw from `minAreaRect` into the base-frame convention."""
    start_px, end_px = rect_long_axis_endpoints(rect)
    start_camera = deproject_pixel_to_camera(*start_px, depth_mm, intrinsics)
    end_camera = deproject_pixel_to_camera(*end_px, depth_mm, intrinsics)
    start_base, end_base = transform_points(
        np.vstack([start_camera, end_camera]), extrinsics
    )

    direction = end_base[:2] - start_base[:2]
    if direction[1] < 0.0:
        direction = -direction
    return yaw_from_vector_base(direction)


def wrap_angle_deg(angle_deg: float) -> float:
    """Wrap an angle into [-180, 180)."""
    wrapped = ((angle_deg + 180.0) % 360.0) - 180.0
    if math.isclose(wrapped, -180.0, abs_tol=1e-9):
        return 180.0
    return wrapped


def circular_distance_deg(a_deg: float, b_deg: float) -> float:
    """Shortest absolute angular distance in degrees."""
    return abs(wrap_angle_deg(a_deg - b_deg))


def circular_median_deg(angles_deg: Sequence[float]) -> float:
    """Return the circular median for angles in degrees.

    The implementation explicitly handles wrap-around cases like +179/-179 by
    picking the angle that minimizes total circular distance to the set.
    """
    if not angles_deg:
        raise ValueError("At least one angle is required for circular median")

    normalized = [wrap_angle_deg(float(value)) for value in angles_deg]
    best_angle = normalized[0]
    best_cost = math.inf

    for candidate in normalized:
        cost = sum(circular_distance_deg(candidate, other) for other in normalized)
        if cost < best_cost:
            best_cost = cost
            best_angle = candidate

    return wrap_angle_deg(best_angle)


def rotation_matrix_z(yaw_deg: float) -> np.ndarray:
    """Return a planar rotation matrix for yaw around +Z."""
    angle_rad = math.radians(yaw_deg)
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def compute_grip_target_base(
    book_center_base_m: Sequence[float],
    yaw_deg: float,
    book_size_mm: dict,
    grip_config: dict,
) -> np.ndarray:
    """Compute the grip target in base frame from the book pose."""
    width_m = float(book_size_mm["width_mm"]) / 1000.0
    length_m = float(book_size_mm["length_mm"]) / 1000.0
    inset_x_m = float(grip_config["inset_x_mm"]) / 1000.0
    inset_y_m = float(grip_config["inset_y_mm"]) / 1000.0
    z_offset_m = float(grip_config.get("z_offset_mm", 0.0)) / 1000.0

    local_target = np.array(
        [
            (width_m / 2.0) - inset_x_m,
            (length_m / 2.0) - inset_y_m,
            z_offset_m,
        ],
        dtype=np.float64,
    )
    rotation = rotation_matrix_z(yaw_deg)
    center = np.asarray(book_center_base_m, dtype=np.float64)
    return center + rotation @ local_target


def point_in_polygon(point_xy: Sequence[float], polygon_xy: np.ndarray) -> bool:
    """Return whether a point lies inside or on the polygon."""
    polygon = np.asarray(polygon_xy, dtype=np.float32)
    point = (float(point_xy[0]), float(point_xy[1]))
    return cv2.pointPolygonTest(polygon, point, False) >= 0.0


def point_polygon_margin(point_xy: Sequence[float], polygon_xy: np.ndarray) -> float:
    """Return signed distance in pixels from point to polygon edge."""
    polygon = np.asarray(polygon_xy, dtype=np.float32)
    point = (float(point_xy[0]), float(point_xy[1]))
    return float(cv2.pointPolygonTest(polygon, point, True))


def median_of_valid_depth(depth_window: np.ndarray, invalid_threshold: float) -> tuple[float | None, float]:
    """Return median depth and invalid fraction from a depth window."""
    valid = depth_window > 0
    invalid_fraction = 1.0 - (np.count_nonzero(valid) / depth_window.size)
    if invalid_fraction > invalid_threshold:
        return None, invalid_fraction
    if not np.any(valid):
        return None, 1.0
    median_depth = float(np.median(depth_window[valid]))
    return median_depth, invalid_fraction
