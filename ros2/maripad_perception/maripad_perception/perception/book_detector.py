"""Main detector entrypoint for MARIPAD book pose estimation."""

from __future__ import annotations

import logging
import math
import os
from dataclasses import dataclass
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any, Iterable, Sequence

import cv2
import numpy as np
import yaml

from .geometry import (
    CameraIntrinsics,
    RigidTransform,
    circular_distance_deg,
    circular_median_deg,
    compute_grip_target_base,
    load_extrinsics,
    load_intrinsics,
    load_yaml,
    median_of_valid_depth,
    normalize_book_yaw,
    point_in_polygon,
    point_polygon_margin,
    transform_point,
    deproject_pixel_to_camera,
)
from .isolation_methods import (
    available_methods,
    isolate_book_by_background_subtraction,
    isolate_book_by_depth,
    isolate_book_by_hsv,
)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - available in sourced ROS envs
    get_package_share_directory = None


LOGGER_NAME = "maripad_perception.book_detector"


@dataclass(frozen=True)
class BookPose:
    """Detected book pose in robot base frame.

    Units:
        x, y, z are in meters.
        yaw is in degrees.
    """

    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class GripTarget:
    """Grip target in robot base frame, meters."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class DetectionResult:
    """Successful detector output."""

    book_pose: BookPose
    grip_target: GripTarget
    confidence: float
    debug: dict[str, Any]


@dataclass
class DetectionArtifacts:
    """Debug artifacts used by scripts and notebooks."""

    mask: np.ndarray | None = None
    contour: np.ndarray | None = None
    rect: tuple | None = None
    center_px: tuple[float, float] | None = None
    grip_target_px: tuple[int, int] | None = None
    selected_method: str | None = None
    reason: str | None = None


class BookDetector:
    """Stateful detector that loads config, calibration, and reference assets."""

    def __init__(
        self,
        config_path: str | os.PathLike[str] | None = None,
        config_override: dict | None = None,
    ) -> None:
        self.package_root = self._resolve_package_root()
        self.config_path = self._resolve_config_path(config_path)
        self.config = self._load_config(config_override)
        self.logger = self._configure_logger(self.config.get("logging", {}))

        calibration_config = self.config["calibration"]
        intrinsics_path = self._resolve_data_path(calibration_config["intrinsics_path"])
        extrinsics_path = self._resolve_data_path(calibration_config["extrinsics_path"])
        self.intrinsics, self._intrinsics_placeholder = load_intrinsics(
            intrinsics_path, allow_placeholder=True
        )
        self.extrinsics, self._extrinsics_placeholder = load_extrinsics(
            extrinsics_path, allow_placeholder=True
        )

        detector_config = self.config["detector"]
        self.valid_zone_polygon = np.asarray(
            detector_config["valid_zone_polygon_px"], dtype=np.float32
        )
        self._reference_rgb: np.ndarray | None = None
        self._reference_depth: np.ndarray | None = None

    def detect_book_pose(
        self, rgb_image: np.ndarray, depth_image: np.ndarray
    ) -> DetectionResult | None:
        """Detect a single-frame book pose."""
        result, _ = self.process_frame(rgb_image, depth_image, return_artifacts=False)
        return result

    def detect_book_pose_temporal(
        self, frames: Sequence[tuple[np.ndarray, np.ndarray]]
    ) -> DetectionResult | None:
        """Aggregate detections across multiple frames and return the median pose."""
        temporal_config = self.config["detector"]["temporal"]
        results: list[DetectionResult] = []
        rejection_reasons: list[str] = []

        for frame_index, (rgb_image, depth_image) in enumerate(frames):
            result, artifacts = self.process_frame(
                rgb_image, depth_image, return_artifacts=False
            )
            if result is None:
                reason = artifacts.reason if artifacts is not None else "unknown"
                rejection_reasons.append(f"frame_{frame_index}: {reason}")
                continue
            results.append(result)

        min_valid = int(temporal_config["min_valid_detections"])
        if len(results) < min_valid:
            self.logger.warning(
                "temporal detection rejected: only %d valid frame(s), need %d (%s)",
                len(results),
                min_valid,
                "; ".join(rejection_reasons) if rejection_reasons else "no valid detections",
            )
            return None

        positions_m = np.array(
            [[r.book_pose.x, r.book_pose.y, r.book_pose.z] for r in results],
            dtype=np.float64,
        )
        median_translation = np.median(positions_m, axis=0)
        yaws_deg = [r.book_pose.yaw for r in results]
        median_yaw = circular_median_deg(yaws_deg)

        translation_spread_m = np.max(
            np.linalg.norm(positions_m - median_translation, axis=1)
        )
        yaw_spread_deg = max(
            circular_distance_deg(median_yaw, yaw_deg) for yaw_deg in yaws_deg
        )

        max_translation_spread_m = (
            float(temporal_config["max_translation_spread_mm"]) / 1000.0
        )
        max_yaw_spread_deg = float(temporal_config["max_yaw_spread_deg"])

        if translation_spread_m > max_translation_spread_m:
            self.logger.warning(
                "temporal detection rejected: translation spread %.3f mm exceeds %.3f mm",
                translation_spread_m * 1000.0,
                max_translation_spread_m * 1000.0,
            )
            return None

        if yaw_spread_deg > max_yaw_spread_deg:
            self.logger.warning(
                "temporal detection rejected: yaw spread %.3f deg exceeds %.3f deg",
                yaw_spread_deg,
                max_yaw_spread_deg,
            )
            return None

        median_confidence = float(np.median([r.confidence for r in results]))
        inlier_ratio = len(results) / max(1, len(frames))
        aggregate_confidence = max(0.0, min(1.0, median_confidence * inlier_ratio))

        book_pose = BookPose(
            x=float(median_translation[0]),
            y=float(median_translation[1]),
            z=float(median_translation[2]),
            yaw=float(median_yaw),
        )
        grip_target = GripTarget(
            x=float(np.median([r.grip_target.x for r in results])),
            y=float(np.median([r.grip_target.y for r in results])),
            z=float(np.median([r.grip_target.z for r in results])),
        )
        debug = {
            "mode": "temporal",
            "valid_frame_count": len(results),
            "translation_spread_mm": translation_spread_m * 1000.0,
            "yaw_spread_deg": yaw_spread_deg,
            "median_confidence": median_confidence,
            "inlier_ratio": inlier_ratio,
            "rejection_reasons": rejection_reasons,
        }

        self.logger.info(
            "temporal detection success: confidence=%.3f valid=%d spread_mm=%.3f yaw_spread_deg=%.3f",
            aggregate_confidence,
            len(results),
            translation_spread_m * 1000.0,
            yaw_spread_deg,
        )
        return DetectionResult(
            book_pose=book_pose,
            grip_target=grip_target,
            confidence=aggregate_confidence,
            debug=debug,
        )

    def process_frame(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        return_artifacts: bool = True,
    ) -> tuple[DetectionResult | None, DetectionArtifacts | None]:
        """Run the detector on one frame, optionally returning debug artifacts."""
        artifacts = DetectionArtifacts() if return_artifacts else None
        try:
            self._validate_inputs(rgb_image, depth_image)
            result = self._detect_single(rgb_image, depth_image, artifacts)
            return result, artifacts
        except Exception as exc:
            reason = f"exception: {exc}"
            if artifacts is not None:
                artifacts.reason = reason
            self.logger.exception("detection failed with exception")
            return None, artifacts

    def _detect_single(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        artifacts: DetectionArtifacts | None,
    ) -> DetectionResult | None:
        detector_config = self.config["detector"]
        mask, selected_method = self._isolate_book(rgb_image, depth_image)
        if artifacts is not None:
            artifacts.mask = mask
            artifacts.selected_method = selected_method

        kernel_size = int(detector_config["morphology_kernel_size"])
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        if artifacts is not None:
            artifacts.mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return self._reject("no contours found after isolation", artifacts)

        contour, area_px = self._select_contour(contours)
        if contour is None:
            return self._reject("no contour passed area and valid-zone filters", artifacts)
        if artifacts is not None:
            artifacts.contour = contour

        rect = cv2.minAreaRect(contour)
        if artifacts is not None:
            artifacts.rect = rect
        (center_x, center_y), _, _ = rect
        if artifacts is not None:
            artifacts.center_px = (center_x, center_y)

        center_depth_mm, invalid_fraction = self._sample_center_depth(depth_image, center_x, center_y)
        if center_depth_mm is None:
            return self._reject(
                (
                    "invalid depth at OBB center "
                    f"(invalid_fraction={invalid_fraction:.3f})"
                ),
                artifacts,
            )

        if self._intrinsics_placeholder:
            return self._reject(
                "camera intrinsics are still placeholder values; "
                "capture/bakeoff can continue but base-frame pose estimation is disabled",
                artifacts,
            )

        if self._extrinsics_placeholder:
            return self._reject(
                "hand-eye extrinsics are not calibrated yet; "
                "capture/bakeoff can continue but base-frame pose estimation is disabled",
                artifacts,
            )

        center_camera = deproject_pixel_to_camera(
            center_x, center_y, center_depth_mm, self.intrinsics
        )
        center_base = transform_point(center_camera, self.extrinsics)
        yaw_deg = normalize_book_yaw(rect, center_depth_mm, self.intrinsics, self.extrinsics)

        grip_target_base = compute_grip_target_base(
            center_base,
            yaw_deg,
            detector_config["book_size_mm"],
            detector_config["grip_target"],
        )

        confidence, sub_scores = self._compute_confidence(
            contour=contour,
            contour_area_px=area_px,
            rect=rect,
            center_px=(center_x, center_y),
            invalid_fraction=invalid_fraction,
        )

        minimum_confidence = float(detector_config["confidence"]["minimum_confidence"])
        if confidence < minimum_confidence:
            return self._reject(
                f"confidence {confidence:.3f} below threshold {minimum_confidence:.3f}",
                artifacts,
                extra_debug={"confidence_sub_scores": sub_scores},
            )

        book_pose = BookPose(
            x=float(center_base[0]),
            y=float(center_base[1]),
            z=float(center_base[2]),
            yaw=float(yaw_deg),
        )
        grip_target = GripTarget(
            x=float(grip_target_base[0]),
            y=float(grip_target_base[1]),
            z=float(grip_target_base[2]),
        )
        debug = {
            "selected_method": selected_method,
            "contour_area_px": area_px,
            "expected_book_area_px": float(detector_config["expected_book_area_px"]),
            "expected_area_ratio": area_px / float(detector_config["expected_book_area_px"]),
            "depth_mm": center_depth_mm,
            "invalid_depth_fraction": invalid_fraction,
            "center_px": [center_x, center_y],
            "confidence_sub_scores": sub_scores,
        }
        self.logger.info(
            "detection success: method=%s area=%.1f ratio=%.3f yaw=%.3f conf=%.3f depth_invalid=%.3f",
            selected_method,
            area_px,
            debug["expected_area_ratio"],
            yaw_deg,
            confidence,
            invalid_fraction,
        )
        return DetectionResult(
            book_pose=book_pose,
            grip_target=grip_target,
            confidence=confidence,
            debug=debug,
        )

    def _isolate_book(
        self, rgb_image: np.ndarray, depth_image: np.ndarray
    ) -> tuple[np.ndarray, str]:
        isolation_config = self.config["detector"]["isolation"]
        method = str(isolation_config["primary_method"]).lower()
        if method not in available_methods():
            raise ValueError(
                f"Unsupported isolation method '{method}'. Available: {available_methods()}"
            )

        if method == "depth":
            mask = isolate_book_by_depth(depth_image, isolation_config["depth"])
        elif method == "hsv":
            mask = isolate_book_by_hsv(rgb_image, isolation_config["hsv"])
        else:
            reference_rgb, reference_depth = self._load_background_references()
            mask = isolate_book_by_background_subtraction(
                rgb_image,
                depth_image,
                isolation_config["background"],
                reference_rgb,
                reference_depth,
            )
        return mask, method

    def generate_masks_for_all_methods(
        self, rgb_image: np.ndarray, depth_image: np.ndarray
    ) -> dict[str, np.ndarray]:
        """Return raw masks for all configured methods, used by the notebook."""
        masks = {}
        isolation_config = self.config["detector"]["isolation"]
        masks["depth"] = isolate_book_by_depth(depth_image, isolation_config["depth"])
        masks["hsv"] = isolate_book_by_hsv(rgb_image, isolation_config["hsv"])
        reference_rgb, reference_depth = self._load_background_references()
        masks["background"] = isolate_book_by_background_subtraction(
            rgb_image,
            depth_image,
            isolation_config["background"],
            reference_rgb,
            reference_depth,
        )
        return masks

    def score_mask_quality(self, mask: np.ndarray) -> dict[str, float]:
        """Compute lightweight mask metrics for bakeoff visualization."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return {
                "largest_area_px": 0.0,
                "contour_count": 0.0,
                "largest_fill_ratio": 0.0,
            }
        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        rect = cv2.minAreaRect(largest)
        rect_area = float(rect[1][0] * rect[1][1]) if rect[1][0] > 0 and rect[1][1] > 0 else 0.0
        fill_ratio = area / rect_area if rect_area > 0.0 else 0.0
        return {
            "largest_area_px": area,
            "contour_count": float(len(contours)),
            "largest_fill_ratio": float(fill_ratio),
        }

    def _sample_center_depth(
        self, depth_image: np.ndarray, center_x: float, center_y: float
    ) -> tuple[float | None, float]:
        window_size = int(self.config["detector"]["center_depth_window"])
        radius = window_size // 2
        x0 = max(0, int(round(center_x)) - radius)
        x1 = min(depth_image.shape[1], int(round(center_x)) + radius + 1)
        y0 = max(0, int(round(center_y)) - radius)
        y1 = min(depth_image.shape[0], int(round(center_y)) + radius + 1)
        depth_window = depth_image[y0:y1, x0:x1]
        invalid_threshold = float(
            self.config["detector"]["invalid_depth_fraction_threshold"]
        )
        return median_of_valid_depth(depth_window, invalid_threshold)

    def _select_contour(
        self, contours: Sequence[np.ndarray]
    ) -> tuple[np.ndarray | None, float]:
        detector_config = self.config["detector"]
        expected_area_px = float(detector_config["expected_book_area_px"])
        tolerance = float(detector_config["expected_book_area_tolerance_ratio"])
        min_area = expected_area_px * (1.0 - tolerance)
        max_area = expected_area_px * (1.0 + tolerance)

        candidates = []
        for contour in contours:
            area_px = float(cv2.contourArea(contour))
            if area_px < min_area or area_px > max_area:
                continue
            moments = cv2.moments(contour)
            if math.isclose(moments["m00"], 0.0, abs_tol=1e-9):
                continue
            centroid = (
                moments["m10"] / moments["m00"],
                moments["m01"] / moments["m00"],
            )
            if not point_in_polygon(centroid, self.valid_zone_polygon):
                continue
            candidates.append((area_px, contour))

        if not candidates:
            return None, 0.0
        area_px, contour = max(candidates, key=lambda item: item[0])
        return contour, area_px

    def _compute_confidence(
        self,
        contour: np.ndarray,
        contour_area_px: float,
        rect: tuple,
        center_px: tuple[float, float],
        invalid_fraction: float,
    ) -> tuple[float, dict[str, float]]:
        confidence_config = self.config["detector"]["confidence"]
        expected_area_px = float(self.config["detector"]["expected_book_area_px"])
        tolerance = float(self.config["detector"]["expected_book_area_tolerance_ratio"])
        area_delta = abs(contour_area_px - expected_area_px)
        area_score = max(0.0, 1.0 - (area_delta / max(1.0, expected_area_px * tolerance)))

        width_px, height_px = rect[1]
        rect_area_px = float(width_px * height_px)
        fill_score = (
            max(0.0, min(1.0, contour_area_px / rect_area_px))
            if rect_area_px > 0.0
            else 0.0
        )

        depth_score = max(0.0, min(1.0, 1.0 - invalid_fraction))

        zone_margin_px = point_polygon_margin(center_px, self.valid_zone_polygon)
        max_zone_margin_px = max(
            1.0, float(confidence_config["zone_margin_normalizer_px"])
        )
        zone_score = max(0.0, min(1.0, zone_margin_px / max_zone_margin_px))

        weights = {
            "area_score": float(confidence_config["area_weight"]),
            "fill_score": float(confidence_config["fill_weight"]),
            "depth_score": float(confidence_config["depth_weight"]),
            "zone_score": float(confidence_config["zone_weight"]),
        }
        weight_sum = sum(weights.values())
        if weight_sum <= 0.0:
            raise ValueError("Confidence weights must sum to a positive number")

        weighted_sum = (
            (area_score * weights["area_score"])
            + (fill_score * weights["fill_score"])
            + (depth_score * weights["depth_score"])
            + (zone_score * weights["zone_score"])
        )
        confidence = max(0.0, min(1.0, weighted_sum / weight_sum))
        sub_scores = {
            "area_score": area_score,
            "fill_score": fill_score,
            "depth_score": depth_score,
            "zone_score": zone_score,
            "zone_margin_px": zone_margin_px,
        }
        return confidence, sub_scores

    def _load_background_references(self) -> tuple[np.ndarray, np.ndarray]:
        if self._reference_rgb is not None and self._reference_depth is not None:
            return self._reference_rgb, self._reference_depth

        background_config = self.config["detector"]["isolation"]["background"]
        rgb_path = self._resolve_data_path(background_config["reference_rgb_path"])
        depth_path = self._resolve_data_path(background_config["reference_depth_path"])
        self._reference_rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if self._reference_rgb is None:
            raise FileNotFoundError(f"Failed to load reference RGB image: {rgb_path}")
        self._reference_rgb = cv2.cvtColor(self._reference_rgb, cv2.COLOR_BGR2RGB)
        self._reference_depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if self._reference_depth is None:
            raise FileNotFoundError(f"Failed to load reference depth image: {depth_path}")
        return self._reference_rgb, self._reference_depth

    def _validate_inputs(self, rgb_image: np.ndarray, depth_image: np.ndarray) -> None:
        if not isinstance(rgb_image, np.ndarray) or not isinstance(depth_image, np.ndarray):
            raise TypeError("rgb_image and depth_image must be numpy arrays")
        if rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
            raise ValueError("rgb_image must have shape HxWx3 in RGB order")
        if depth_image.ndim != 2:
            raise ValueError("depth_image must be a single-channel depth image")
        if rgb_image.shape[:2] != depth_image.shape:
            raise ValueError("rgb_image and depth_image shapes must match")
        if rgb_image.dtype != np.uint8:
            raise ValueError("rgb_image must be uint8")
        if depth_image.dtype != np.uint16:
            raise ValueError(
                "depth_image must be uint16 depth in millimeters"
            )

    def _reject(
        self,
        reason: str,
        artifacts: DetectionArtifacts | None,
        extra_debug: dict[str, Any] | None = None,
    ) -> None:
        if artifacts is not None:
            artifacts.reason = reason
        if extra_debug:
            self.logger.warning("detection rejected: %s | %s", reason, extra_debug)
        else:
            self.logger.warning("detection rejected: %s", reason)
        return None

    def _resolve_package_root(self) -> Path:
        if get_package_share_directory is not None:
            try:
                share_dir = Path(get_package_share_directory("maripad_perception"))
                return share_dir.parent.parent / "src" / "maripad_perception"
            except Exception:
                pass
        return Path(__file__).resolve().parents[2]

    def _resolve_config_path(
        self, config_path: str | os.PathLike[str] | None
    ) -> Path:
        if config_path is None:
            env_path = os.environ.get("MARIPAD_PERCEPTION_CONFIG")
            if env_path:
                return Path(env_path).expanduser().resolve()
            return (self.package_root / "config" / "perception_config.yaml").resolve()
        return Path(config_path).expanduser().resolve()

    def _load_config(self, config_override: dict | None) -> dict:
        config = load_yaml(self.config_path)
        if config_override:
            config = _deep_merge(config, config_override)
        return config

    def _resolve_data_path(self, configured_path: str) -> Path:
        candidate = Path(configured_path).expanduser()
        if candidate.is_absolute():
            return candidate

        config_relative = (self.config_path.parent / candidate).resolve()
        if config_relative.exists():
            return config_relative

        package_relative = (self.package_root / candidate).resolve()
        if package_relative.exists():
            return package_relative

        share_relative = (self.package_root / candidate).resolve()
        return share_relative

    def _configure_logger(self, logging_config: dict) -> logging.Logger:
        logger = logging.getLogger(LOGGER_NAME)
        logger.setLevel(logging.INFO)
        if logger.handlers:
            return logger

        log_file = Path(
            os.path.expanduser(
                logging_config.get(
                    "log_file",
                    "~/.ros/log/maripad_perception/book_detector.log",
                )
            )
        )
        log_file.parent.mkdir(parents=True, exist_ok=True)
        handler = RotatingFileHandler(
            log_file,
            maxBytes=int(logging_config.get("max_bytes", 1_048_576)),
            backupCount=int(logging_config.get("backup_count", 5)),
        )
        formatter = logging.Formatter(
            "%(asctime)s %(levelname)s %(name)s %(message)s"
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.propagate = False
        return logger


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge dictionaries."""
    result = dict(base)
    for key, value in override.items():
        if isinstance(result.get(key), dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


_DEFAULT_DETECTOR: BookDetector | None = None


def _get_default_detector() -> BookDetector:
    global _DEFAULT_DETECTOR
    if _DEFAULT_DETECTOR is None:
        _DEFAULT_DETECTOR = BookDetector()
    return _DEFAULT_DETECTOR


def detect_book_pose(
    rgb_image: np.ndarray, depth_image: np.ndarray
) -> DetectionResult | None:
    """Public convenience wrapper around the default detector."""
    return _get_default_detector().detect_book_pose(rgb_image, depth_image)


def detect_book_pose_temporal(
    frames: Sequence[tuple[np.ndarray, np.ndarray]]
) -> DetectionResult | None:
    """Public convenience wrapper for the temporal detector."""
    return _get_default_detector().detect_book_pose_temporal(frames)
