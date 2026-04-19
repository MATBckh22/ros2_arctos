"""Isolation strategies for the MARIPAD answer book."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


def median_filter_depth(depth_image: np.ndarray, kernel_size: int) -> np.ndarray:
    """Apply median filtering to a uint16 depth image."""
    if kernel_size % 2 == 0:
        raise ValueError("Median kernel size must be odd")
    return cv2.medianBlur(depth_image, kernel_size)


def isolate_book_by_depth(depth_image: np.ndarray, config: dict) -> np.ndarray:
    """Isolate the book by absolute depth range."""
    filtered_depth = median_filter_depth(
        depth_image, int(config["median_kernel_size"])
    )
    min_depth_mm = int(config["min_depth_mm"])
    max_depth_mm = int(config["max_depth_mm"])
    mask = (
        (filtered_depth > 0)
        & (filtered_depth >= min_depth_mm)
        & (filtered_depth <= max_depth_mm)
    )
    return (mask.astype(np.uint8) * 255)


def isolate_book_by_hsv(rgb_image: np.ndarray, config: dict) -> np.ndarray:
    """Isolate the book by a configured HSV cover-color range."""
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    lower = np.array(config["lower"], dtype=np.uint8)
    upper = np.array(config["upper"], dtype=np.uint8)
    mask = cv2.inRange(hsv_image, lower, upper)
    return mask


def load_reference_image(path: Path, flags: int) -> np.ndarray:
    """Load a reference image from disk with an explicit error on failure."""
    image = cv2.imread(str(path), flags)
    if image is None:
        raise FileNotFoundError(f"Failed to load reference image: {path}")
    return image


def isolate_book_by_background_subtraction(
    rgb_image: np.ndarray,
    depth_image: np.ndarray,
    config: dict,
    reference_rgb: np.ndarray,
    reference_depth: np.ndarray,
) -> np.ndarray:
    """Isolate the book by difference from an empty-workbench reference."""
    if reference_rgb.shape != rgb_image.shape:
        raise ValueError("Reference RGB image shape does not match input image")
    if reference_depth.shape != depth_image.shape:
        raise ValueError("Reference depth image shape does not match input image")

    rgb_delta = cv2.absdiff(rgb_image, reference_rgb)
    rgb_delta_gray = cv2.cvtColor(rgb_delta, cv2.COLOR_RGB2GRAY)
    rgb_mask = rgb_delta_gray >= int(config["rgb_threshold"])

    depth_delta = np.abs(depth_image.astype(np.int32) - reference_depth.astype(np.int32))
    depth_mask = depth_delta >= int(config["depth_threshold_mm"])

    combined = np.logical_or(rgb_mask, depth_mask)
    return combined.astype(np.uint8) * 255


def available_methods() -> tuple[str, ...]:
    """Return the available isolation method names."""
    return ("depth", "hsv", "background")
