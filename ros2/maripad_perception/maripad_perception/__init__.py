"""Public API for MARIPAD book pose detection."""

from maripad_perception.perception.book_detector import (
    BookDetector,
    BookPose,
    DetectionResult,
    GripTarget,
    detect_book_pose,
    detect_book_pose_temporal,
)

__all__ = [
    "BookDetector",
    "BookPose",
    "DetectionResult",
    "GripTarget",
    "detect_book_pose",
    "detect_book_pose_temporal",
]
