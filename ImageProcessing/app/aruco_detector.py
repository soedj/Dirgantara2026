from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import cv2
import numpy as np

from .config import ArucoConfig


WAYPOINT_NAMES = {
    1: "WP1",
    2: "WP2",
    3: "WP3",
    4: "WP4",
}


def ensure_opencv_version(required_major_minor: str = "4.11") -> None:
    current = cv2.__version__
    current_major_minor = ".".join(current.split(".")[:2])

    if current_major_minor != required_major_minor:
        raise RuntimeError(
            f"OpenCV yang aktif adalah {current}, tetapi project mensyaratkan "
            f"OpenCV {required_major_minor}.x. Aktifkan virtual environment "
            "yang benar atau periksa instalasi OpenCV pada Jetson."
        )

    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "Modul cv2.aruco tidak tersedia. Pada Ubuntu instal "
            "opencv-contrib-python==4.11.0.86."
        )


@dataclass(frozen=True)
class ArucoDetection:
    marker_id: int
    waypoint: str
    corners: np.ndarray
    center_x_px: float
    center_y_px: float
    error_x_normalized: float
    error_y_normalized: float
    area_px2: float
    side_length_px: float
    rotation_deg: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "marker_id": self.marker_id,
            "waypoint": self.waypoint,
            "center_px": {
                "x": round(self.center_x_px, 3),
                "y": round(self.center_y_px, 3),
            },
            "center_error_normalized": {
                "x": round(self.error_x_normalized, 6),
                "y": round(self.error_y_normalized, 6),
            },
            "area_px2": round(self.area_px2, 3),
            "side_length_px": round(self.side_length_px, 3),
            "rotation_deg": round(self.rotation_deg, 3),
            "corners_px": [
                {"x": round(float(point[0]), 3), "y": round(float(point[1]), 3)}
                for point in self.corners
            ],
        }


class KRTIArucoDetector:
    def __init__(self, config: ArucoConfig) -> None:
        self.config = config

        if not hasattr(cv2.aruco, config.dictionary):
            raise ValueError(
                f"Dictionary ArUco tidak dikenal: {config.dictionary}"
            )

        dictionary_id = getattr(cv2.aruco, config.dictionary)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

        parameters = cv2.aruco.DetectorParameters()
        parameters.minMarkerPerimeterRate = config.min_marker_perimeter_rate
        parameters.maxMarkerPerimeterRate = config.max_marker_perimeter_rate
        parameters.markerBorderBits = config.border_bits

        refinement_map = {
            "NONE": cv2.aruco.CORNER_REFINE_NONE,
            "SUBPIX": cv2.aruco.CORNER_REFINE_SUBPIX,
            "CONTOUR": cv2.aruco.CORNER_REFINE_CONTOUR,
            "APRILTAG": cv2.aruco.CORNER_REFINE_APRILTAG,
        }
        parameters.cornerRefinementMethod = refinement_map.get(
            config.corner_refinement,
            cv2.aruco.CORNER_REFINE_SUBPIX,
        )

        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.allowed_ids = set(config.allowed_ids)

    def detect(
        self,
        frame: np.ndarray,
        allow_any_id: bool = False,
    ) -> tuple[list[ArucoDetection], list[np.ndarray]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is None:
            return [], rejected

        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width / 2.0
        frame_center_y = frame_height / 2.0

        detections: list[ArucoDetection] = []

        for marker_corners, marker_id_array in zip(corners, ids):
            marker_id = int(marker_id_array[0])
            if not allow_any_id and marker_id not in self.allowed_ids:
                continue

            points = marker_corners.reshape(4, 2).astype(np.float32)
            center = points.mean(axis=0)
            center_x = float(center[0])
            center_y = float(center[1])

            area = abs(float(cv2.contourArea(points)))
            side_lengths = [
                float(np.linalg.norm(points[(index + 1) % 4] - points[index]))
                for index in range(4)
            ]
            side_length = sum(side_lengths) / 4.0

            top_vector = points[1] - points[0]
            rotation_deg = math.degrees(
                math.atan2(float(top_vector[1]), float(top_vector[0]))
            )

            error_x = (center_x - frame_center_x) / max(frame_center_x, 1.0)
            error_y = (center_y - frame_center_y) / max(frame_center_y, 1.0)

            detections.append(
                ArucoDetection(
                    marker_id=marker_id,
                    waypoint=WAYPOINT_NAMES.get(marker_id, f"ID{marker_id}"),
                    corners=points,
                    center_x_px=center_x,
                    center_y_px=center_y,
                    error_x_normalized=error_x,
                    error_y_normalized=error_y,
                    area_px2=area,
                    side_length_px=side_length,
                    rotation_deg=rotation_deg,
                )
            )

        detections.sort(key=lambda item: item.area_px2, reverse=True)
        return detections, rejected
