from __future__ import annotations

import cv2
import numpy as np

from .aruco_detector import ArucoDetection


def draw_crosshair(frame: np.ndarray) -> None:
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)

    cv2.line(frame, (center[0] - 25, center[1]), (center[0] + 25, center[1]),
             (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(frame, (center[0], center[1] - 25), (center[0], center[1] + 25),
             (255, 255, 255), 1, cv2.LINE_AA)


def draw_detections(
    frame: np.ndarray,
    detections: list[ArucoDetection],
) -> None:
    height, width = frame.shape[:2]
    frame_center = (width // 2, height // 2)

    for detection in detections:
        points = detection.corners.astype(np.int32)
        cv2.polylines(
            frame,
            [points],
            isClosed=True,
            color=(0, 255, 0),
            thickness=3,
            lineType=cv2.LINE_AA,
        )

        center = (
            int(round(detection.center_x_px)),
            int(round(detection.center_y_px)),
        )
        cv2.circle(frame, center, 6, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.line(frame, frame_center, center, (255, 255, 0), 2, cv2.LINE_AA)

        label = (
            f"{detection.waypoint} | ID {detection.marker_id} | "
            f"ex={detection.error_x_normalized:+.3f} "
            f"ey={detection.error_y_normalized:+.3f}"
        )

        text_origin = (
            max(5, int(points[:, 0].min())),
            max(25, int(points[:, 1].min()) - 10),
        )
        cv2.putText(
            frame,
            label,
            text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            label,
            text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


def draw_status(
    frame: np.ndarray,
    *,
    camera_name: str,
    camera_mount: str,
    dictionary_name: str,
    fps: float,
    detection_count: int,
) -> None:
    lines = [
        f"Camera: {camera_name} ({camera_mount})",
        f"Dictionary: {dictionary_name}",
        f"FPS: {fps:.1f}",
        f"Waypoint terdeteksi: {detection_count}",
        "Tekan Q atau ESC untuk keluar",
    ]

    panel_width = min(frame.shape[1] - 20, 470)
    panel_height = 30 + len(lines) * 26
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (10 + panel_width, 10 + panel_height),
                  (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    for index, text in enumerate(lines):
        cv2.putText(
            frame,
            text,
            (25, 42 + index * 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
