from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CameraConfig:
    name: str
    mount: str
    source_type: str
    index: int
    device: str
    input_path: str | None
    width: int
    height: int
    fps: int
    sensor_id: int
    flip_method: int
    pipeline: str | None


@dataclass(frozen=True)
class ArucoConfig:
    dictionary: str
    allowed_ids: tuple[int, ...]
    border_bits: int
    corner_refinement: str
    min_marker_perimeter_rate: float
    max_marker_perimeter_rate: float


@dataclass(frozen=True)
class OutputConfig:
    display: bool
    jsonl_path: str | None
    save_video_path: str | None
    json_rate_hz: float


@dataclass(frozen=True)
class AppConfig:
    required_opencv: str
    camera: CameraConfig
    aruco: ArucoConfig
    output: OutputConfig


def _require_dict(data: dict[str, Any], key: str) -> dict[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"Konfigurasi {key!r} harus berupa object JSON")
    return value


def load_config(path: str | Path) -> AppConfig:
    config_path = Path(path)
    if not config_path.is_file():
        raise FileNotFoundError(f"File konfigurasi tidak ditemukan: {config_path}")

    data = json.loads(config_path.read_text(encoding="utf-8"))
    camera_data = _require_dict(data, "camera")
    aruco_data = _require_dict(data, "aruco")
    output_data = _require_dict(data, "output")

    camera = CameraConfig(
        name=str(camera_data.get("name", "camera")),
        mount=str(camera_data.get("mount", "unknown")),
        source_type=str(camera_data.get("source_type", "usb")).lower(),
        index=int(camera_data.get("index", 0)),
        device=str(camera_data.get("device", "/dev/video0")),
        input_path=(
            None
            if camera_data.get("input_path") in (None, "")
            else str(camera_data["input_path"])
        ),
        width=int(camera_data.get("width", 1280)),
        height=int(camera_data.get("height", 720)),
        fps=int(camera_data.get("fps", 30)),
        sensor_id=int(camera_data.get("sensor_id", 0)),
        flip_method=int(camera_data.get("flip_method", 0)),
        pipeline=(
            None
            if camera_data.get("pipeline") in (None, "")
            else str(camera_data["pipeline"])
        ),
    )

    aruco = ArucoConfig(
        dictionary=str(aruco_data.get("dictionary", "DICT_7X7_50")),
        allowed_ids=tuple(int(value) for value in aruco_data.get(
            "allowed_ids", [1, 2, 3, 4]
        )),
        border_bits=int(aruco_data.get("border_bits", 1)),
        corner_refinement=str(
            aruco_data.get("corner_refinement", "SUBPIX")
        ).upper(),
        min_marker_perimeter_rate=float(
            aruco_data.get("min_marker_perimeter_rate", 0.02)
        ),
        max_marker_perimeter_rate=float(
            aruco_data.get("max_marker_perimeter_rate", 4.0)
        ),
    )

    output = OutputConfig(
        display=bool(output_data.get("display", True)),
        jsonl_path=(
            None
            if output_data.get("jsonl_path") in (None, "")
            else str(output_data["jsonl_path"])
        ),
        save_video_path=(
            None
            if output_data.get("save_video_path") in (None, "")
            else str(output_data["save_video_path"])
        ),
        json_rate_hz=float(output_data.get("json_rate_hz", 5.0)),
    )

    required_opencv = str(data.get("required_opencv", "4.11"))

    if camera.width <= 0 or camera.height <= 0 or camera.fps <= 0:
        raise ValueError("width, height, dan fps harus lebih besar dari 0")
    if output.json_rate_hz <= 0:
        raise ValueError("output.json_rate_hz harus lebih besar dari 0")
    if camera.source_type not in {"usb", "csi", "file", "gstreamer"}:
        raise ValueError(
            "camera.source_type harus usb, csi, file, atau gstreamer"
        )

    return AppConfig(
        required_opencv=required_opencv,
        camera=camera,
        aruco=aruco,
        output=output,
    )
