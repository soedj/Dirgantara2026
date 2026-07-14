from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import cv2

from .aruco_detector import KRTIArucoDetector, ensure_opencv_version
from .camera import CameraSource
from .config import AppConfig, load_config
from .overlay import draw_crosshair, draw_detections, draw_status


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Deteksi waypoint ArUco KRTI 2026 VTOL."
    )
    parser.add_argument(
        "--config",
        default="config/ubuntu_usb.json",
        help="File konfigurasi JSON.",
    )
    parser.add_argument(
        "--source-type",
        choices=["usb", "csi", "file", "gstreamer"],
        help="Override jenis sumber kamera.",
    )
    parser.add_argument("--camera-index", type=int)
    parser.add_argument("--device", help="Contoh: /dev/video0")
    parser.add_argument("--input", help="Path video untuk source-type=file")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Tanpa cv2.imshow; cocok untuk Jetson tanpa monitor.",
    )
    parser.add_argument(
        "--allow-any-id",
        action="store_true",
        help="Tampilkan seluruh ID pada dictionary, bukan hanya ID 1-4.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Berhenti setelah N frame; 0 berarti terus berjalan.",
    )
    return parser.parse_args()


def apply_overrides(config: AppConfig, args: argparse.Namespace) -> AppConfig:
    camera = config.camera
    output = config.output

    if args.source_type:
        camera = replace(camera, source_type=args.source_type)
    if args.camera_index is not None:
        camera = replace(camera, index=args.camera_index)
    if args.device:
        camera = replace(camera, device=args.device)
    if args.input:
        camera = replace(camera, input_path=args.input, source_type="file")
    if args.headless:
        output = replace(output, display=False)

    return replace(config, camera=camera, output=output)


def open_jsonl(path_value: str | None):
    if not path_value:
        return None

    path = Path(path_value)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.open("a", encoding="utf-8", buffering=1)


def create_video_writer(
    path_value: str | None,
    width: int,
    height: int,
    fps: int,
):
    if not path_value:
        return None

    path = Path(path_value)
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (width, height),
    )
    if not writer.isOpened():
        writer.release()
        raise RuntimeError(f"Gagal membuka output video: {path}")
    return writer


def main() -> int:
    args = parse_args()
    config = apply_overrides(load_config(args.config), args)

    ensure_opencv_version(config.required_opencv)

    detector = KRTIArucoDetector(config.aruco)
    camera = CameraSource.open(config.camera)
    jsonl_file = open_jsonl(config.output.jsonl_path)
    video_writer = None

    print(f"OpenCV     : {cv2.__version__}")
    print(f"Kamera     : {camera.description}")
    print(f"Mount      : {config.camera.mount}")
    print(f"Dictionary : {config.aruco.dictionary}")
    print(f"Allowed ID : {list(config.aruco.allowed_ids)}")
    print("Program berjalan. Tekan Q/ESC atau Ctrl+C untuk berhenti.")

    frame_count = 0
    fps = 0.0
    fps_alpha = 0.10
    previous_frame_time = time.perf_counter()
    next_json_time = 0.0
    next_console_time = 0.0

    try:
        while True:
            frame = camera.read()
            frame_count += 1

            now = time.perf_counter()
            frame_delta = max(now - previous_frame_time, 1e-6)
            instant_fps = 1.0 / frame_delta
            fps = instant_fps if fps == 0 else (
                (1.0 - fps_alpha) * fps + fps_alpha * instant_fps
            )
            previous_frame_time = now

            detections, _ = detector.detect(
                frame,
                allow_any_id=args.allow_any_id,
            )

            draw_crosshair(frame)
            draw_detections(frame, detections)
            draw_status(
                frame,
                camera_name=config.camera.name,
                camera_mount=config.camera.mount,
                dictionary_name=config.aruco.dictionary,
                fps=fps,
                detection_count=len(detections),
            )

            if video_writer is None and config.output.save_video_path:
                video_writer = create_video_writer(
                    config.output.save_video_path,
                    frame.shape[1],
                    frame.shape[0],
                    config.camera.fps,
                )

            if video_writer is not None:
                video_writer.write(frame)

            if now >= next_json_time:
                payload = {
                    "timestamp": utc_now_iso(),
                    "camera": {
                        "name": config.camera.name,
                        "mount": config.camera.mount,
                        "width": int(frame.shape[1]),
                        "height": int(frame.shape[0]),
                    },
                    "dictionary": config.aruco.dictionary,
                    "fps": round(fps, 3),
                    "detections": [
                        detection.to_dict() for detection in detections
                    ],
                }

                if jsonl_file is not None:
                    jsonl_file.write(
                        json.dumps(payload, ensure_ascii=False) + "\n"
                    )

                next_json_time = now + 1.0 / config.output.json_rate_hz

            if now >= next_console_time:
                if detections:
                    summary = ", ".join(
                        (
                            f"{item.waypoint}"
                            f"(ex={item.error_x_normalized:+.2f},"
                            f" ey={item.error_y_normalized:+.2f})"
                        )
                        for item in detections
                    )
                    print(f"[DETECTED] {summary}")
                else:
                    print("[SEARCHING] Belum ada waypoint ArUco.")
                next_console_time = now + 1.0

            if config.output.display:
                cv2.imshow("Dirgantara UMM - ArUco Waypoint Detection", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break

            if args.max_frames > 0 and frame_count >= args.max_frames:
                break

    except KeyboardInterrupt:
        print("\nDihentikan pengguna.")
    except RuntimeError as error:
        print(f"\nERROR: {error}", file=sys.stderr)
        return 2
    finally:
        camera.release()
        if video_writer is not None:
            video_writer.release()
        if jsonl_file is not None:
            jsonl_file.close()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
