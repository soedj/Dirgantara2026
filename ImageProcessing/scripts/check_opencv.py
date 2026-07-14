from __future__ import annotations

import argparse
import re
import sys

import cv2


def build_flag(name: str) -> str:
    pattern = rf"^\s*{re.escape(name)}\s*:\s*(.+)$"
    for line in cv2.getBuildInformation().splitlines():
        match = re.match(pattern, line)
        if match:
            return match.group(1).strip()
    return "UNKNOWN"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero jika bukan OpenCV 4.11.x.",
    )
    args = parser.parse_args()

    version = cv2.__version__
    major_minor = ".".join(version.split(".")[:2])
    has_aruco = hasattr(cv2, "aruco")
    has_detector = has_aruco and hasattr(cv2.aruco, "ArucoDetector")

    print("PEMERIKSAAN OPENCV")
    print("=" * 60)
    print(f"Version             : {version}")
    print(f"Target              : 4.11.x")
    print(f"cv2.aruco           : {'YES' if has_aruco else 'NO'}")
    print(f"ArucoDetector API   : {'YES' if has_detector else 'NO'}")
    print(f"GStreamer           : {build_flag('GStreamer')}")
    print(f"Video4Linux/V4L2    : {build_flag('v4l/v4l2')}")
    print(f"NVIDIA CUDA         : {build_flag('NVIDIA CUDA')}")
    try:
        cuda_devices = cv2.cuda.getCudaEnabledDeviceCount()
    except Exception:
        cuda_devices = 0
    print(f"CUDA device count   : {cuda_devices}")
    print("=" * 60)

    ok = major_minor == "4.11" and has_detector
    print(f"HASIL: {'LULUS' if ok else 'BELUM SESUAI'}")

    if args.strict and not ok:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
