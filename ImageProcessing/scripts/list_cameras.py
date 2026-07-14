from __future__ import annotations

from pathlib import Path


def main() -> None:
    devices = sorted(Path("/dev").glob("video*"))
    if not devices:
        print("Tidak ada /dev/video* yang terdeteksi.")
        return

    print("Camera device yang terdeteksi:")
    for device in devices:
        print(f"- {device}")


if __name__ == "__main__":
    main()
