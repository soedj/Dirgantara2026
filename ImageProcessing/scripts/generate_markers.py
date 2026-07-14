from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Membuat marker uji WP1-WP4 KRTI."
    )
    parser.add_argument("--output", default="markers")
    parser.add_argument("--pixels", type=int, default=1000)
    parser.add_argument("--margin", type=int, default=120)
    args = parser.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_7X7_50
    )
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    for marker_id in range(1, 5):
        marker = cv2.aruco.generateImageMarker(
            dictionary,
            marker_id,
            args.pixels,
            borderBits=1,
        )
        canvas = np.full(
            (
                args.pixels + 2 * args.margin,
                args.pixels + 2 * args.margin,
            ),
            255,
            dtype=np.uint8,
        )
        canvas[
            args.margin:args.margin + args.pixels,
            args.margin:args.margin + args.pixels,
        ] = marker

        path = output / f"WP{marker_id}_DICT_7X7_50_ID{marker_id}.png"
        if not cv2.imwrite(str(path), canvas):
            raise RuntimeError(f"Gagal menyimpan {path}")
        print(f"Dibuat: {path}")


if __name__ == "__main__":
    main()
