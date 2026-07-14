from __future__ import annotations

from dataclasses import dataclass

import cv2

from .config import CameraConfig


def build_jetson_csi_pipeline(config: CameraConfig) -> str:
    """
    Pipeline CSI untuk Jetson Orin Nano.

    Pipeline memakai nvarguscamerasrc dan menghasilkan frame BGR yang dapat
    dibaca OpenCV. OpenCV pada Jetson harus dibangun dengan GStreamer=YES.
    """
    return (
        f"nvarguscamerasrc sensor-id={config.sensor_id} ! "
        f"video/x-raw(memory:NVMM), "
        f"width=(int){config.width}, height=(int){config.height}, "
        f"format=(string)NV12, framerate=(fraction){config.fps}/1 ! "
        f"nvvidconv flip-method={config.flip_method} ! "
        f"video/x-raw, width=(int){config.width}, "
        f"height=(int){config.height}, format=(string)BGRx ! "
        f"videoconvert ! video/x-raw, format=(string)BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


@dataclass
class CameraSource:
    config: CameraConfig
    capture: cv2.VideoCapture
    description: str

    @classmethod
    def open(cls, config: CameraConfig) -> "CameraSource":
        source_type = config.source_type

        if source_type == "csi":
            pipeline = config.pipeline or build_jetson_csi_pipeline(config)
            capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            description = f"Jetson CSI sensor-id={config.sensor_id}"

        elif source_type == "gstreamer":
            if not config.pipeline:
                raise ValueError(
                    "camera.pipeline wajib diisi untuk source_type=gstreamer"
                )
            capture = cv2.VideoCapture(config.pipeline, cv2.CAP_GSTREAMER)
            description = "custom GStreamer pipeline"

        elif source_type == "file":
            if not config.input_path:
                raise ValueError(
                    "camera.input_path wajib diisi untuk source_type=file"
                )
            capture = cv2.VideoCapture(config.input_path)
            description = f"file={config.input_path}"

        else:
            # V4L2 dipakai pada Ubuntu dan juga untuk USB camera pada Jetson.
            source: int | str = config.device or config.index
            capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
            capture.set(cv2.CAP_PROP_FPS, config.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            description = f"USB/V4L2 source={source}"

        if not capture.isOpened():
            capture.release()
            raise RuntimeError(
                f"Kamera tidak dapat dibuka ({description}). "
                "Periksa device, permission, pipeline, dan dukungan GStreamer."
            )

        return cls(config=config, capture=capture, description=description)

    def read(self):
        ok, frame = self.capture.read()
        if not ok or frame is None:
            raise RuntimeError(
                f"Gagal membaca frame dari kamera: {self.description}"
            )
        return frame

    def release(self) -> None:
        self.capture.release()
