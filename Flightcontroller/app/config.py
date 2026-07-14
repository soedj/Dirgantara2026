from __future__ import annotations

import os
from dataclasses import dataclass

from dotenv import load_dotenv

load_dotenv()


def _get_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"{name} harus berupa bilangan bulat, didapat: {value!r}") from exc


def _get_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError as exc:
        raise ValueError(f"{name} harus berupa angka, didapat: {value!r}") from exc


@dataclass(frozen=True)
class Settings:
    mavlink_connection: str = os.getenv("MAVLINK_CONNECTION", "/dev/ttyACM0")
    mavlink_baud: int = _get_int("MAVLINK_BAUD", 115200)
    mavlink_source_system: int = _get_int("MAVLINK_SOURCE_SYSTEM", 245)
    mavlink_source_component: int = _get_int("MAVLINK_SOURCE_COMPONENT", 191)
    heartbeat_timeout_s: float = _get_float("HEARTBEAT_TIMEOUT_S", 10.0)
    reconnect_delay_s: float = _get_float("RECONNECT_DELAY_S", 2.0)
    link_timeout_s: float = _get_float("LINK_TIMEOUT_S", 5.0)

    api_host: str = os.getenv("API_HOST", "0.0.0.0")
    api_port: int = _get_int("API_PORT", 8000)
    websocket_rate_hz: float = _get_float("WEBSOCKET_RATE_HZ", 5.0)


settings = Settings()
