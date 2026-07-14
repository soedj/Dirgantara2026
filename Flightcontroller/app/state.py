from __future__ import annotations

import copy
import threading
from datetime import datetime, timezone
from typing import Any


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class TelemetryState:
    """Penyimpanan status telemetri terbaru yang aman dipakai lintas-thread."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._data: dict[str, Any] = {
            "timestamp": utc_now_iso(),
            "connection": {
                "connected": False,
                "port": None,
                "baud": None,
                "last_message_at": None,
                "last_heartbeat_at": None,
                "error": None,
            },
            "vehicle": {
                "system_id": None,
                "component_id": None,
                "vehicle_type": None,
                "autopilot": None,
                "flight_mode": "UNKNOWN",
                "armed": False,
                "system_status": None,
                "landed_state": "UNKNOWN",
            },
            "attitude": {
                "roll_deg": None,
                "pitch_deg": None,
                "yaw_deg": None,
                "rollspeed_rad_s": None,
                "pitchspeed_rad_s": None,
                "yawspeed_rad_s": None,
            },
            "global_position": {
                "latitude_deg": None,
                "longitude_deg": None,
                "altitude_msl_m": None,
                "relative_altitude_m": None,
                "heading_deg": None,
            },
            "local_position_ned": {
                "x_m": None,
                "y_m": None,
                "z_m": None,
                "vx_m_s": None,
                "vy_m_s": None,
                "vz_m_s": None,
            },
            "velocity": {
                "groundspeed_m_s": None,
                "airspeed_m_s": None,
                "climb_m_s": None,
                "vx_m_s": None,
                "vy_m_s": None,
                "vz_m_s": None,
            },
            "gps": {
                "fix_type": None,
                "satellites_visible": None,
                "hdop": None,
                "vdop": None,
            },
            "battery": {
                "voltage_v": None,
                "current_a": None,
                "remaining_percent": None,
            },
            "navigation": {
                "mission_seq": None,
                "target_bearing_deg": None,
                "nav_bearing_deg": None,
                "waypoint_distance_m": None,
                "altitude_error_m": None,
                "airspeed_error_m_s": None,
                "cross_track_error_m": None,
            },
            "ekf": {
                "flags": None,
                "velocity_variance": None,
                "horizontal_position_variance": None,
                "vertical_position_variance": None,
                "compass_variance": None,
                "terrain_altitude_variance": None,
            },
            "rc": {
                "rssi_raw": None,
                "rssi_percent": None,
                "channels": {},
            },
            "servo_outputs": {},
            "distance_sensor": {
                "current_distance_m": None,
                "min_distance_m": None,
                "max_distance_m": None,
                "orientation": None,
            },
            "status_text": [],
        }

    def update_section(self, section: str, values: dict[str, Any]) -> None:
        with self._lock:
            target = self._data.setdefault(section, {})
            if not isinstance(target, dict):
                raise TypeError(f"Bagian state {section!r} bukan dictionary")
            target.update(values)
            self._data["timestamp"] = utc_now_iso()

    def set_connection(
        self,
        *,
        connected: bool,
        port: str | None = None,
        baud: int | None = None,
        error: str | None = None,
        heartbeat: bool = False,
    ) -> None:
        now = utc_now_iso()
        with self._lock:
            connection = self._data["connection"]
            connection["connected"] = connected
            if port is not None:
                connection["port"] = port
            if baud is not None:
                connection["baud"] = baud
            connection["error"] = error
            if connected:
                connection["last_message_at"] = now
            if heartbeat:
                connection["last_heartbeat_at"] = now
            self._data["timestamp"] = now

    def mark_message_received(self, *, heartbeat: bool = False) -> None:
        now = utc_now_iso()
        with self._lock:
            self._data["connection"]["connected"] = True
            self._data["connection"]["last_message_at"] = now
            self._data["connection"]["error"] = None
            if heartbeat:
                self._data["connection"]["last_heartbeat_at"] = now
            self._data["timestamp"] = now

    def add_status_text(self, severity: int, text: str, max_items: int = 30) -> None:
        item = {
            "timestamp": utc_now_iso(),
            "severity": severity,
            "text": text,
        }
        with self._lock:
            messages = self._data["status_text"]
            messages.append(item)
            if len(messages) > max_items:
                del messages[:-max_items]
            self._data["timestamp"] = item["timestamp"]

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._data)
