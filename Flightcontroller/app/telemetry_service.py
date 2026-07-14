from __future__ import annotations

import math
import threading
import time
from typing import Any

from pymavlink import mavutil

from .config import Settings
from .mavlink_client import MavlinkClient
from .state import TelemetryState


LANDED_STATE_NAMES = {
    0: "UNDEFINED",
    1: "ON_GROUND",
    2: "IN_AIR",
    3: "TAKEOFF",
    4: "LANDING",
}


class TelemetryService:
    """Membaca MAVLink pada background thread dan memperbarui state terbaru."""

    def __init__(self, settings: Settings, state: TelemetryState) -> None:
        self.settings = settings
        self.state = state
        self.client = MavlinkClient(settings)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="mavlink-telemetry",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self.client.close()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.state.set_connection(
                    connected=False,
                    port=self.settings.mavlink_connection,
                    baud=self.settings.mavlink_baud,
                    error=None,
                )

                heartbeat = self.client.connect()
                self.state.set_connection(
                    connected=True,
                    port=self.settings.mavlink_connection,
                    baud=self.settings.mavlink_baud,
                    heartbeat=True,
                )
                self._handle_message(heartbeat)
                self._request_streams()

                last_rx_monotonic = time.monotonic()

                while not self._stop_event.is_set():
                    message = self.client.receive(timeout_s=1.0)

                    if message is None:
                        if (
                            time.monotonic() - last_rx_monotonic
                            > self.settings.link_timeout_s
                        ):
                            raise ConnectionError(
                                f"Tidak ada paket MAVLink selama "
                                f"{self.settings.link_timeout_s:.1f} detik"
                            )
                        continue

                    message_type = message.get_type()
                    if message_type == "BAD_DATA":
                        continue

                    last_rx_monotonic = time.monotonic()
                    self.state.mark_message_received(
                        heartbeat=message_type == "HEARTBEAT"
                    )
                    self._handle_message(message)

            except Exception as exc:
                self.state.set_connection(
                    connected=False,
                    port=self.settings.mavlink_connection,
                    baud=self.settings.mavlink_baud,
                    error=f"{type(exc).__name__}: {exc}",
                )
                self.client.close()
                self._stop_event.wait(self.settings.reconnect_delay_s)

        self.client.close()

    def _request_streams(self) -> None:
        # Daftar ini cukup untuk dashboard, debugging misi, dan integrasi visi awal.
        message_rates = (
            (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 5),
        )

        for message_id, rate_hz in message_rates:
            try:
                self.client.request_message_interval(message_id, rate_hz)
                time.sleep(0.02)
            except Exception as exc:
                self.state.add_status_text(
                    4,
                    f"Gagal meminta message ID {message_id} @ {rate_hz} Hz: {exc}",
                )

    @staticmethod
    def _enum_name(enum_name: str, value: int) -> str | int:
        try:
            entry = mavutil.mavlink.enums[enum_name][value]
            return entry.name
        except (KeyError, TypeError, AttributeError):
            return value

    @staticmethod
    def _valid_scaled(value: int | float, invalid: int | float, scale: float) -> float | None:
        if value == invalid:
            return None
        return float(value) / scale

    @staticmethod
    def _rad_to_deg(value: float) -> float:
        return math.degrees(float(value))

    def _handle_message(self, msg: Any) -> None:
        message_type = msg.get_type()

        if message_type == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
            armed = bool(
                int(msg.base_mode)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self.state.update_section(
                "vehicle",
                {
                    "system_id": int(msg.get_srcSystem()),
                    "component_id": int(msg.get_srcComponent()),
                    "vehicle_type": self._enum_name(
                        "MAV_TYPE", int(msg.type)
                    ),
                    "autopilot": self._enum_name(
                        "MAV_AUTOPILOT", int(msg.autopilot)
                    ),
                    "flight_mode": mode,
                    "armed": armed,
                    "system_status": self._enum_name(
                        "MAV_STATE", int(msg.system_status)
                    ),
                },
            )
            return

        if message_type == "SYS_STATUS":
            voltage = None if int(msg.voltage_battery) == 65535 else int(msg.voltage_battery) / 1000.0
            current = None if int(msg.current_battery) == -1 else int(msg.current_battery) / 100.0
            remaining = None if int(msg.battery_remaining) == -1 else int(msg.battery_remaining)
            self.state.update_section(
                "battery",
                {
                    "voltage_v": voltage,
                    "current_a": current,
                    "remaining_percent": remaining,
                },
            )
            return

        if message_type == "ATTITUDE":
            self.state.update_section(
                "attitude",
                {
                    "roll_deg": self._rad_to_deg(msg.roll),
                    "pitch_deg": self._rad_to_deg(msg.pitch),
                    "yaw_deg": self._rad_to_deg(msg.yaw),
                    "rollspeed_rad_s": float(msg.rollspeed),
                    "pitchspeed_rad_s": float(msg.pitchspeed),
                    "yawspeed_rad_s": float(msg.yawspeed),
                },
            )
            return

        if message_type == "GLOBAL_POSITION_INT":
            heading = None if int(msg.hdg) == 65535 else int(msg.hdg) / 100.0
            vx = int(msg.vx) / 100.0
            vy = int(msg.vy) / 100.0
            vz = int(msg.vz) / 100.0

            self.state.update_section(
                "global_position",
                {
                    "latitude_deg": int(msg.lat) / 10_000_000.0,
                    "longitude_deg": int(msg.lon) / 10_000_000.0,
                    "altitude_msl_m": int(msg.alt) / 1000.0,
                    "relative_altitude_m": int(msg.relative_alt) / 1000.0,
                    "heading_deg": heading,
                },
            )
            self.state.update_section(
                "velocity",
                {
                    "vx_m_s": vx,
                    "vy_m_s": vy,
                    "vz_m_s": vz,
                },
            )
            return

        if message_type == "LOCAL_POSITION_NED":
            self.state.update_section(
                "local_position_ned",
                {
                    "x_m": float(msg.x),
                    "y_m": float(msg.y),
                    "z_m": float(msg.z),
                    "vx_m_s": float(msg.vx),
                    "vy_m_s": float(msg.vy),
                    "vz_m_s": float(msg.vz),
                },
            )
            return

        if message_type == "GPS_RAW_INT":
            eph = int(msg.eph)
            epv = int(msg.epv)
            self.state.update_section(
                "gps",
                {
                    "fix_type": int(msg.fix_type),
                    "satellites_visible": int(msg.satellites_visible),
                    "hdop": None if eph == 65535 else eph / 100.0,
                    "vdop": None if epv == 65535 else epv / 100.0,
                },
            )
            return

        if message_type == "VFR_HUD":
            self.state.update_section(
                "velocity",
                {
                    "groundspeed_m_s": float(msg.groundspeed),
                    "airspeed_m_s": float(msg.airspeed),
                    "climb_m_s": float(msg.climb),
                },
            )
            return

        if message_type == "EXTENDED_SYS_STATE":
            landed_state = int(msg.landed_state)
            self.state.update_section(
                "vehicle",
                {
                    "landed_state": LANDED_STATE_NAMES.get(
                        landed_state, str(landed_state)
                    )
                },
            )
            return

        if message_type == "MISSION_CURRENT":
            self.state.update_section(
                "navigation",
                {"mission_seq": int(msg.seq)},
            )
            return

        if message_type == "NAV_CONTROLLER_OUTPUT":
            self.state.update_section(
                "navigation",
                {
                    "target_bearing_deg": int(msg.target_bearing),
                    "nav_bearing_deg": int(msg.nav_bearing),
                    "waypoint_distance_m": int(msg.wp_dist),
                    "altitude_error_m": float(msg.alt_error),
                    "airspeed_error_m_s": float(msg.aspd_error),
                    "cross_track_error_m": float(msg.xtrack_error),
                },
            )
            return

        if message_type == "EKF_STATUS_REPORT":
            self.state.update_section(
                "ekf",
                {
                    "flags": int(msg.flags),
                    "velocity_variance": float(msg.velocity_variance),
                    "horizontal_position_variance": float(msg.pos_horiz_variance),
                    "vertical_position_variance": float(msg.pos_vert_variance),
                    "compass_variance": float(msg.compass_variance),
                    "terrain_altitude_variance": float(msg.terrain_alt_variance),
                },
            )
            return

        if message_type == "RC_CHANNELS":
            rssi_raw = int(msg.rssi)
            rssi_percent = (
                None
                if rssi_raw == 255
                else round(max(0, min(254, rssi_raw)) * 100.0 / 254.0, 1)
            )
            channels: dict[str, int | None] = {}
            for channel_number in range(1, 19):
                field = f"chan{channel_number}_raw"
                value = getattr(msg, field, 65535)
                channels[str(channel_number)] = (
                    None if int(value) == 65535 else int(value)
                )

            self.state.update_section(
                "rc",
                {
                    "rssi_raw": rssi_raw,
                    "rssi_percent": rssi_percent,
                    "channels": channels,
                },
            )
            return

        if message_type == "SERVO_OUTPUT_RAW":
            outputs: dict[str, int] = {}
            for channel_number in range(1, 17):
                field = f"servo{channel_number}_raw"
                if hasattr(msg, field):
                    outputs[str(channel_number)] = int(getattr(msg, field))
            self.state.update_section("servo_outputs", outputs)
            return

        if message_type == "DISTANCE_SENSOR":
            self.state.update_section(
                "distance_sensor",
                {
                    "current_distance_m": int(msg.current_distance) / 100.0,
                    "min_distance_m": int(msg.min_distance) / 100.0,
                    "max_distance_m": int(msg.max_distance) / 100.0,
                    "orientation": int(msg.orientation),
                },
            )
            return

        if message_type == "STATUSTEXT":
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            self.state.add_status_text(
                int(msg.severity),
                str(text).replace("\x00", "").strip(),
            )
