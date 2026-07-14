from __future__ import annotations

from typing import Any

from pymavlink import mavutil

from .config import Settings


class MavlinkClient:
    """Pembungkus koneksi MAVLink agar kode layanan tidak bergantung langsung pada serial."""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.master: Any | None = None

    def connect(self) -> Any:
        self.close()

        self.master = mavutil.mavlink_connection(
            self.settings.mavlink_connection,
            baud=self.settings.mavlink_baud,
            autoreconnect=True,
            source_system=self.settings.mavlink_source_system,
            source_component=self.settings.mavlink_source_component,
        )

        heartbeat = self.master.wait_heartbeat(
            timeout=self.settings.heartbeat_timeout_s
        )
        if heartbeat is None:
            raise TimeoutError(
                f"HEARTBEAT tidak diterima dari {self.settings.mavlink_connection} "
                f"dalam {self.settings.heartbeat_timeout_s:.1f} detik"
            )

        return heartbeat

    @property
    def target_system(self) -> int:
        if self.master is None:
            return 0
        return int(self.master.target_system)

    @property
    def target_component(self) -> int:
        if self.master is None:
            return 0
        return int(self.master.target_component)

    def receive(self, timeout_s: float = 1.0) -> Any | None:
        if self.master is None:
            raise RuntimeError("Koneksi MAVLink belum dibuat")
        return self.master.recv_match(blocking=True, timeout=timeout_s)

    def request_message_interval(self, message_id: int, rate_hz: float) -> None:
        if self.master is None:
            raise RuntimeError("Koneksi MAVLink belum dibuat")
        if rate_hz <= 0:
            raise ValueError("rate_hz harus lebih besar dari 0")

        interval_us = int(1_000_000 / rate_hz)
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def close(self) -> None:
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
            finally:
                self.master = None
