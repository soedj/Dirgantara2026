from __future__ import annotations

import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from pymavlink import mavutil

from app.config import settings
from app.mavlink_client import MavlinkClient


def pressure_to_relative_altitude(
    pressure_hpa: float,
    reference_pressure_hpa: float,
) -> float:
    """
    Mengubah perubahan tekanan menjadi ketinggian relatif
    terhadap tekanan awal program.
    """
    if pressure_hpa <= 0 or reference_pressure_hpa <= 0:
        return 0.0

    return 44330.0 * (
        1.0 - (pressure_hpa / reference_pressure_hpa) ** 0.190294957
    )


def main() -> None:
    client = MavlinkClient(settings)

    reference_pressure: float | None = None
    last_pressure_time = 0.0
    received_pressure = False

    try:
        print(
            f"Menghubungkan ke {settings.mavlink_connection} "
            f"@ {settings.mavlink_baud}..."
        )

        heartbeat = client.connect()

        print(
            "Terhubung ke Pixhawk:",
            f"system={heartbeat.get_srcSystem()}",
            f"component={heartbeat.get_srcComponent()}",
        )

        requested_messages = (
            (
                mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
                10,
            ),
            (
                mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
                10,
            ),
            (
                mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                5,
            ),
            (
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                5,
            ),
        )

        for message_id, rate_hz in requested_messages:
            try:
                client.request_message_interval(message_id, rate_hz)
            except Exception as error:
                print(
                    f"Peringatan: gagal meminta message "
                    f"{message_id}: {error}"
                )

        print()
        print("Lepaskan propeller sebelum pengujian.")
        print("Diamkan Pixhawk beberapa detik, lalu angkat 1–2 meter.")
        print("Jangan meniup langsung lubang atau busa barometer.")
        print("-" * 72)

        while True:
            message = client.receive(timeout_s=1.0)

            if message is None:
                if time.monotonic() - last_pressure_time > 5:
                    print("Menunggu data tekanan dari Pixhawk...")
                continue

            message_type = message.get_type()

            if message_type == "BAD_DATA":
                continue

            if message_type == "SCALED_PRESSURE":
                pressure = float(message.press_abs)
                temperature = float(message.temperature) / 100.0

                if (
                    reference_pressure is None
                    and math.isfinite(pressure)
                    and pressure > 0
                ):
                    reference_pressure = pressure

                relative_altitude = (
                    pressure_to_relative_altitude(
                        pressure,
                        reference_pressure,
                    )
                    if reference_pressure is not None
                    else 0.0
                )

                received_pressure = True
                last_pressure_time = time.monotonic()

                print(
                    f"BAROMETER | "
                    f"pressure={pressure:8.3f} hPa | "
                    f"temperature={temperature:6.2f} °C | "
                    f"relative_alt={relative_altitude:7.3f} m"
                )

            elif message_type == "HIGHRES_IMU":
                absolute_pressure = float(message.abs_pressure)
                pressure_altitude = float(message.pressure_alt)

                if absolute_pressure > 0:
                    print(
                        f"HIGHRES   | "
                        f"abs_pressure={absolute_pressure:8.3f} hPa | "
                        f"pressure_alt={pressure_altitude:8.3f} m"
                    )

            elif message_type == "VFR_HUD":
                print(
                    f"EKF/HUD   | "
                    f"alt={float(message.alt):8.3f} m | "
                    f"climb={float(message.climb):7.3f} m/s"
                )

            elif message_type == "GLOBAL_POSITION_INT":
                relative_altitude = (
                    float(message.relative_alt) / 1000.0
                )

                print(
                    f"POSITION  | "
                    f"relative_alt={relative_altitude:8.3f} m"
                )

    except KeyboardInterrupt:
        print("\n" + "=" * 72)

        if received_pressure:
            print("HASIL: DATA BAROMETER BERHASIL DITERIMA.")
        else:
            print("HASIL: DATA BAROMETER BELUM DITERIMA.")

    finally:
        client.close()


if __name__ == "__main__":
    main()
