from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time
from collections import Counter, defaultdict, deque
from dataclasses import replace
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from pymavlink import mavutil  # noqa: E402

from app.config import settings as default_settings  # noqa: E402
from app.mavlink_client import MavlinkClient  # noqa: E402


G = 9.80665
GPS_FIX_NAMES = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
    7: "STATIC",
    8: "PPP",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Memeriksa apakah Ubuntu menerima accelerometer/IMU, gyroscope, "
            "magnetometer/compass, attitude, dan GPS dari Pixhawk melalui MAVLink."
        )
    )
    parser.add_argument(
        "--port",
        help="Override port dari .env, contoh /dev/ttyACM0",
    )
    parser.add_argument(
        "--baud",
        type=int,
        help="Override baud dari .env, contoh 115200",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Durasi pengujian dalam detik (default: 30)",
    )
    parser.add_argument(
        "--refresh",
        type=float,
        default=0.5,
        help="Interval refresh tampilan terminal dalam detik (default: 0.5)",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Jangan membersihkan layar setiap refresh",
    )
    return parser.parse_args()


def safe_rate(times: deque[float]) -> float:
    if len(times) < 2:
        return 0.0
    elapsed = times[-1] - times[0]
    return 0.0 if elapsed <= 0 else (len(times) - 1) / elapsed


def fmt(value: float | int | None, digits: int = 3) -> str:
    if value is None:
        return "-"
    if isinstance(value, int):
        return str(value)
    return f"{value:.{digits}f}"


def span(values: list[float]) -> float:
    return 0.0 if len(values) < 2 else max(values) - min(values)


def angle_span_deg(values: list[float]) -> float:
    """Rentang sudut dengan menghindari masalah wrap -180/180."""
    if len(values) < 2:
        return 0.0
    radians = [math.radians(value) for value in values]
    unwrapped = [radians[0]]
    for value in radians[1:]:
        previous = unwrapped[-1]
        delta = (value - previous + math.pi) % (2 * math.pi) - math.pi
        unwrapped.append(previous + delta)
    degrees = [math.degrees(value) for value in unwrapped]
    return max(degrees) - min(degrees)


def request_streams(client: MavlinkClient) -> None:
    message_rates = (
        (mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, 20),
        (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, 20),
        (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, 5),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5),
        (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5),
    )

    for message_id, rate_hz in message_rates:
        try:
            client.request_message_interval(message_id, rate_hz)
        except Exception:
            # Beberapa firmware dapat mengabaikan message tertentu.
            pass
        time.sleep(0.02)


def scaled_imu_to_si(msg: Any) -> dict[str, Any]:
    return {
        "source": msg.get_type(),
        "id": {
            "SCALED_IMU": 0,
            "SCALED_IMU2": 1,
            "SCALED_IMU3": 2,
        }.get(msg.get_type()),
        "accel": (
            float(msg.xacc) * G / 1000.0,
            float(msg.yacc) * G / 1000.0,
            float(msg.zacc) * G / 1000.0,
        ),
        "gyro": (
            float(msg.xgyro) / 1000.0,
            float(msg.ygyro) / 1000.0,
            float(msg.zgyro) / 1000.0,
        ),
        "mag": (
            float(msg.xmag) / 1000.0,
            float(msg.ymag) / 1000.0,
            float(msg.zmag) / 1000.0,
        ),
        "temperature_c": (
            None
            if not hasattr(msg, "temperature") or int(msg.temperature) == 0
            else float(msg.temperature) / 100.0
        ),
        "scaled": True,
    }


def highres_imu_to_si(msg: Any) -> dict[str, Any]:
    return {
        "source": "HIGHRES_IMU",
        "id": int(getattr(msg, "id", 0)),
        "accel": (float(msg.xacc), float(msg.yacc), float(msg.zacc)),
        "gyro": (float(msg.xgyro), float(msg.ygyro), float(msg.zgyro)),
        "mag": (float(msg.xmag), float(msg.ymag), float(msg.zmag)),
        "temperature_c": float(msg.temperature),
        "scaled": True,
    }


def raw_imu(msg: Any) -> dict[str, Any]:
    return {
        "source": "RAW_IMU",
        "id": int(getattr(msg, "id", 0)),
        "accel": (float(msg.xacc), float(msg.yacc), float(msg.zacc)),
        "gyro": (float(msg.xgyro), float(msg.ygyro), float(msg.zgyro)),
        "mag": (float(msg.xmag), float(msg.ymag), float(msg.zmag)),
        "temperature_c": (
            None
            if not hasattr(msg, "temperature") or int(msg.temperature) == 0
            else float(msg.temperature) / 100.0
        ),
        "scaled": False,
    }


def choose_primary_imu(instances: dict[str, dict[str, Any]]) -> dict[str, Any] | None:
    # Prioritas SI unit, lalu IMU instance pertama.
    priorities = (
        "HIGHRES_IMU:0",
        "SCALED_IMU:0",
        "SCALED_IMU2:1",
        "SCALED_IMU3:2",
        "RAW_IMU:0",
    )
    for key in priorities:
        if key in instances:
            return instances[key]
    return next(iter(instances.values()), None)


def status_label(received: bool, ready: bool = True) -> str:
    if not received:
        return "[BELUM ADA DATA]"
    return "[OK]" if ready else "[DATA MASUK, BELUM READY]"


def clear_screen() -> None:
    print("\033[2J\033[H", end="")


def render(
    *,
    port: str,
    baud: int,
    elapsed: float,
    duration: float,
    counts: Counter[str],
    rates: dict[str, deque[float]],
    imu_instances: dict[str, dict[str, Any]],
    attitude: dict[str, float] | None,
    gps: dict[str, Any] | None,
    gps2: dict[str, Any] | None,
    global_position: dict[str, Any] | None,
    history: dict[str, list[float]],
    status_text: str | None,
) -> None:
    imu = choose_primary_imu(imu_instances)
    imu_received = imu is not None
    accel = imu["accel"] if imu else (None, None, None)
    gyro = imu["gyro"] if imu else (None, None, None)
    mag = imu["mag"] if imu else (None, None, None)
    mag_norm = (
        math.sqrt(sum(float(value) ** 2 for value in mag))
        if imu_received
        else None
    )
    compass_ready = imu_received and mag_norm is not None and mag_norm > 1e-6

    gps_received = gps is not None
    gps_ready = gps_received and int(gps["fix_type"]) >= 3
    fix_name = GPS_FIX_NAMES.get(int(gps["fix_type"]), str(gps["fix_type"])) if gps else "-"

    print("KRTI VTOL - PEMERIKSAAN SENSOR PIXHAWK / HERE3")
    print("=" * 72)
    print(f"Port        : {port} @ {baud}")
    print(f"Waktu uji   : {elapsed:5.1f} / {duration:.1f} detik")
    print("Gerakkan wahana perlahan: tilt depan-belakang, roll kiri-kanan, lalu yaw.")
    print("Lakukan uji GPS di luar ruangan dengan pandangan langit terbuka.")
    print("-" * 72)

    heartbeat_ok = counts["HEARTBEAT"] > 0
    print(f"MAVLink     : {status_label(heartbeat_ok)}  HEARTBEAT={counts['HEARTBEAT']}")
    print()

    print(f"IMU/ACCEL   : {status_label(imu_received)}")
    if imu:
        unit = "m/s²" if imu["scaled"] else "RAW"
        print(
            f"  sumber    : {imu['source']} instance={imu['id']} "
            f"rate={safe_rate(rates[imu['source']]):.1f} Hz"
        )
        print(
            f"  accel XYZ : {fmt(accel[0])}, {fmt(accel[1])}, {fmt(accel[2])} {unit}"
        )
        print(f"  temperatur: {fmt(imu['temperature_c'], 2)} °C")
    print()

    print(f"GYROSCOPE   : {status_label(imu_received)}")
    if imu:
        unit = "rad/s" if imu["scaled"] else "RAW"
        print(
            f"  gyro XYZ  : {fmt(gyro[0])}, {fmt(gyro[1])}, {fmt(gyro[2])} {unit}"
        )
        print(
            "  perubahan : "
            f"X={span(history['gx']):.3f}, "
            f"Y={span(history['gy']):.3f}, "
            f"Z={span(history['gz']):.3f}"
        )
    print()

    print(f"COMPASS/MAG : {status_label(imu_received, compass_ready)}")
    if imu:
        unit = "gauss" if imu["scaled"] else "RAW"
        print(
            f"  mag XYZ   : {fmt(mag[0])}, {fmt(mag[1])}, {fmt(mag[2])} {unit}"
        )
        print(f"  magnitude : {fmt(mag_norm)} {unit}")
        print(
            "  perubahan : "
            f"X={span(history['mx']):.3f}, "
            f"Y={span(history['my']):.3f}, "
            f"Z={span(history['mz']):.3f}"
        )
    print()

    attitude_received = attitude is not None
    print(f"ATTITUDE    : {status_label(attitude_received)}")
    if attitude:
        print(
            "  R/P/Y     : "
            f"{attitude['roll_deg']:.2f}°, "
            f"{attitude['pitch_deg']:.2f}°, "
            f"{attitude['yaw_deg']:.2f}°"
        )
        print(f"  perubahan yaw selama uji: {angle_span_deg(history['yaw']):.2f}°")
    print()

    print(f"GPS HERE3   : {status_label(gps_received, gps_ready)}")
    if gps:
        print(
            f"  fix       : {fix_name} ({gps['fix_type']}) | "
            f"satelit={gps['satellites']} | HDOP={fmt(gps['hdop'], 2)}"
        )
        print(
            f"  posisi    : lat={gps['lat']:.7f}, lon={gps['lon']:.7f}, "
            f"alt={gps['alt_m']:.2f} m"
        )
        print(
            f"  speed/COG : {fmt(gps['speed_m_s'], 2)} m/s | "
            f"{fmt(gps['cog_deg'], 2)}°"
        )
        print(f"  rate      : {safe_rate(rates['GPS_RAW_INT']):.1f} Hz")
    if gps2:
        print(
            f"  GPS2      : fix={GPS_FIX_NAMES.get(gps2['fix_type'], gps2['fix_type'])}, "
            f"satelit={gps2['satellites']}"
        )
    print()

    if global_position:
        print(
            "ESTIMASI EKF: "
            f"lat={global_position['lat']:.7f}, "
            f"lon={global_position['lon']:.7f}, "
            f"heading={fmt(global_position['heading_deg'], 2)}°"
        )

    if imu_instances:
        print()
        print("Instance IMU MAVLink yang terdeteksi:")
        for key, value in sorted(imu_instances.items()):
            print(
                f"  - {key}: count={counts[value['source']]}, "
                f"rate={safe_rate(rates[value['source']]):.1f} Hz"
            )

    if status_text:
        print()
        print(f"STATUSTEXT terakhir: {status_text}")

    print("=" * 72)
    print("Ctrl+C untuk menghentikan.")


def build_summary(
    counts: Counter[str],
    imu_instances: dict[str, dict[str, Any]],
    attitude: dict[str, float] | None,
    gps: dict[str, Any] | None,
    history: dict[str, list[float]],
) -> int:
    imu = choose_primary_imu(imu_instances)
    imu_ok = imu is not None
    compass_ok = False
    if imu:
        compass_ok = math.sqrt(sum(value * value for value in imu["mag"])) > 1e-6

    gps_data_ok = gps is not None
    gps_fix_ok = gps_data_ok and int(gps["fix_type"]) >= 3
    attitude_ok = attitude is not None
    heartbeat_ok = counts["HEARTBEAT"] > 0

    gyro_changed = max(
        span(history["gx"]),
        span(history["gy"]),
        span(history["gz"]),
    ) > 0.02
    mag_changed = max(
        span(history["mx"]),
        span(history["my"]),
        span(history["mz"]),
    ) > 0.005
    yaw_changed = angle_span_deg(history["yaw"]) > 3.0

    print("\nHASIL AKHIR")
    print("=" * 72)
    print(f"MAVLink/HEARTBEAT : {'LULUS' if heartbeat_ok else 'GAGAL'}")
    print(f"IMU/Accelerometer : {'LULUS' if imu_ok else 'GAGAL'}")
    print(
        "Gyroscope         : "
        + (
            "LULUS, data berubah saat digerakkan"
            if imu_ok and gyro_changed
            else "DATA MASUK, tetapi perubahan belum terdeteksi"
            if imu_ok
            else "GAGAL"
        )
    )
    print(
        "Compass/Magnetometer: "
        + (
            "LULUS, data berubah saat diputar"
            if compass_ok and (mag_changed or yaw_changed)
            else "DATA MASUK, tetapi perubahan belum terdeteksi"
            if compass_ok
            else "GAGAL"
        )
    )
    print(f"Attitude R/P/Y    : {'LULUS' if attitude_ok else 'GAGAL'}")
    print(
        "GPS data          : "
        + (
            "LULUS, 3D fix atau lebih baik"
            if gps_fix_ok
            else "DATA MASUK, tetapi belum 3D fix"
            if gps_data_ok
            else "GAGAL"
        )
    )
    print("=" * 72)

    if imu_ok and not gyro_changed:
        print("Catatan: ulangi sambil memiringkan dan memutar wahana.")
    if gps_data_ok and not gps_fix_ok:
        print("Catatan: pindahkan Here3 ke luar ruangan dan tunggu beberapa menit.")
    if not gps_data_ok:
        print("Catatan: periksa CAN/DroneCAN, GPS_TYPE/GPS1_TYPE, dan kabel Here3.")

    required_ok = heartbeat_ok and imu_ok and compass_ok and attitude_ok and gps_data_ok
    return 0 if required_ok else 2


def main() -> int:
    args = parse_args()

    if args.duration <= 0:
        raise ValueError("--duration harus lebih besar dari 0")
    if args.refresh <= 0:
        raise ValueError("--refresh harus lebih besar dari 0")

    settings = replace(
        default_settings,
        mavlink_connection=args.port or default_settings.mavlink_connection,
        mavlink_baud=args.baud or default_settings.mavlink_baud,
    )
    client = MavlinkClient(settings)

    counts: Counter[str] = Counter()
    rates: dict[str, deque[float]] = defaultdict(lambda: deque(maxlen=100))
    imu_instances: dict[str, dict[str, Any]] = {}
    attitude: dict[str, float] | None = None
    gps: dict[str, Any] | None = None
    gps2: dict[str, Any] | None = None
    global_position: dict[str, Any] | None = None
    status_text: str | None = None
    history: dict[str, list[float]] = defaultdict(list)

    try:
        print(
            f"Menghubungkan ke {settings.mavlink_connection} "
            f"@ {settings.mavlink_baud}..."
        )
        heartbeat = client.connect()
        counts["HEARTBEAT"] += 1
        rates["HEARTBEAT"].append(time.monotonic())
        print(
            "Terhubung ke Pixhawk: "
            f"system={heartbeat.get_srcSystem()}, "
            f"component={heartbeat.get_srcComponent()}"
        )
        request_streams(client)

        start = time.monotonic()
        next_render = start

        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= args.duration:
                break

            msg = client.receive(timeout_s=0.2)
            now = time.monotonic()

            if msg is not None and msg.get_type() != "BAD_DATA":
                msg_type = msg.get_type()
                counts[msg_type] += 1
                rates[msg_type].append(now)

                if msg_type == "HIGHRES_IMU":
                    data = highres_imu_to_si(msg)
                    key = f"HIGHRES_IMU:{data['id']}"
                    imu_instances[key] = data

                elif msg_type in {"SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"}:
                    data = scaled_imu_to_si(msg)
                    key = f"{msg_type}:{data['id']}"
                    imu_instances[key] = data

                elif msg_type == "RAW_IMU":
                    data = raw_imu(msg)
                    key = f"RAW_IMU:{data['id']}"
                    imu_instances[key] = data

                elif msg_type == "ATTITUDE":
                    attitude = {
                        "roll_deg": math.degrees(float(msg.roll)),
                        "pitch_deg": math.degrees(float(msg.pitch)),
                        "yaw_deg": math.degrees(float(msg.yaw)),
                    }
                    history["yaw"].append(attitude["yaw_deg"])

                elif msg_type in {"GPS_RAW_INT", "GPS2_RAW"}:
                    eph = int(msg.eph)
                    velocity = int(msg.vel)
                    cog = int(msg.cog)
                    item = {
                        "fix_type": int(msg.fix_type),
                        "lat": int(msg.lat) / 10_000_000.0,
                        "lon": int(msg.lon) / 10_000_000.0,
                        "alt_m": int(msg.alt) / 1000.0,
                        "hdop": None if eph == 65535 else eph / 100.0,
                        "speed_m_s": None if velocity == 65535 else velocity / 100.0,
                        "cog_deg": None if cog == 65535 else cog / 100.0,
                        "satellites": (
                            None
                            if int(msg.satellites_visible) == 255
                            else int(msg.satellites_visible)
                        ),
                    }
                    if msg_type == "GPS_RAW_INT":
                        gps = item
                    else:
                        gps2 = item

                elif msg_type == "GLOBAL_POSITION_INT":
                    heading = int(msg.hdg)
                    global_position = {
                        "lat": int(msg.lat) / 10_000_000.0,
                        "lon": int(msg.lon) / 10_000_000.0,
                        "heading_deg": None if heading == 65535 else heading / 100.0,
                    }

                elif msg_type == "STATUSTEXT":
                    text = msg.text
                    if isinstance(text, bytes):
                        text = text.decode("utf-8", errors="replace")
                    status_text = str(text).replace("\x00", "").strip()

                selected = choose_primary_imu(imu_instances)
                if selected is not None:
                    ax, ay, az = selected["accel"]
                    gx, gy, gz = selected["gyro"]
                    mx, my, mz = selected["mag"]
                    for name, value in (
                        ("ax", ax), ("ay", ay), ("az", az),
                        ("gx", gx), ("gy", gy), ("gz", gz),
                        ("mx", mx), ("my", my), ("mz", mz),
                    ):
                        history[name].append(float(value))
                        if len(history[name]) > 1000:
                            del history[name][:-1000]

            if now >= next_render:
                if not args.no_clear and os.isatty(sys.stdout.fileno()):
                    clear_screen()
                render(
                    port=settings.mavlink_connection,
                    baud=settings.mavlink_baud,
                    elapsed=elapsed,
                    duration=args.duration,
                    counts=counts,
                    rates=rates,
                    imu_instances=imu_instances,
                    attitude=attitude,
                    gps=gps,
                    gps2=gps2,
                    global_position=global_position,
                    history=history,
                    status_text=status_text,
                )
                next_render = now + args.refresh

    except KeyboardInterrupt:
        print("\nPengujian dihentikan pengguna.")
    except PermissionError as exc:
        print(f"\nGagal membuka serial port: {exc}", file=sys.stderr)
        print(
            "Pastikan user sudah masuk grup dialout dan login ulang: "
            "sudo usermod -aG dialout $USER",
            file=sys.stderr,
        )
        return 3
    except OSError as exc:
        print(f"\nKesalahan serial: {exc}", file=sys.stderr)
        print(
            "Tutup Mission Planner/QGroundControl/app.main karena satu serial port "
            "tidak boleh dipakai dua proses sekaligus.",
            file=sys.stderr,
        )
        return 3
    except Exception as exc:
        print(f"\nPengujian gagal: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 3
    finally:
        client.close()

    return build_summary(counts, imu_instances, attitude, gps, history)


if __name__ == "__main__":
    raise SystemExit(main())
