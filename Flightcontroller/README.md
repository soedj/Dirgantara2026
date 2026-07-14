# Flightcontroller

Bridge MAVLink untuk membaca telemetri Cube/Pixhawk (ArduPilot), menyimpan
state terbaru, dan menyediakannya ke BaseStation melalui REST dan WebSocket.

## Endpoint

- `GET /health`
- `GET /api/telemetry/latest`
- `GET /api/status-text`
- `WS /ws/telemetry`

## Menjalankan

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
cp .env.example .env
python scripts/detect_port.py
python scripts/test_connection.py
python -m app.main
```

Dokumentasi API lokal tersedia di `http://127.0.0.1:8000/docs`.

## Pemeriksaan IMU, gyroscope, compass, dan GPS

Hentikan service lain yang sedang memakai port Pixhawk, lalu jalankan:

```bash
source .venv/bin/activate
python scripts/check_sensors.py --duration 30
```

Selama pengujian:

1. Miringkan wahana ke depan/belakang.
2. Roll ke kiri/kanan.
3. Putar yaw secara perlahan.
4. Untuk memperoleh GPS 3D fix, lakukan pengujian Here3 di luar ruangan.

Override port bila diperlukan:

```bash
python scripts/check_sensors.py \
  --port /dev/ttyACM0 \
  --baud 115200 \
  --duration 30
```

Checklist konfigurasi Here3 terdapat di `docs/HERE3_CHECKLIST.md`.
