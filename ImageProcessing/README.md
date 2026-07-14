# ImageProcessing - KRTI 2026 VTOL

Program awal untuk mendeteksi waypoint ArUco WP1-WP4 menggunakan kamera
USB, kamera CSI Jetson, file video, atau pipeline GStreamer.

## Acuan marker KRTI

Konfigurasi bawaan:

- Dictionary: `DICT_7X7_50`
- WP1: ID 1
- WP2: ID 2
- WP3: ID 3
- WP4: ID 4

Panduan menampilkan alas waypoint oranye 2000 mm x 2000 mm serta marker
500 mm x 500 mm dan 100 mm x 100 mm.

## Instalasi Ubuntu 22.04

```bash
cd ImageProcessing_KRTI_VTOL
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements-ubuntu.txt
python scripts/check_opencv.py --strict
```

## Membuat marker uji

```bash
python scripts/generate_markers.py
```

Tampilkan salah satu PNG pada layar atau cetak untuk menguji deteksi.

## Menjalankan kamera USB di Ubuntu

```bash
python -m app.main --config config/ubuntu_usb.json
```

atau:

```bash
./run_ubuntu.sh
```

Keluar dengan tombol `Q`, `ESC`, atau `Ctrl+C`.

## Uji menggunakan video

```bash
python -m app.main \
  --config config/ubuntu_usb.json \
  --input /path/video.mp4
```

## Menjalankan di Jetson Orin Nano

Kode programnya sama. OpenCV pada Jetson harus:

- versi 4.11.x;
- memiliki `cv2.aruco.ArucoDetector`;
- memiliki `GStreamer: YES` bila menggunakan kamera CSI.

Periksa dengan:

```bash
python scripts/check_opencv.py --strict
```

Kamera CSI:

```bash
python -m app.main --config config/jetson_csi_downward.json
```

USB camera pada Jetson:

```bash
python -m app.main --config config/jetson_usb_forward.json
```

Jetson tanpa monitor:

```bash
python -m app.main \
  --config config/jetson_csi_downward.json \
  --headless
```

## Data keluaran

Deteksi disimpan sebagai JSON Lines di folder `output/`. Contoh:

```json
{
  "marker_id": 2,
  "waypoint": "WP2",
  "center_px": {"x": 634.2, "y": 361.5},
  "center_error_normalized": {"x": -0.0091, "y": 0.0042},
  "area_px2": 58234.1,
  "side_length_px": 241.8,
  "rotation_deg": 2.3
}
```

`center_error_normalized` disiapkan untuk integrasi navigasi berikutnya:

- `x < 0`: marker di kiri kamera;
- `x > 0`: marker di kanan kamera;
- `y < 0`: marker di atas pusat gambar;
- `y > 0`: marker di bawah pusat gambar.

Program ini baru melakukan deteksi dan belum mengirim perintah kendali ke
Pixhawk.
