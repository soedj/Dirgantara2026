# Checklist Here3 + Cube Orange

## Hubungan data

Ubuntu tidak membaca Here3 secara langsung. Alurnya:

```text
Here3 --DroneCAN--> Cube Orange/Pixhawk --MAVLink USB/UART--> Ubuntu/Jetson
```

Dalam penggunaan ArduPilot normal:

- Data accelerometer dan gyroscope MAVLink umumnya berasal dari IMU internal
  Cube Orange.
- Data GPS berasal dari Here3 apabila Here3 aktif sebagai GPS DroneCAN.
- Data magnetometer/compass berasal dari Here3 apabila compass bertipe UAVCAN
  terdeteksi, dikalibrasi, diaktifkan, dan ditempatkan pada prioritas pertama.

## Pemeriksaan Mission Planner

1. Buka `Setup > Optional Hardware > DroneCAN/UAVCAN`.
2. Pastikan node Here3 terlihat.
3. Periksa parameter CAN pada port yang dipakai:
   - `CAN_P1_DRIVER = 1` untuk CAN1, atau `CAN_P2_DRIVER = 1` untuk CAN2.
   - Driver DroneCAN terkait menggunakan protocol `1`.
4. Periksa tipe GPS:
   - Firmware lama dapat menampilkan `GPS_TYPE = 9`.
   - Firmware lebih baru dapat menampilkan `GPS1_TYPE = 9`.
5. Buka `Setup > Mandatory Hardware > Compass`.
6. Pastikan external compass memiliki BusType `UAVCAN`.
7. Pindahkan compass UAVCAN ke prioritas pertama, kemudian kalibrasi compass.
8. Reboot flight controller.

Jangan mengubah parameter keselamatan hanya untuk membuat pengujian sensor
berjalan. Lepaskan propeller selama pemeriksaan meja.
