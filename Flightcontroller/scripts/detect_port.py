from __future__ import annotations

from serial.tools import list_ports


def main() -> None:
    ports = sorted(list_ports.comports(), key=lambda item: item.device)

    if not ports:
        print("Tidak ada serial port yang terdeteksi.")
        return

    for port in ports:
        print(f"Device       : {port.device}")
        print(f"Description  : {port.description}")
        print(f"Manufacturer : {port.manufacturer}")
        print(f"Product      : {port.product}")
        print(f"Serial       : {port.serial_number}")
        print(f"HWID         : {port.hwid}")
        print("-" * 72)


if __name__ == "__main__":
    main()
