from __future__ import annotations

import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from app.config import settings  # noqa: E402
from app.mavlink_client import MavlinkClient  # noqa: E402


def main() -> None:
    client = MavlinkClient(settings)

    try:
        heartbeat = client.connect()
        print(
            "Terhubung:",
            f"system={heartbeat.get_srcSystem()}",
            f"component={heartbeat.get_srcComponent()}",
            f"port={settings.mavlink_connection}",
        )

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            message = client.receive(timeout_s=1.0)
            if message is None or message.get_type() == "BAD_DATA":
                continue
            print(message)
    except KeyboardInterrupt:
        print("\nDihentikan pengguna.")
    finally:
        client.close()


if __name__ == "__main__":
    main()
