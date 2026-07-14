#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec .venv/bin/python -m app.main \
  --config config/jetson_csi_downward.json "$@"
