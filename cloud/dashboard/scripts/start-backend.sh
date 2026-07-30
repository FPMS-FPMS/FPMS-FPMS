#!/usr/bin/env bash
# Start the FPMS dashboard backend on :8000.
# Creates a venv the first time and re-uses it thereafter.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
VENV="$ROOT/.venv"

if [ ! -d "$VENV" ]; then
  echo "→ creating venv at $VENV"
  python3 -m venv "$VENV"
fi

# shellcheck disable=SC1090
source "$VENV/bin/activate" 2>/dev/null || source "$VENV/Scripts/activate"

pip install --upgrade pip >/dev/null
pip install -r "$ROOT/backend/requirements.txt"

export FPMS_MQTT_HOST="${FPMS_MQTT_HOST:-localhost}"
export FPMS_MQTT_PORT="${FPMS_MQTT_PORT:-1883}"
export FPMS_AWS_ENDPOINT="${FPMS_AWS_ENDPOINT:-http://localhost:4566}"
export FPMS_BIND_HOST="${FPMS_BIND_HOST:-0.0.0.0}"
export FPMS_BIND_PORT="${FPMS_BIND_PORT:-8000}"

cd "$ROOT"
echo "→ backend on http://$FPMS_BIND_HOST:$FPMS_BIND_PORT  (MQTT $FPMS_MQTT_HOST:$FPMS_MQTT_PORT, IoT/S3 $FPMS_AWS_ENDPOINT)"
exec uvicorn backend.main:app --host "$FPMS_BIND_HOST" --port "$FPMS_BIND_PORT" --reload
