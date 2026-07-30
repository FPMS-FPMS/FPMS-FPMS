#!/usr/bin/env bash
# Start the Vite dev server bound to 0.0.0.0 so LAN peers can join.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
FRONT="$(cd "$HERE/../frontend" && pwd)"

cd "$FRONT"
if [ ! -d node_modules ]; then
  echo "→ npm install (first run)"
  npm install
fi

export VITE_BACKEND_URL="${VITE_BACKEND_URL:-http://localhost:8000}"
echo "→ frontend on http://0.0.0.0:5173  (proxies /api and /ws to $VITE_BACKEND_URL)"
exec npm run dev -- --host 0.0.0.0 --port 5173
