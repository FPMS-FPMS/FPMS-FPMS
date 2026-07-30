#!/usr/bin/env bash
# Build the frontend for production. Run this once (or after any src/ change
# in the frontend) and the backend will serve the built app at /.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
FRONT="$(cd "$HERE/../frontend" && pwd)"

cd "$FRONT"
if [ ! -d node_modules ]; then
  echo "→ npm install"
  npm install
fi

echo "→ vite build"
npm run build

echo "✓ built to $FRONT/dist"
