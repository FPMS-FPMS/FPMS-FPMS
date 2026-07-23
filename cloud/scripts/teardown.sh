#!/usr/bin/env bash
# Stop fpms-cloud. By default keeps volumes (S3 data survives restart).
# Pass --wipe to drop the volumes too.

set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
cd "$here/../localstack"

if [[ "${1:-}" == "--wipe" ]]; then
  echo "==> stopping and wiping fpms-cloud volumes"
  docker compose down --volumes
else
  echo "==> stopping fpms-cloud (volumes preserved; pass --wipe to remove)"
  docker compose down
fi
