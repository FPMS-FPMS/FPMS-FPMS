#!/usr/bin/env bash
# Bundle each Lambda under cloud/lambda/<fn>/ into build/<fn>.zip.
#
# Usage: cloud/lambda/package.sh [function_name]
#   With no arg, packages every function directory.
#
# We keep this dependency-light: if requirements.txt has anything beyond
# comments, we pip-install it into a temp dir first. Otherwise we just zip
# the source.

set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
build_dir="$here/build"
mkdir -p "$build_dir"

package_one() {
  local fn="$1"
  local src="$here/$fn"
  local zip="$build_dir/$fn.zip"

  if [[ ! -d "$src" ]]; then
    echo "no such lambda: $fn" >&2
    return 1
  fi

  echo "==> packaging $fn"
  rm -f "$zip"

  local staging
  staging="$(mktemp -d)"
  trap "rm -rf '$staging'" RETURN

  # Copy source (only .py, .json).
  find "$src" -maxdepth 1 -type f \( -name '*.py' -o -name '*.json' \) \
    -exec cp {} "$staging/" \;

  # Install deps if requirements.txt has any non-comment, non-empty lines.
  if grep -qE '^[^#[:space:]]' "$src/requirements.txt" 2>/dev/null; then
    echo "    pip install -r requirements.txt"
    python3 -m pip install --quiet --target "$staging" -r "$src/requirements.txt"
  fi

  ( cd "$staging" && zip -qr "$zip" . )
  echo "    $zip ($(du -h "$zip" | cut -f1))"
}

if [[ $# -gt 0 ]]; then
  package_one "$1"
else
  for dir in "$here"/*/; do
    fn="$(basename "$dir")"
    [[ "$fn" == "build" ]] && continue
    package_one "$fn"
  done
fi
