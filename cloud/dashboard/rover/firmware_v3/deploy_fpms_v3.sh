#!/bin/bash
# =============================================================================
# FPMS V3 firmware -- install into ~/lino and BUILD. Runs ON THE ORANGE PI.
#
# This script does NOT flash and does NOT drive. It only puts files in place
# and produces a .bin. Flashing is a separate, deliberate act performed by the
# operator with the wheels off the ground (see flash_fpms_v3.sh).
#
# It is idempotent: safe to run repeatedly.
#
#   scp -r firmware_v3 ubuntu@<pi>:~/
#   ssh ubuntu@<pi> 'bash ~/firmware_v3/deploy_fpms_v3.sh'
# =============================================================================
set -euo pipefail

LINO="${LINO:-$HOME/lino}"
SRC="$(cd "$(dirname "$0")" && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"

echo "=== FPMS V3 deploy ==="
echo "source: $SRC"
echo "tree:   $LINO"

[ -d "$LINO/firmware" ] || { echo "FATAL: $LINO/firmware not found"; exit 1; }

# --- 1. Back up anything we are about to touch. -----------------------------
# ~/lino has been described as 'dirty and unbacked-up' more than once in this
# project. Never overwrite in it without a copy.
BK="$HOME/lino_backup_$STAMP"
mkdir -p "$BK"
for f in config/config.h config/custom/fpms_config.h firmware/platformio.ini; do
    [ -f "$LINO/$f" ] && { mkdir -p "$BK/$(dirname "$f")"; cp -a "$LINO/$f" "$BK/$f"; }
done
echo "backup: $BK"

# --- 2. Install the config and the firmware source. -------------------------
mkdir -p "$LINO/config/custom"
cp -v "$SRC/fpms_config.h" "$LINO/config/custom/fpms_config.h"

# fpms_main.cpp REPLACES firmware.cpp for our env only. PlatformIO compiles
# everything in src/, so upstream's firmware.cpp must be excluded from OUR
# build rather than deleted -- deleting it would break every other env and
# lose the ability to rebuild the old firmware.
mkdir -p "$LINO/firmware/src"
cp -v "$SRC/fpms_main.cpp" "$LINO/firmware/src/fpms_main.cpp"

# --- 3. Hook the config into config/config.h (upstream's own mechanism). -----
if ! grep -q "USE_FPMS_CONFIG" "$LINO/config/config.h"; then
    # Insert at the top so it wins before any fallback to lino_base_config.h.
    sed -i '1i #ifdef USE_FPMS_CONFIG\n#include "custom/fpms_config.h"\n#endif' \
        "$LINO/config/config.h"
    echo "config.h: added USE_FPMS_CONFIG hook"
else
    echo "config.h: hook already present"
fi

# --- 4. Append the build env if it is not there yet. ------------------------
PIO="$LINO/firmware/platformio.ini"
if ! grep -q '^\[env:fpms\]' "$PIO"; then
    printf '\n' >> "$PIO"
    cat "$SRC/fpms_platformio_env.ini" >> "$PIO"
    echo "platformio.ini: appended [env:fpms]"
else
    echo "platformio.ini: [env:fpms] already present"
fi

# Exclude upstream's firmware.cpp from OUR env only, and only ours.
if ! grep -q 'build_src_filter' "$PIO"; then
    printf '%s\n' \
      '' \
      '; Our env compiles fpms_main.cpp INSTEAD of upstream firmware.cpp.' \
      '; Both define setup()/loop(), so compiling both is a duplicate-symbol' \
      '; link error. Scoped to [env:fpms]; every other env is untouched.' \
      'build_src_filter = +<*> -<firmware.cpp>' >> "$PIO"
    echo "platformio.ini: added build_src_filter"
fi

# --- 5. PIN micro_ros_platformio to the commit that currently works. --------
# Rationale: pinning to a hash someone guessed is not safer than not pinning.
# Pinning to the hash that is ALREADY CHECKED OUT AND BUILDING is.
LIBDIR=$(find "$LINO/firmware/.pio/libdeps" -maxdepth 2 -type d -name 'micro_ros_platformio' 2>/dev/null | head -1 || true)
if [ -n "$LIBDIR" ] && [ -d "$LIBDIR/.git" ]; then
    SHA=$(git -C "$LIBDIR" rev-parse HEAD 2>/dev/null || true)
    if [ -n "$SHA" ] && ! grep -q "micro_ros_platformio#" "$PIO"; then
        sed -i "s|https://github.com/micro-ROS/micro_ros_platformio$|https://github.com/micro-ROS/micro_ros_platformio#$SHA|" "$PIO"
        echo "pinned micro_ros_platformio -> $SHA"
    fi
else
    echo "WARNING: micro_ros_platformio not yet checked out; cannot pin."
    echo "         Re-run this script AFTER the first successful build to pin it."
fi

# --- 6. Build. --------------------------------------------------------------
# Resolve pio EXPLICITLY. A non-interactive ssh shell does not source the
# profile, so PATH lacks ~/.platformio/penv/bin and a bare `pio` fails with
# "command not found" -- after steps 1-5 have already modified ~/lino. That is
# the worst place to stop: the tree is half-updated and the error looks like a
# missing install rather than a missing PATH.
PIO_BIN="${PIO_BIN:-}"
if [ -z "$PIO_BIN" ]; then
    for c in "$HOME/.platformio/penv/bin/pio" "$HOME/.local/bin/pio" "$(command -v pio 2>/dev/null || true)"; do
        [ -n "$c" ] && [ -x "$c" ] && { PIO_BIN="$c"; break; }
    done
fi
[ -n "$PIO_BIN" ] || { echo "FATAL: pio not found. Set PIO_BIN=/path/to/pio"; exit 1; }
echo "pio:    $PIO_BIN"

cd "$LINO/firmware"
echo "=== building (env: fpms) ==="
"$PIO_BIN" run -e fpms

BIN="$LINO/firmware/.pio/build/fpms/firmware.bin"
if [ -f "$BIN" ]; then
    echo
    echo "=== BUILD OK ==="
    ls -l "$BIN"
    md5sum "$BIN"
    echo
    echo "NEXT: do NOT flash yet. Read the on-blocks verification procedure,"
    echo "put the rover on blocks, then run flash_fpms_v3.sh."
else
    echo "BUILD FAILED: no firmware.bin"; exit 1
fi
