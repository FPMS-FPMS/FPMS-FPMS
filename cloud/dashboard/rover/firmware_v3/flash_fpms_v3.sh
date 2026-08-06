#!/bin/bash
# =============================================================================
# FPMS V3 -- FLASH and ROLLBACK. Runs ON THE ORANGE PI.
#
# *** THE OPERATOR RUNS THIS, WATCHING, WITH THE WHEELS OFF THE GROUND. ***
#
#   bash flash_fpms_v3.sh backup    # dump current 4MB flash first (do this)
#   bash flash_fpms_v3.sh flash     # write FPMS V3
#   bash flash_fpms_v3.sh rollback  # restore the full stock 4MB dump
#   bash flash_fpms_v3.sh rollback-app   # restore only the factory app image
#
# Nothing happens without one of those words. There is no default action, on
# purpose.
# =============================================================================
set -euo pipefail

# The system python3 esptool. The PlatformIO venv has NO esptool module --
# `python -m esptool` from inside the penv fails, which has wasted time before.
ESPTOOL="${ESPTOOL:-/home/ubuntu/.platformio/packages/tool-esptoolpy/esptool.py}"
PY="${PY:-/usr/bin/python3}"

# BY-PATH, NEVER BY-ID. There are TWO CP2102s on this Pi -- the board and the
# LiDAR -- and they share ID_SERIAL "0001", so /dev/serial/by-id cannot tell
# them apart. by-path is tied to the physical USB port and can.
#   board = usb-0:1.3      lidar = usb-0:1.2   <-- do not mix these up
# CORRECTED 2026-08-06. The old default was
#   /dev/serial/by-path/platform-xhci-hcd.0-usb-0:1.3:1.0
# which DOES NOT EXIST on this Pi -- the controller is fc880000.usb, not
# xhci-hcd.0, and the by-path names carry a "-port0" suffix. esptool would have
# failed to open the port. The value below is the device the WORKING
# micro-ros-agent is bound to (fpms-uros-agent-run:17), which is the
# authoritative answer rather than the one in a comment.
#   board = usb-0:1.3   lidar = usb-0:1.2   <-- do not mix these up
PORT="${PORT:-/dev/serial/by-path/platform-fc880000.usb-usb-0:1.3:1.0-port0}"
[ -e "$PORT" ] || { echo "FATAL: $PORT does not exist. Check: ls /dev/serial/by-path/"; exit 1; }

BUILD="${BUILD:-$HOME/lino/firmware/.pio/build/fpms}"
STOCK_FULL="${STOCK_FULL:-/home/ubuntu/stock_backup_preflash.bin}"
STOCK_APP="${STOCK_APP:-/home/ubuntu/microROS_Robot_V2.0.0.bin}"
CHIP="esp32s3"
BAUD="921600"

need() { [ -e "$1" ] || { echo "FATAL: missing $1"; exit 1; }; }

stop_consumers() {
    # Anything holding the tty will make esptool fail to sync, and the agent
    # will fight for the port mid-flash.
    echo "-- stopping ROS consumers of the board tty"
    sudo systemctl stop micro-ros-agent fpms-uros-supervisor fpms-missions \
        fpms-teleop fpms-odom-tf 2>/dev/null || true
    sleep 1
}

case "${1:-}" in

backup)
    need "$ESPTOOL"
    stop_consumers
    OUT="$HOME/flash_backup_$(date +%Y%m%d-%H%M%S).bin"
    echo "-- reading entire 4MB flash to $OUT"
    "$PY" "$ESPTOOL" --chip "$CHIP" --port "$PORT" --baud "$BAUD" \
        read_flash 0x0 0x400000 "$OUT"
    ls -l "$OUT"; md5sum "$OUT"
    echo "KEEP THIS FILE. It is the only complete rollback for the state you"
    echo "are in right now."
    ;;

flash)
    need "$ESPTOOL"
    need "$BUILD/firmware.bin"
    need "$BUILD/bootloader.bin"
    need "$BUILD/partitions.bin"
    stop_consumers
    echo "-- flashing FPMS V3"
    ls -l "$BUILD/firmware.bin"; md5sum "$BUILD/firmware.bin"
    # ESP32-S3 puts the bootloader at 0x0 (NOT 0x1000 -- that is the older
    # ESP32). Wrong offset gives a board that never prints anything.
    "$PY" "$ESPTOOL" --chip "$CHIP" --port "$PORT" --baud "$BAUD" \
        --before default_reset --after hard_reset \
        write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB \
        0x0     "$BUILD/bootloader.bin" \
        0x8000  "$BUILD/partitions.bin" \
        0xe000  "$HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin" \
        0x10000 "$BUILD/firmware.bin"
    echo
    echo "=== FLASHED. WHEELS MUST STILL BE OFF THE GROUND. ==="
    echo "Now: start the agent, then run onblocks_check.py. Do not put the"
    echo "rover on the floor until all four wheels pass."
    ;;

rollback)
    need "$ESPTOOL"; need "$STOCK_FULL"
    stop_consumers
    echo "-- restoring FULL 4MB stock dump: $STOCK_FULL"
    md5sum "$STOCK_FULL"
    "$PY" "$ESPTOOL" --chip "$CHIP" --port "$PORT" --baud "$BAUD" \
        --before default_reset --after hard_reset \
        write_flash -z --flash_size 4MB 0x0 "$STOCK_FULL"
    echo "=== ROLLED BACK to the pre-flash board state. ==="
    echo "REMEMBER: the factory firmware needs the OLD Pi settings back --"
    echo "agent baud, ODOM_POSE_SIGN=-1, UInt16 /battery. See the change table."
    ;;

rollback-app)
    need "$ESPTOOL"; need "$STOCK_APP"
    stop_consumers
    # Use this only if the full dump is unavailable. It rewrites the app slot
    # but leaves whatever bootloader/partition table is currently on the board,
    # so it is a weaker guarantee than `rollback`.
    echo "-- restoring factory APP image only: $STOCK_APP"
    md5sum "$STOCK_APP"    # expect c71e8826669acd0f0e13e30640402254
    "$PY" "$ESPTOOL" --chip "$CHIP" --port "$PORT" --baud "$BAUD" \
        --before default_reset --after hard_reset \
        write_flash -z --flash_size 4MB 0x10000 "$STOCK_APP"
    echo "=== FACTORY APP RESTORED (bootloader/partitions untouched). ==="
    ;;

*)
    sed -n '2,20p' "$0"
    exit 1
    ;;
esac
