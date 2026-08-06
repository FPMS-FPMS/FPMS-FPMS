#!/bin/bash
# Flash and run the simple forward crawl. Nothing else runs.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
PENV="$HOME/.platformio/penv"

# Nothing may hold the serial port or publish commands.
S systemctl stop micro-ros-agent fpms-missions fpms-teleop 2>/dev/null
sleep 2

mkdir -p /home/ubuntu/crawl/src
cp /home/ubuntu/crawl_main.cpp /home/ubuntu/crawl/src/main.cpp
cat > /home/ubuntu/crawl/platformio.ini <<'EOF'
[env:crawl]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
upload_port = /dev/ttyUSB1
upload_speed = 460800
board_upload.flash_size = 4MB
board_build.flash_size = 4MB
board_build.partitions = default.csv
build_flags = -D __PGMSPACE_H_
EOF

cd /home/ubuntu/crawl || exit 1
echo "=== BUILD ==="
"$PENV/bin/pio" run 2>&1 | tail -5
echo "=== FLASH ==="
"$PENV/bin/pio" run -t upload 2>&1 | tail -4
echo ""
echo ">>> 6 SECOND PAUSE, THEN ALL FOUR MOTORS FORWARD SLOWLY FOR 6 SECONDS <<<"
echo ">>> WATCH IT NOW <<<"
sleep 18
echo "=== MOVEMENT FINISHED - motors are off and will stay off ==="
