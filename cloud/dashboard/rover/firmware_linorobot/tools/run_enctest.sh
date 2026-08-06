#!/bin/bash
# Build, flash and read the encoder polarity test.
# Standalone project so the mission firmware tree is untouched.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
PENV="$HOME/.platformio/penv"

mkdir -p /home/ubuntu/enctest/src
cp /home/ubuntu/enc_main.cpp /home/ubuntu/enctest/src/main.cpp
cat > /home/ubuntu/enctest/platformio.ini <<'EOF'
[env:enctest]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
upload_port = /dev/ttyUSB1
upload_speed = 460800
; This board is 4MB, not the 8MB the devkitc-1 definition assumes.
board_upload.flash_size = 4MB
board_build.flash_size = 4MB
board_build.partitions = default.csv
lib_deps = madhephaestus/ESP32Encoder
build_flags = -D __PGMSPACE_H_
EOF

S systemctl stop micro-ros-agent
S systemctl stop fpms-missions 2>/dev/null
S systemctl stop fpms-teleop 2>/dev/null
sleep 2

cd /home/ubuntu/enctest || exit 1
echo "=== BUILD ==="
"$PENV/bin/pio" run 2>&1 | tail -8
echo "=== FLASH ==="
"$PENV/bin/pio" run -t upload 2>&1 | tail -5

echo "=== READING TEST OUTPUT (the rover will twitch each wheel) ==="
python3 - <<'EOF'
import time, serial
ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=0.3)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.2); ser.setRTS(False)
time.sleep(0.05); ser.reset_input_buffer()
t0 = time.time(); buf = b""
while time.time() - t0 < 40.0:
    c = ser.read(4096)
    if c: buf += c
ser.close()
print(buf.decode("utf-8", "replace"))
EOF
echo "=== ENCTEST DONE ==="
