#!/bin/bash
# Slow forward crawl + encoder readout. Open-loop only.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
PENV="$HOME/.platformio/penv"

# Nothing may hold the port or publish commands.
S systemctl stop micro-ros-agent fpms-missions fpms-teleop 2>/dev/null
sleep 2

mkdir -p /home/ubuntu/crawl/src
cp /home/ubuntu/fwd800_main.cpp /home/ubuntu/crawl/src/main.cpp
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
lib_deps = madhephaestus/ESP32Encoder
build_flags = -D __PGMSPACE_H_
EOF

cd /home/ubuntu/crawl || exit 1
"$PENV/bin/pio" run 2>&1 | tail -3
"$PENV/bin/pio" run -t upload 2>&1 | tail -3

echo ""
echo ">>> 5s PAUSE, THEN FORWARD 80cm THEN BACK 80cm, THEN IT STOPS <<<"
python3 - <<'EOF'
import time, serial
ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=0.3)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.2); ser.setRTS(False)
time.sleep(0.05); ser.reset_input_buffer()
t0 = time.time(); buf = b""
while time.time() - t0 < 75.0:
    c = ser.read(4096)
    if c: buf += c
ser.close()
print(buf.decode("utf-8", "replace"))
EOF
echo "=== DONE - motors off ==="
