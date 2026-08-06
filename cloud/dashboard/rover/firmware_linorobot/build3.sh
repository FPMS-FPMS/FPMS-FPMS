#!/bin/bash
# Rebuild with the correct 4MB flash size and reflash.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
PENV="$HOME/.platformio/penv"
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble

python3 /home/ubuntu/patch3.py || exit 1
sed -n '/\[env:fpms\]/,/^$/p' /home/ubuntu/lino/firmware/platformio.ini

S systemctl stop fpms-teleop
S systemctl stop micro-ros-agent
sleep 2

cd /home/ubuntu/lino/firmware || exit 1
echo "=== BUILD3 ==="
"$PENV/bin/pio" run -e fpms 2>&1 | tail -18
echo "=== UPLOAD3 ==="
"$PENV/bin/pio" run -e fpms -t upload 2>&1 | tail -12

echo "=== BOOT CONSOLE (115200, where the panic appeared) ==="
python3 - <<'EOF'
import time, serial
ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=0.2)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.15); ser.setRTS(False)
time.sleep(0.05); ser.reset_input_buffer()
t0 = time.time(); buf = bytearray()
while time.time() - t0 < 6.0:
    c = ser.read(4096)
    if c: buf.extend(c)
ser.close()
txt = "".join(chr(b) if 32 <= b < 127 or b in (10,13) else "." for b in buf)
print(txt[:1200])
EOF

echo "=== RESTART AGENT ==="
S systemctl start micro-ros-agent
sleep 75
journalctl -u micro-ros-agent -n 10 --no-pager 2>&1 | tail -10

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "=== OBSERVE (no motion) ==="
timeout 45 python3 /home/ubuntu/crawl_test.py --mode observe 2>&1 | tail -6
echo "=== BUILD3 DONE ==="
