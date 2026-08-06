#!/bin/bash
# Is the board still talking after the session went quiet?
# Capture WITHOUT resetting first -- a reset would hide a post-boot crash by
# restarting the very thing that died.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }

S systemctl stop micro-ros-agent
sleep 2

echo "=== STEADY STATE, NO RESET, 921600, 10s ==="
python3 - <<'EOF'
import collections, time, serial
ser = serial.Serial("/dev/ttyUSB1", 921600, timeout=0.2)
t0 = time.time(); buf = bytearray()
while time.time() - t0 < 10.0:
    c = ser.read(4096)
    if c: buf.extend(c)
ser.close()
h = collections.Counter(buf)
pr = sum(1 for b in buf if 32 <= b < 127)
print(f"bytes={len(buf)} printable={pr} distinct={len(h)}")
print("top:", [(hex(v), n) for v, n in h.most_common(6)])
print("first64:", buf[:64].hex(" "))
txt = "".join(chr(b) if 32 <= b < 127 else "." for b in buf[:600])
print("text:", txt)
EOF

echo "=== AFTER RESET, 115200 (panic/boot log baud), 8s ==="
python3 - <<'EOF'
import time, serial
ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=0.2)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.15); ser.setRTS(False)
time.sleep(0.05); ser.reset_input_buffer()
t0 = time.time(); buf = bytearray()
while time.time() - t0 < 8.0:
    c = ser.read(4096)
    if c: buf.extend(c)
ser.close()
txt = "".join(chr(b) if 32 <= b < 127 or b in (10,13) else "." for b in buf)
keep = [l for l in txt.splitlines() if sum(c.isalnum() for c in l) > 6]
print("\n".join(keep[:30]) if keep else "(no readable boot text)")
EOF

S systemctl start micro-ros-agent
sleep 45
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "=== PUBLISHER COUNT AFTER RECONNECT ==="
timeout 20 ros2 topic info /odom_raw 2>&1
echo "=== DIAG DONE ==="
