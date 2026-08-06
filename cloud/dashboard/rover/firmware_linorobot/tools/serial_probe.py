"""Reset the board and capture its boot output on UART0.

The previous session lost hours to a boot loop it could not read. firmware.cpp
treats a failed IMU init as fatal and parks in `while(1)` after printing
"IMU init failed", so the console distinguishes that from a transport problem
in one shot.
"""
import sys
import time

import serial

PORT = "/dev/ttyUSB1"
BAUD = 921600

ser = serial.Serial(PORT, BAUD, timeout=0.2)

# ESP32 auto-reset: RTS drives EN, DTR drives IO0. IO0 high = boot into app.
ser.setDTR(False)
ser.setRTS(True)
time.sleep(0.15)
ser.setRTS(False)
time.sleep(0.05)
ser.reset_input_buffer()

print(f"--- reset, capturing {PORT} @ {BAUD} for 12s ---")
t0 = time.time()
buf = bytearray()
while time.time() - t0 < 12.0:
    chunk = ser.read(4096)
    if chunk:
        buf.extend(chunk)

ser.close()
print(f"--- {len(buf)} bytes ---")

# micro-ROS traffic is binary XRCE-DDS; a fatal-error message is plain ASCII.
text = "".join(chr(b) if 32 <= b < 127 or b in (10, 13) else "." for b in buf)
lines = [ln for ln in text.splitlines() if ln.strip(".") .strip()]
for ln in lines[:40]:
    print("TXT:", ln)

printable = sum(1 for b in buf if 32 <= b < 127 or b in (9, 10, 13))
print(f"--- printable {printable}/{len(buf)} "
      f"({100.0 * printable / len(buf) if buf else 0:.0f}%) ---")
