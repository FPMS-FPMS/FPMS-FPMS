"""What is actually on the wire? Sample several bauds and histogram the bytes.

A wrong baud produces broadly-distributed garbage; a stuck line produces one
dominant value. Distinguishing those two decides whether to chase the transport
config or the hardware.
"""
import collections
import time

import serial

PORT = "/dev/ttyUSB1"

for baud in (921600, 115200, 460800):
    try:
        ser = serial.Serial(PORT, baud, timeout=0.2)
    except Exception as e:
        print(f"{baud}: open failed {e}")
        continue
    ser.setDTR(False)
    ser.setRTS(True)
    time.sleep(0.15)
    ser.setRTS(False)
    time.sleep(0.05)
    ser.reset_input_buffer()

    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 4.0:
        c = ser.read(4096)
        if c:
            buf.extend(c)
    ser.close()

    hist = collections.Counter(buf)
    top = hist.most_common(5)
    printable = sum(1 for b in buf if 32 <= b < 127 or b in (9, 10, 13))
    print(f"\n=== baud {baud}: {len(buf)} bytes, printable {printable} "
          f"({100.0 * printable / len(buf) if buf else 0:.1f}%), "
          f"{len(hist)} distinct values ===")
    print("  top values:", [(hex(v), n) for v, n in top])
    print("  first 48:", buf[:48].hex(" "))
    if printable > 20:
        txt = "".join(chr(b) if 32 <= b < 127 else "." for b in buf[:400])
        print("  as text:", txt)
