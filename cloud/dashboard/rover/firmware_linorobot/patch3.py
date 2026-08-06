"""Tell the build this board has 4 MB of flash, not 8 MB.

The esp32-s3-devkitc-1 board definition declares 8 MB. This board has 4 MB
(md5-verified 4194304-byte stock dump). The mismatch is written into the image
header, so esp_flash's probe fails at do_core_init and the board panics and
reboots forever -- transmitting nothing but reset noise, which is why the
micro-ROS agent never saw a session.
"""
import pathlib
import re
import sys

ini = pathlib.Path("/home/ubuntu/lino/firmware/platformio.ini")
txt = ini.read_text()

if "board_upload.flash_size" in txt:
    print("platformio.ini: already patched")
    sys.exit(0)

m = re.search(r"^\[env:fpms\]$", txt, re.M)
if not m:
    sys.exit("platformio.ini: [env:fpms] not found")

insert = (
    "\n"
    "; This board is 4MB, not the 8MB the devkitc-1 definition assumes.\n"
    "board_upload.flash_size = 4MB\n"
    "board_upload.maximum_size = 4194304\n"
    "board_build.flash_size = 4MB\n"
    "; default.csv fits a 4MB layout; the 8MB default would not.\n"
    "board_build.partitions = default.csv\n"
)
idx = m.end()
txt = txt[:idx] + insert + txt[idx:]
ini.write_text(txt)
print("platformio.ini: pinned flash size to 4MB")
print("PATCH3 OK")
