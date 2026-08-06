"""Wire the FPMS config into the upstream linorobot2 tree.

Kept as an idempotent script rather than hand edits so the whole firmware can
be rebuilt from a fresh clone without anyone remembering what was changed.
"""
import pathlib
import sys

LINO = pathlib.Path("/home/ubuntu/lino")

# 1. dispatcher entry -----------------------------------------------------
cfg = LINO / "config/config.h"
txt = cfg.read_text()
if "USE_FPMS_CONFIG" not in txt:
    anchor = "// add user configurations above this line"
    if anchor not in txt:
        sys.exit("config.h: user-config anchor missing, upstream layout changed")
    txt = txt.replace(
        anchor,
        '#ifdef USE_FPMS_CONFIG\n    #include "custom/fpms_config.h"\n#endif\n\n' + anchor,
    )
    cfg.write_text(txt)
    print("config.h: added USE_FPMS_CONFIG include")
else:
    print("config.h: already patched")

# 2. build environment ----------------------------------------------------
# Upstream's esp32s3 env targets a board with native USB (/dev/ttyACM0 and
# ARDUINO_USB_CDC_ON_BOOT). This board reaches the host through an external
# CP2102 on UART0 -- proven by esptool's DTR/RTS auto-reset working on
# ttyUSB1 -- so Serial must stay on UART0 and that define must not be set.
ini = LINO / "firmware/platformio.ini"
txt = ini.read_text()
if "[env:fpms]" not in txt:
    txt += """
[env:fpms]
platform = espressif32
board = esp32-s3-devkitc-1
monitor_speed = 921600
monitor_port = /dev/ttyUSB1
upload_port = /dev/ttyUSB1
upload_protocol = esptool
upload_speed = 460800
lib_deps =
    ${env.lib_deps}
    madhephaestus/ESP32Encoder
build_flags =
    -I ../config
    -D __PGMSPACE_H_
    -D USE_FPMS_CONFIG
"""
    ini.write_text(txt)
    print("platformio.ini: added [env:fpms]")
else:
    print("platformio.ini: already patched")

# 3. topic names ----------------------------------------------------------
# Rename at the source so the existing Pi stack (fpms-teleop, fpms-odom-tf,
# dashboard) keeps working untouched -- they all expect /odom_raw and /imu.
# Cheaper and less risky than re-pointing four services.
fw = LINO / "firmware/src/firmware.cpp"
txt = fw.read_text()
subs = [('TOPIC_PREFIX "odom/unfiltered"', 'TOPIC_PREFIX "odom_raw"'),
        ('TOPIC_PREFIX "imu/data"', 'TOPIC_PREFIX "imu"')]
for old, new in subs:
    if old in txt:
        txt = txt.replace(old, new)
        print(f"firmware.cpp: {old} -> {new}")
    elif new in txt:
        print(f"firmware.cpp: already renamed to {new}")
    else:
        sys.exit(f"firmware.cpp: expected topic literal not found: {old}")
fw.write_text(txt)

print("PATCH OK")
