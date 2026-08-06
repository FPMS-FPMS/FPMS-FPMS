"""Second build: add the real ICM42670P gyro.

Kept separate from patch.py so the motor path can be proven with FakeIMU first.
If the board bricks after this patch (3-flash LED loop), the IMU is the only
variable that changed.
"""
import pathlib
import sys

LINO = pathlib.Path("/home/ubuntu/lino")

imu_h = LINO / "firmware/lib/imu/imu.h"
txt = imu_h.read_text()
if "USE_ICM42670_IMU" not in txt:
    # Must sit BEFORE the `#ifndef IMU` fallback, which both selects FakeIMU and
    # defines USE_FAKE_IMU -- firmware.cpp tests that macro to decide whether to
    # substitute odometry yaw rate for a real gyro reading.
    anchor = "#ifndef IMU"
    if anchor not in txt:
        sys.exit("imu.h: fallback anchor missing, upstream layout changed")
    txt = txt.replace(
        anchor,
        '#ifdef USE_ICM42670_IMU\n    #define IMU ICM42670IMU\n#endif\n\n' + anchor,
        1,
    )
    txt = txt.replace('#include "default_imu.h"',
                      '#include "default_imu.h"\n#include "icm42670_imu.h"', 1)
    imu_h.write_text(txt)
    print("imu.h: added ICM42670 branch")
else:
    print("imu.h: already patched")

cfg = LINO / "config/custom/fpms_config.h"
txt = cfg.read_text()
if "#define USE_ICM42670_IMU" not in txt:
    anchor = "#define K_P 0.6"
    txt = txt.replace(anchor, "#define USE_ICM42670_IMU\n\n" + anchor, 1)
    cfg.write_text(txt)
    print("fpms_config.h: enabled USE_ICM42670_IMU")
else:
    print("fpms_config.h: already enabled")

print("PATCH2 OK")
