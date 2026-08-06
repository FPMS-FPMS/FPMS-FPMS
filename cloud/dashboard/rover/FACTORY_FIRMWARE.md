# Factory firmware — this may remove the need for custom firmware entirely

Analysed 2026-08-04.
`C:\Users\ruchi\Downloads\Factory-Firmware\Factory-Firmware\microROS_Robot_V2.0.0.bin`

## What it is

A **complete 4 MB flash image** — bootloader + partition table + app — not an
app-only binary. Flash at **0x0**, high confidence (confirmed two independent
ways: the file contains all three at their correct absolute offsets, and
Yahboom's own doc specifies 0x0).

    1,517,120 bytes
    0x0      2nd-stage bootloader (ESP32-S3, chip_id 9)
    0x8000   partition table (magic 0xAA50)
    0x10000  application  -> factory partition, 3 MB
    app descriptor: v2.0.0, project microROS_Robot,
                    built 2024-04-03, ESP-IDF v5.1.2

## THE KEY FINDING: it runs BOTH interfaces at once

It carries micro-ROS **and** a framed, checksummed UART protocol, concurrently:

- **micro-ROS**: node `YB_Car_Node`, topics `cmd_vel`, `odom_raw`, `battery`,
  `servo_s1`, `servo_s2`; frames `odom_frame`, `imu_frame`, `laser_frame`,
  `base_footprint`; `rmw_microxrcedds` with serial AND wifi-udp transports.
- **Rosmaster-style framed protocol**: `PROTOCOL`, `Protocol_Task`, separate
  `Uart0/1/2_Rx_Task`, and the error string
  **`Check sum error!, CalSum:%d, recvSum:%d`** — a sum-based checksum, matching
  Rosmaster_Lib's `sum % 256`.

**That is the interface the golden B8B code used.** `bot.set_motor(m1..m4)` with
raw duty -100..100 over serial, while ROS topics stayed live. The motor stack is
visible too: `PwmMotor_Set_Speed_M1..M4`, `bdc_motor_forward/reverse/brake/coast`,
plus `pid_motor[i]` and a yaw PID in `car_motion.c`.

Caveat: the specific frame bytes (0xFF/0xF8 out, 0xF7 back) could not be
confirmed from strings — they are inline immediates. That a checksummed framed
protocol exists is high confidence; the exact header bytes are inferred from the
golden code, not verified in the binary.

## Runtime-configurable via NVS

`ROS_DOMAIN_ID`, `ROS_SER_BAUD`, `AGENT_TYPE`, `IP_ADDR`, `IP_PORT`,
`WIFI_PASSWD`, `ROS_NAMESPACE`, `OFFSET_SERVO1/2`, `MOTOR_PID`, `YAW_PID`.

So domain and baud are **not** hard-coded — they are stored in NVS. That solves
the domain-0 problem we hit with linorobot (where the micro-ROS CLIENT declares
its domain) without a rebuild.

## Flash command

    esptool.py --chip esp32s3 --port /dev/ttyUSB1 --baud 460800 \
      write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB \
      0x0 microROS_Robot_V2.0.0.bin

Run `erase_flash` first — recommended, to clear stale NVS and `yb_data` left by
linorobot2_hardware, which could otherwise persist.

## WHY THIS MATTERS

The golden B8B code achieved 0.6% distance error and +/-1-4 deg turns **against
this exact firmware**. Flashing it back restores the precise environment that
code was written for, and makes `firmware_v2/fpms_drive_rtos.ino` a fallback
rather than the critical path.

## THE OPEN QUESTION — settle it before committing

This firmware is presumably what carried `PWM_MOTOR_DEAD_ZONE (200)` of a
400-tick scale, added as feed-forward, which made ~230 mm the smallest possible
move and blocked three sessions.

But **B8B never used `cmd_vel`.** It sent RAW DUTY through the framed protocol.
The dead zone lived in the velocity path. Two possibilities:

1. Raw duty bypasses the dead zone entirely -> B8B's 26/100 duty is genuinely
   26% -> everything works as it did.
2. Raw duty also passes through the dead-zone feed-forward -> 26 became ~76%
   duty -> B8B's constants were calibrated *around* it.

Either way B8B worked. But this decides whether the crawl we measured
(open-loop duty 70/255, 14.8 counts/mm, 0.3% error) transfers directly or needs
re-measuring.

**Test on hardware before trusting either:** flash factory, drive `set_motor`
raw duty at a low value, and measure. If a small duty produces a small
proportional move, the dead zone is not in the raw path.
