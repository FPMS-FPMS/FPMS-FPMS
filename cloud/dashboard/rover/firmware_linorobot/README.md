# Replacement drive firmware — linorobot2_hardware on the Yahboom ESP32-S3

**Session of 2026-08-03. Everything below was measured on live hardware.**

This replaces the vendor Yahboom firmware. It is the fix for the problem three
sessions failed to solve by tuning: `PWM_MOTOR_DEAD_ZONE (200)` of a 400-tick
scale added as feed-forward, which made 50.25 % duty the smallest output that
existed and ~230 mm the smallest possible move.

## The headline result

Measured, wheels on hardwood, under load:

| cmd (m/s) | travel in 2 s | implied m/s |
|---|---|---|
| 0.010 | 0.0 mm | — |
| 0.020 | 0.0 mm | — |
| 0.030 | **9.3 mm** | 0.0046 |
| 0.050 | 53.6 mm | 0.0268 |
| 0.080 | 118.8 mm | 0.0594 |
| 0.120 | 202.3 mm | 0.1011 |

**Minimum controllable move: ~230 mm → ~9 mm.** Travel now scales monotonically
with the setpoint instead of collapsing onto one 50 %-duty lurch. Turns work:
a commanded in-place turn produced a clean 180°, operator-observed.

Two properties of this firmware are why it works, and both should be preserved
through any future change:
- **No dead zone.** `grep -riE "dead_?zone|min_pwm|feed_?forward"` over its motor
  library returns nothing. Duty passes through untouched.
- **The PID regulates float RPM**, not integer encoder counts per 10 ms, so a
  crawl setpoint is representable rather than being quantisation noise.

## Upstream

`github.com/hippo5329/linorobot2_hardware` (Apache-2.0), an ESP32-S3 fork of
`linorobot/linorobot2_hardware`. Chosen over the previously-attempted
`PrwTsrt/microros_esp32_diffdrive` because its serial transport is the
*documented default* (`board_microros_transport = serial` plus an unconditional
`set_microros_serial_transports(Serial)`), which is exactly the uncommitted
menuconfig state that blocked the earlier attempt.

Cloned to `/home/ubuntu/lino` on the rover.

## Reproducing the build from a clean clone

```bash
git clone --depth 1 https://github.com/hippo5329/linorobot2_hardware.git ~/lino
bash setup2.sh "$PI_PASSWORD"     # creates ~/.platformio/penv with ROS build deps
cp fpms_config.h        ~/lino/config/custom/
cp icm42670_imu.h       ~/lino/firmware/lib/imu/
python3 patch.py    # config.h include, [env:fpms], topic renames
python3 patch2.py   # enable the real ICM42670P gyro
python3 patch3.py   # 4MB flash size
python3 patch4.py   # DDS domain 20
~/.platformio/penv/bin/pio run -e fpms -t upload
```

`flash.sh` does the guarded version: stops `fpms-teleop` first, backs up the
stock image, uploads, restarts the agent, verifies.

## Four blockers hit and solved — each alone looked like "the board is dead"

1. **No PlatformIO penv.** `micro_ros_platformio` runs
   `. $HOME/.platformio/penv/bin/activate` and then invokes colcon inside it. A
   `pip install --user platformio` never creates that venv, so the micro-ROS
   library build died before starting. Same shape as the ESP-IDF `catkin_pkg`
   blocker already in `../firmware/README.md`: **ROS build tooling must live in
   the venv the build actually activates.** → `setup2.sh`.

2. **8 MB flash header on a 4 MB chip.** The `esp32-s3-devkitc-1` board
   definition assumes 8 MB. The mismatch goes into the image header and the chip
   panics at `do_core_init`:
   `Detected size(4096k) smaller than the size in the binary image header(8192k)`.
   Boot loop, no session. → `patch3.py` pins `board_upload.flash_size = 4MB` and
   `default.csv` partitions.

3. **Participant created in DDS domain 0.** *In micro-ROS the CLIENT declares its
   domain in the CREATE_PARTICIPANT request — the agent's `ROS_DOMAIN_ID` does
   not place it.* Upstream calls plain `rclc_support_init()`, leaving domain 0
   while this rover runs domain 20. The agent logged a flawless session
   (participant, 3 datawriters, 1 datareader) and `ros2 topic info` still showed
   `Publisher count: 0`. Confirmed by re-running the graph query with
   `ROS_DOMAIN_ID=0`, which showed `/fpms_drive` with everything at `pub=1`.
   → `patch4.py` calls `rcl_init_options_set_domain_id(..., 20)`.

4. **Native-USB assumptions.** Upstream's esp32s3 env sets `/dev/ttyACM0` and
   `-D ARDUINO_USB_CDC_ON_BOOT`. This board reaches the host through an external
   CP2102 on UART0 (TX 43 / RX 44) — proven because esptool's DTR/RTS auto-reset
   works on `ttyUSB1`. Both had to go or `Serial` lands on the wrong peripheral
   and the board looks dead. → `[env:fpms]` in `patch.py`.

### Diagnostic lesson worth keeping
Blocker 2 was invisible until a serial console was sampled at **three different
bauds**. At 921600 the boot loop produced 147 KB of "binary" that looked like
plausible micro-ROS traffic (0 % printable, only `0x00`/`0x80` — the signature of
a baud mismatch, not data). At 115200 it was a one-line plain-text panic. The
previous session's undiagnosed boot loop was the same trap; `../SESSION_HANDOFF.md`
already said to put a console on it first, and that advice was correct.

## Configuration decisions

- **`USE_BTS7960_MOTOR_DRIVER`** — upstream documents it as covering
  A4950/DRV8833 modules, i.e. PWM on *both* `IN_A` and `IN_B` with
  `MOTORn_PWM = -1`. That is exactly this board's sign-magnitude topology
  (`board_pins.h`: "2 per motor, sign-magnitude drive"). A hand-written dual-PWM
  motor class was drafted and **discarded** once the stock driver was found to
  fit — prefer the tested upstream driver.
- **1320 CPR, not 1170.** From the golden driver's own constant
  (`_TKMM = pi*70/1320`, `fpms_phase6_LATEST.py:1309`), the code that achieved
  0.6 % distance error. The 1170 in the project briefs is not what the working
  code used.
- **Battery on GPIO 3**, not the template's GPIO 1 — GPIO 1 is the rear-left
  encoder A channel on this board. `BATTERY_ADJUST` is still the template's
  33k+10k divider formula and is **provisional**: it read 11.75 V where the
  vendor firmware read 11.6 V, so it is close but uncalibrated.
- **Topics renamed in firmware** to `/odom_raw` and `/imu` so `fpms-teleop`,
  `fpms-odom-tf` and the dashboard keep working with no Pi-side changes.
- **`USE_SHORT_BRAKE` deliberately off.** The golden driver stopped by coasting
  and its constants are calibrated against coast; braking on every PID
  zero-crossing would also add jerk during a crawl.

## Measured state after the swap

| Signal | Before (vendor) | After |
|---|---|---|
| `/odom_raw` | ~11 Hz | **50.1 Hz** |
| `/imu` | ~25 Hz | **50.0 Hz** |
| `/battery` | `UInt16` decivolts | `sensor_msgs/BatteryState` volts |
| min move | ~230 mm | **~9 mm** |

- IMU accelerometer verified: `|a| = 9.804 m/s²` against 9.81 expected.
- `/odom_raw` orientation is the **identity quaternion** — measured over 134
  messages. This settles the open contradiction flagged in `../NAV2_BRIEF.md` §5:
  `fpms_teleop._on_odom` is correct, `summarize_leg`'s "odom yaw is dead-reckoned
  from the wheels" comment is wrong. **There is no wheel-derived heading.**

## The gyro bug — FIXED

The chassis rotated a clean 180° while `angular_velocity.z` integrated to 0.0°.

**Root cause: the gyro was being read as bytes 6–11 of a 12-byte burst starting
at `ACCEL_DATA_X1` (0x0B).** That assumes the accel and gyro blocks are
contiguous; on this part they are not. Fixed by reading the gyro from its own
`GYRO_DATA_X1` (0x11) address, and lengthening the post-enable settle from 50 ms
to 200 ms.

The same misaligned burst also explains the accelerometer's apparent ~23° tilt
— `(-3.842, +0.014, -9.020)` with the rover flat. After the fix it reads
`(0.000, 0.000, +9.800)`. One root cause, two symptoms.

Verified on a full 13.41 V pack, gyro against wheel odometry:

| commanded | gyro | wheel-derived | disagreement |
|---|---|---|---|
| +0.60 rad/s | +41.7° | +41.6° | 0.1° |
| +0.90 rad/s | +70.2° | +69.6° | 0.6° |
| +1.20 rad/s | +106.2° | +105.4° | 0.8° |
| −1.20 rad/s | −107.8° | −107.0° | 0.8° |

Two independent sensors agreeing within 0.8°, correct signs, symmetric.

### Diagnostic lesson (cost two wasted build cycles)

The first two attempts at this diagnostic reported all-zero registers, which
read as "the gyro is off" and would have sent the next session rewriting the
power-management sequence. Both were artefacts of the instrument:

1. **`IMU_TWEAK` was defined in the driver header.** It is consumed inside
   `imu_interface.h`, which `imu.h` includes *before* `icm42670_imu.h`, so the
   `#ifdef` silently compiled to nothing and the fields carried message
   defaults. It must be declared in `fpms_config.h`, which `imu_interface.h`
   pulls in at its top.
2. **The build had failed** and the diagnostic ran against the previous
   firmware. `build5.sh` now prints the `SUCCESS` count before any measurement.

What caught both was a **control channel**: the diagnostic also reports
`WHO_AM_I` and raw accel Z. `WHO_AM_I = 0x00` is impossible on a board that is
running at all, which exposed the readings as invalid instead of letting them
masquerade as a finding. **Any diagnostic added here should carry a value whose
correct answer is already known.**

### `/odom_raw` now carries a REAL orientation

Measured after the swap: `identity: False`. This firmware derives yaw from wheel
odometry, so the "identity quaternion, no wheel-derived heading" constraint that
shaped `../NAV2_BRIEF.md` §3a **no longer applies** — there are now two
independent heading sources. That is what made the cross-check table above
possible.

## Also outstanding

- `MOTOR_MAX_RPM 180` is an **estimate**. linorobot derives max linear speed from
  it, so it scales every `cmd_vel`. Calibrate against measured wheel RPM.
- **Start-up lag.** Implied speed falls short of commanded, and the gap widens as
  the command shrinks (0.12→84 %, 0.08→74 %, 0.05→54 %, 0.03→15 %). The PID spends
  part of each burst breaking stiction — 0.05 m/s for 1.0 s gave 1 mm but for
  2.0 s gave 53.6 mm. Raising `K_I` is the first thing to try.
- **Stiction floor ~0.03 m/s.** Mechanical, not firmware. Usable crawl starts here.
- `CRUISE_MPS` in `../fpms_missions.py` is clamped below by
  `WIRE_FLOOR_MPS * CMD_SCALE`, derived from the *vendor* firmware's 0.0145 m/s
  quantisation limit. That floor no longer exists and the clamp will actively
  block commanding a crawl. Re-derive it from the numbers above.
- Stock 4 MB image backed up pre-flash, md5 `d7e02541627eaa40946c647169bd38d9` —
  an exact match to the md5 recorded in `../SESSION_HANDOFF.md`. Revert is one
  `esptool write_flash`.
