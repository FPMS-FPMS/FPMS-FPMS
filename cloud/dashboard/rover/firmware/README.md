# FPMS rover firmware — Yahboom MicroROS Board V2.0 (ESP32-S3)

Replacement ESP-IDF / micro-ROS firmware for the rover's motor controller.

It exists to fix one defect: **the rover has no slow speed.** Not a slow
speed that is hard to reach — no slow speed at all. This firmware gives it
one, and makes every per-robot constant tunable over ROS so that calibrating
it never needs another reflash.

> **This has never been compiled.** There is no ESP-IDF toolchain on the
> machine it was written on, so `idf.py build` has not been run, not once.
> Every file has been checked by inspection — CMake syntax, C syntax,
> matching braces, header/source agreement, ESP-IDF API shapes against the
> v5.x documentation — but **expect to fix compile errors on the first
> build.** Do not treat a green light here as a green light there.
> Sections marked **MEASURE-ME** are things nobody has measured yet.

---

## 1. The defect

The vendor / reference driver linearises motor stiction by **adding** a
constant to the controller output:

```c
#define PWM_MOTOR_DUTY_TICK_MAX (10000000 / 25000)   // 400
#define PWM_MOTOR_DEAD_ZONE     (200)                // 50% of full scale
#define PWM_MOTOR_MAX_VALUE     (400 - 200)          // the PID output clamp

static int PwmMotor_Ignore_Dead_Zone(int speed) {
    if (speed > 0) return speed + PWM_MOTOR_DEAD_ZONE;
    if (speed < 0) return speed - PWM_MOTOR_DEAD_ZONE;
    return 0;
}
```

The arithmetic is not in dispute:

| | |
|---|---|
| Full scale | 10 MHz / 25 kHz = **400 ticks** |
| Dead zone | **200 ticks = exactly 50.0% duty** |
| Smallest non-zero output | (1 + 200) / 400 = **50.25% duty** |
| Reachable band | **50.25% … 100%.** The bottom half does not exist. |
| Controller resolution | 200 levels, not 400 — the clamp halves it too |

Measured on this rover, exactly as that predicts: minimum burst ~0.35 s,
minimum move ~230 mm, and a 300 mm commanded segment that travelled about a
metre and hit an obstacle. Full root-cause analysis in
[`../research/R4_FIRMWARE.md`](../research/R4_FIRMWARE.md).

## 2. The fix: map, don't add

`components/pwm_motor/pwm_motor.c`. A non-zero request is mapped
**proportionally** onto `[MIN_PWM, FULL_SCALE]`:

```
|req| <= EPS  ->  0                                           (a real stop)
|req| >  EPS  ->  MIN_PWM + (|req| - EPS) * (FULL - MIN_PWM)
                                          / (FULL - EPS)
```

| Property | Additive (old) | Proportional map (new) |
|---|---|---|
| Smallest non-zero duty | 50.25% | `MIN_PWM`, default **0.25%** |
| Reachable duty band | 50.25%–100% | `MIN_PWM`–100%, i.e. all of it |
| Distinct output levels | 200 | **399** at the default; 370 even with an 8% floor |
| Doubling the request | does roughly nothing below half scale | roughly doubles the torque |

### Why `MIN_PWM` defaults to **zero**

At `MIN_PWM = 0` the map is a pure proportional pass-through — one tick of
request, one tick of duty.

That is not a shrug, it is the one setting on this axis with *evidence*
behind it. The operator's own Arduino driver for this board (recovered from
its build cache, built 2026-05-02) used the same 25 kHz carrier, wrote duty
straight through across its full 0–255 range with **no dead-zone term at
all**, and drove this hardware. The hardware does fine-grained duty. The
50% floor was a software choice, and nothing else.

A stiction feed-forward is still one command away:

```bash
ros2 param set /YB_Car_Node min_pwm_percent 8.0
```

**Raise it only if a *loaded* wheel is measured to stall at low duty, and
raise it as little as that measurement allows.** The asymmetry is the whole
argument:

- **Too low is benign.** The velocity loop's integrator winds the duty up
  until the wheel breaks free. You lose a little low-end responsiveness.
- **Too high recreates the exact defect this firmware exists to remove.**

If a floor does turn out to be needed, `R4_FIRMWARE.md` puts it at 7.5–15%.
Approach that from below, not above.

### The other half of the low-speed problem

A 50% duty floor was only one of the two things making slow motion
impossible. The other was that the velocity loop **quantised the thing it
was integrating**. The reference converts a commanded speed into an integer
count target per 10 ms period; at 0.003 m/s that target is 0.2 counts while
the feedback is an integer that is 0 most periods and 1 occasionally. The
loop was not controlling, it was dithering — and every dither became a
lurch.

`components/motor/motor.c` advances a **fractional** reference position
every period and servos on `(reference − measured)`. A 0.2 count/period
setpoint accumulates a full count of error every five periods and tracks
properly. Mathematically it is the same PID (the integral of velocity error
*is* position error) — just computed without quantising the integrand.

There is also a feed-forward term derived automatically from
`max_wheel_mps` and the wheel geometry, so the loop starts near the right
duty instead of integrating up to it, and a slew limit on `/cmd_vel` so a
step command is not delivered to a stictioned chassis as a step in duty.

---

## 3. Breaking changes — read before deploying

Flashing this firmware **without** making these host-side changes in the
same deployment will make the rover drive worse, not better.

### 3.1 `/odom_raw` twist sign is now correct

The old firmware published a `twist.linear.x` that was **sign-inverted with
respect to its own `pose.position`**. Measured, wheels off the ground:

| commanded `linear.x` | reported `twist.linear.x` | pose displacement |
|---|---|---|
| +0.012 | mean **−0.842** | **+1.395** (forward) |
| +0.100 | mean **−1.225** | **+3.505** (forward) |
| −0.012 | mean **+0.574** | **−1.506** (backward) |

The whole host stack works around it, and `NAV2_BRIEF.md` codifies the
workaround as *"TRUST POSE. NEVER TRUST TWIST."*

Here they cannot disagree: one per-step wheel displacement produces **both**
the pose increment and the reported velocity (`components/odometry/odometry.c`).
There is no second code path to get out of step with.

**Set `ODOM_TWIST_SIGN = +1`** in all three places:

- `fpms_teleop.py:150`
- `fpms_odom_tf.py:387`
- `deadband_sweep.py:154`

`ODOM_TWIST_ANG_SIGN` (`fpms_teleop.py:156`) is already `+1` and stays `+1`.

### 3.2 Turn direction is now REP-103

`components/car_motion/car_motion.h` encodes the **current, mirrored**
physical layout — M1 front-right, M2 rear-right, M3 front-left, M4 rear-left
— so a commanded `+angular.z` rotates the chassis **counter-clockwise**, as
REP-103 requires.

`fpms_missions.py:733` carries `TURN_WIRE_SIGN = -1` to compensate for the
old firmware. **It must become `+1`.** `fpms_teleop.py` has no turn-sign
correction at all and needs no change — it becomes correct by itself.

### 3.3 `CMD_SCALE = 6.1` is a symptom, not a calibration

`fpms_teleop.py:106` and `fpms_missions.py:668` divide every command by
6.1, because commanding 0.10 produced ~0.61 m/s.

**That factor is the defect.** A small command asked for a little effort,
the dead zone turned it into 50% duty, and the rover ran at 0.65 m/s
regardless. It was never a gain — the loop was saturated.

With a loop that can actually close, `CMD_SCALE` should collapse toward 1.0
(the residual is whatever genuine error remains in the encoder constants).
**Re-measure it after flashing. Do not carry 6.1 across.**

### 3.4 `/scan` is gone

The old firmware published `/scan` with every range at 0.0. It was measured
dead, the LiDAR is wired to the host SBC, and `fpms_teleop.py` deliberately
does not subscribe to it. It is not published here. Nothing should notice;
if something waits on it, that something was already waiting forever.

---

## 4. ROS interface (otherwise unchanged)

Node `/YB_Car_Node`, domain **20**, serial UART0 at **921600**, QoS
RELIABLE / VOLATILE / KEEP_LAST — the same profile the host uses.

| Direction | Topic | Type | Rate / notes |
|---|---|---|---|
| sub | `/cmd_vel` | `geometry_msgs/Twist` | `linear.y` ignored (skid-steer) |
| sub | `/beep` | `std_msgs/UInt16` | 0 off · 1 on · ≥10 = that many ms |
| sub | `/servo_s1` | `std_msgs/Int32` | degrees, 0–180 — **see warning below** |
| sub | `/servo_s2` | `std_msgs/Int32` | degrees, 0–180 |
| pub | `/odom_raw` | `nav_msgs/Odometry` | 10 Hz, `odom` → `base_footprint` |
| pub | `/imu` | `sensor_msgs/Imu` | 25 Hz, frame `imu_frame`, orientation **not** fused |
| pub | `/battery` | `std_msgs/UInt16` | 1 Hz, **decivolts** (÷10 for volts) |

> **GPIO8 is the S1 servo header, and on this rover it may be a sprayer.**
> Yahboom document GPIO8 as servo S1; the operator's Arduino sketch calls it
> "spray". Both can be true — a pump wired into the S1 header. This firmware
> emits **no pulses at all** on either servo channel until the first
> `/servo_sN` message arrives, so nothing fires on boot. Confirm what is
> physically on that header before publishing to `/servo_s1`.

`/imu` publishes an identity quaternion with `orientation_covariance[0] = -1`
(the `sensor_msgs` convention for "no data"). The host integrates
`angular_velocity.z` itself and has never used the quaternion. If the IMU
does not answer, the live covariances are set to −1 too, so a dead IMU reads
as a dead IMU rather than as a perfectly stationary rover.

---

## 5. Build

Do this on **Linux**. ESP-IDF plus the micro-ROS component build on Windows
is a fight you do not need.

```bash
# ---- 1. ESP-IDF v5.2.2 -------------------------------------------------
mkdir -p ~/esp && cd ~/esp
git clone -b v5.2.2 --recursive https://github.com/espressif/esp-idf.git
cd ~/esp/esp-idf
./install.sh esp32s3
. ~/esp/esp-idf/export.sh          # re-run this in every new shell

# ---- 2. Python packages the micro-ROS build needs, INSIDE the IDF venv --
pip3 install catkin_pkg lark colcon-common-extensions "empy<4"

# ---- 3. The micro-ROS component ----------------------------------------
cd <repo>/cloud/dashboard/rover/firmware
git clone -b humble \
    https://github.com/micro-ROS/micro_ros_espidf_component.git \
    components/micro_ros_espidf_component

# ---- 4. Build ----------------------------------------------------------
idf.py set-target esp32s3          # applies sdkconfig.defaults
idf.py build
```

The first build compiles the whole micro-ROS middleware and takes **10–25
minutes**. Later builds are seconds. `idf.py clean-microros` forces the
middleware to rebuild — needed after editing `app-colcon.meta`, and not
otherwise.

`app-colcon.meta` in this directory **replaces** the component's own
`colcon.meta` (the component looks for `app-colcon.meta` in the project
directory and uses it instead of its own, so it is a whole file, not a
patch). Two things differ from the component defaults: the transport is
`custom` instead of `udp`, and the entity limits are raised from 2/2/1 —
three publishers, four subscriptions, and five services for the parameter
server do not fit in the stock budget. Running out shows up as a bare
`RCL_RET_ERROR` from `create_entities()` and nothing else, so the limits are
set with headroom.

### If `rclc_parameter` is missing

If the build fails on `rclc_parameter/rclc_parameter.h`, that package is not
in your micro-ROS build. Add it under
`components/micro_ros_espidf_component/extra_packages/` and
`idf.py clean-microros`. The firmware degrades gracefully if the parameter
server fails at *runtime* — it drives, it just cannot be tuned — but it will
not link without the header.

---

## 6. Flash

**Back up the factory image first if that has not already been done.** There
is no confirmed public download of Yahboom's stock firmware; without a
backup it is gone. (Reported as already done and verified — confirm before
relying on it.)

The board's CP210x auto-resets over DTR/RTS, so **no button press is needed**
and the whole sequence can be scripted.

```bash
# On the rover's host SBC. The agent holds the serial port; it must stop.
sudo systemctl stop micro-ros-agent

# Flash. Use the stable by-path device — /dev/ttyUSB1 renumbers, and the
# LiDAR is on the neighbouring USB port.
PORT=/dev/serial/by-path/platform-fc880000.usb-usb-0:1.3:1.0-port0

idf.py -p "$PORT" -b 921600 flash

# Bring the agent back.
sudo systemctl start micro-ros-agent
```

Then **wait**. Reconnection after an agent restart has been measured at
**90–225 seconds** on this rover. It is not hung.

```bash
export ROS_DOMAIN_ID=20
ros2 node list          # expect /YB_Car_Node
ros2 topic hz /odom_raw # expect ~10 Hz
```

Two short beeps on power-up mean `app_main` was reached. They are the only
boot confirmation there is — the console is disabled, because UART0 belongs
to micro-ROS and a single stray log line corrupts the XRCE framing. To debug
with printf, uncomment the UART1 console block in `sdkconfig.defaults` and
watch GPIO17 (the unused LiDAR header). **Never send the console back to
UART0.**

The LED on GPIO45: solid = being commanded, slow blink = watchdog holding
the motors down (idle or disconnected), dark = the control task has died.

**Never run `espefuse.py burn_efuse` on this board.** It is the only
category of command that can permanently brick it.

---

## 7. Bring-up — wheels off the ground

A 300 mm command already produced a ~1 m run and a collision. **Put the
rover on blocks with the wheels clear for all of this.**

**1. One motor at a time.** The M1/M2 ordering is the one pin-map question
still open (Yahboom's docs and the third-party tree disagree about which
connector is which — see `components/board_pins/include/board_pins.h`).

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
```

All four wheels must turn **forward**. Any that turn backwards:

```bash
ros2 param set /YB_Car_Node invert_left  true     # or invert_right
ros2 param set /YB_Car_Node invert_enc_left true  # if odometry counts down
```

If front and rear of one side disagree, the M1/M2 ordering is swapped — fix
it in `board_pins.h` and reflash. It is the only thing in this bring-up that
cannot be fixed at runtime.

**2. Rotation.** `angular.z: 0.5` must rotate **counter-clockwise** seen
from above. If it does not, the side assignment in `car_motion.h` is wrong.

**3. Odometry sign.** Drive forward and confirm `/odom_raw` `pose.position.x`
**and** `twist.linear.x` are both positive. If `twist` is negative, something
in `odometry.c` has been changed — that is the bug this firmware fixes.

**4. Low speed — the whole point.**

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.02}}'
```

The wheels should turn slowly and *continuously*. If they lurch and stall,
raise `min_pwm_percent` in 2% steps until they do not, and stop there.

**Only then put it on the floor.**

---

## 8. Constants that MUST be measured on hardware

Everything here is a ROS parameter. Nothing here needs a reflash.

```bash
ros2 param list /YB_Car_Node
ros2 param get  /YB_Car_Node min_pwm_percent
ros2 param set  /YB_Car_Node min_pwm_percent 8.0
ros2 param set  /YB_Car_Node save_to_nvs true      # persist across reboot
```

| Parameter | Default | Status |
|---|---|---|
| `min_pwm_percent` | **0.0** | Pass-through, matching the known-good Arduino driver. Raise only on measured stall. |
| `enc_counts_per_rev` | 1170 | **MEASURE-ME** — see below |
| `wheel_circum_mm` | 219.9 | **MEASURE-ME** — 70 mm wheel |
| `track_width_m` | 0.170 | **MEASURE-ME** — effective, not geometric |
| `max_wheel_mps` | 1.30 | **MEASURE-ME** — from 0.65 m/s at the old 50% floor |
| `bat_divider` | 5.0 | **GUESS** — the divider ratio is undocumented |
| `pid_kp` / `pid_ki` / `pid_kd` | 1.5 / 0.35 / 0.0 | untuned starting point |
| `max_accel_mps2` / `max_alpha_radps2` | 1.0 / 6.0 | slew limits, taste |
| `cmd_timeout_ms` | 500 | watchdog; clamped to 50–2000 |
| `invert_left` / `invert_right` | false | set during bring-up |
| `invert_enc_left` / `invert_enc_right` | false | set during bring-up |

### Wheel geometry — two minutes, and everything depends on it

Four mutually inconsistent pairs are in circulation:

| counts/rev | mm/rev | counts/mm | source |
|---|---|---|---|
| **1170** | **219.9** | **5.32** | **default.** Operator's Arduino driver: 70 mm wheels, 1170 CPR, 5.32 counts/mm — the only set that is self-consistent (1170 / π·70 = 5.32) |
| 1040 | 150.8 | 6.90 | stock firmware. 1040 is corroborated by Yahboom (13 lines × 20 reduction × 4 edges); 150.8 mm is not, and is not a 70 mm wheel |
| 1320 | 219.9 | 6.00 | `fpms_odom_tf.py`. Wheel agrees, count does not |
| 2244 | 396.4 | 5.66 | the third-party repo — a **different robot** |

To settle it: mark a start line, note `enc1`..`enc4` (exported as ROS
parameters, updated at 1 Hz), **push the rover 2.000 m by hand in a straight
line**, note them again.

```
counts_per_metre = mean(delta) / 2.000
wheel_circum_mm  = enc_counts_per_rev / counts_per_metre * 1000
```

Set `wheel_circum_mm` to that and save. Every distance the rover reports is
directly proportional to this number.

### Effective track width

Not the ruler distance between the wheels: a skid-steer chassis scrubs, so
the yaw rate you actually get corresponds to a *wider* track, typically
1.2–1.8× geometric. (The 100 mm figure that appears elsewhere for this
chassis is the wheelbase, front axle to rear axle — a different number.)

Command a fixed `angular.z`, spin exactly 10 full turns by eye, and compare
against `/odom_raw` yaw. Scale `track_width_m` by the ratio.

### Battery divider

Multimeter on the pack, compare with `/battery` ÷ 10, and set
`bat_scale` to `actual / reported`.

---

## 9. Safety

- **`/cmd_vel` watchdog**, default 500 ms, in `car_motion.c`. It runs in the
  **control task on core 1**, not as a micro-ROS timer — so it still fires
  when the executor blocks, the agent dies, or the USB cable is pulled. A
  watchdog living inside the thing it is watching is not a watchdog.
- On a trip the ramp is **dropped, not decelerated**: if contact with the
  commander is lost, how far it is safe to keep travelling is unknown.
- A watchdog you can disable is not a watchdog either — `cmd_timeout_ms` is
  clamped to 50–2000 ms in `rover_config.c`.
- The bridges are parked in **coast** at boot, before anything slow runs.
- Every parameter is clamped in `rover_config.c` before it can reach
  anything that moves a wheel. A ROS parameter is an unauthenticated remote
  write into a motor controller; it is treated as hostile input.
- The velocity loop's integrator is bounded and back-calculated on
  saturation, so a stalled wheel cannot store up a lurch.

---

## 10. Attribution

This is an original implementation. No third-party source was copied.

**[`PrwTsrt/microros_esp32_diffdrive`](https://github.com/PrwTsrt/microros_esp32_diffdrive)** — read as a *reference*, not forked.
**It has no `LICENSE` file, so all rights are reserved and it cannot be
vendored into a public repo.** What was learned from it, all of it factual
rather than expressive:

- the GPIO pin map (cross-checked against Yahboom's docs, which disagree
  about M1/M2 — see `board_pins.h`);
- that the driver is MCPWM-based, deduced from its timer-group split
  (`M1..M4 = 0,0,0,1`), which is what a 3-timers-per-group part forces;
- the exact text of the defect — `PWM_MOTOR_DEAD_ZONE`,
  `PwmMotor_Ignore_Dead_Zone()`, `PWM_MOTOR_MAX_VALUE` — quoted in §1 above
  and in `pwm_motor.h` as the thing being replaced;
- the topic-name fingerprint (`odom_raw`, `imu`, `cmd_vel`) confirming the
  vendor lineage;
- that its own constants (2244 counts, 396.4 mm) are for a **different
  robot** and must not be copied.

Note that it ships the **same** `PWM_MOTOR_DEAD_ZONE 200`, so it does not
fix this problem either — and its `Motor_Update_PID_Parm()` is an internal C
function wired to no ROS interface at all.

**Yahboom documentation** — the authoritative GPIO table (motors, encoders,
buzzer 46, servos 8/21, battery ADC 3, IMU I2C 39/40, LED 45), the active
buzzer's polarity, and the encoder derivation `13 × 20 × 4 = 1040`.

**The operator's own Arduino driver for this board**, recovered from its
build cache (built 2026-05-02) — the evidence that dead-zone-free
pass-through drives this hardware, which is why `min_pwm_percent` defaults
to 0; and the self-consistent 70 mm / 1170 CPR / 5.32 counts-per-mm set.
That source is truncated and could not be used as a base.

**[`../research/R4_FIRMWARE.md`](../research/R4_FIRMWARE.md)** — the
root-cause analysis: the arithmetic of the 50.25% floor, the sub-count
dithering, the recommended 7.5–15% replacement band, the flash-risk
assessment.

**`_golden_ref/` and the host stack** — the measured behaviour this firmware
is answering to: the twist-sign table, `CMD_SCALE = 6.1`, `FULL_DUTY_MPS =
0.65`, the 0.35 s minimum pulse, the mirrored motor layout, and the exact
topic/type/QoS contract in §4.

---

## 11. Layout

```
firmware/
├── CMakeLists.txt              project
├── sdkconfig.defaults          console OFF, 4 MB flash, 1 kHz tick
├── app-colcon.meta             micro-ROS: custom transport, entity limits
├── partitions.csv              4 MB, 3 MB app, nvs kept
├── main/
│   ├── main.c                  bring-up; the 100 Hz control task
│   ├── uros_node.c             topics, timers, agent reconnection
│   ├── uros_params.c           the ROS parameter server
│   └── uart_transport.c        UART0 @ 921600, custom XRCE transport
└── components/
    ├── board_pins/             the pin map, and where sources disagree
    ├── rover_config/           every per-robot constant + NVS persistence
    ├── pwm_motor/              *** THE FIX *** MCPWM + the min-PWM map
    ├── encoder/                4x PCNT, x4 quadrature
    ├── motor/                  per-wheel velocity loop, fractional integral
    ├── car_motion/             mirrored mixing, slew limit, watchdog
    ├── odometry/               dead reckoning; pose and twist from one source
    ├── imu_icm42670/           I2C IMU
    ├── battery/                ADC1 ch2, decivolts
    └── aux_io/                 buzzer + two servo channels
```
