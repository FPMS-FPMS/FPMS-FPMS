# FPMS Rover — Nav2 / ROS 2 autonomy brief

**Every agent working on rover autonomy MUST read this file first.** It is the
shared context: hardware truth, what has been measured, what is already built,
and — importantly — the wrong conclusions that were reached earlier so nobody
re-derives them. Where this file and your memory disagree, this file wins.

Last verified: 2026-07-31, against live hardware.

---

## 1. Access

SSH from Windows via paramiko. Use this interpreter (paramiko installed):

    C:\Users\ruchi\FPMS-FPMS\cloud\dashboard\.venv\Scripts\python.exe

Host `fpms-pi.local`, user `ubuntu`. **The password is NOT recorded here** — this
file is committed to a public GitHub repo. Read it from the `FPMS_PI_PASSWORD`
environment variable, or get it from the operator.

The host resolves to an IPv6 link-local address, so a plain
`client.connect(host)` fails — connect the socket yourself:

```python
import os, socket, paramiko
pw = os.environ["FPMS_PI_PASSWORD"]          # never hardcode
sa = socket.getaddrinfo("fpms-pi.local", 22, socket.AF_INET6, socket.SOCK_STREAM)[0][-1]
s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM); s.settimeout(15); s.connect(sa)
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(hostname="fpms-pi.local", username="ubuntu", password=pw,
          sock=s, timeout=25, allow_agent=False, look_for_keys=False)
```

IPv4 fallbacks seen on this rover: `192.168.137.181`, `.201`, `.73` (DHCP moves).
`sudo` works via `echo "$FPMS_PI_PASSWORD" | sudo -S <cmd>`.

**Never write a password, token, or key into a repo file.** The first draft of
this very brief leaked the SSH password and had to be scrubbed before commit —
it is an easy mistake to make while writing "helpful" documentation.

---

## 2. Hardware truth

| Thing | Fact |
|---|---|
| SBC | Orange Pi 5B, RK3588, Ubuntu 22.04, kernel 5.10 rockchip |
| ROS | **ROS 2 Humble** at `/opt/ros/humble`, already installed |
| **ROS_DOMAIN_ID** | **20 — MANDATORY on every ROS call.** The firmware stores this itself. |
| Drive board | Yahboom **MicroROS Board V2.0 (ESP32-S3)**, micro-ROS / XRCE-DDS |
| Drive board port | `/dev/serial/by-path/platform-fc880000.usb-usb-0:1.3:1.0-port0` @ **921600** |
| LiDAR | D500/LD-series, `/dev/serial/by-path/platform-fc880000.usb-usb-0:1.2:1.0-port0` @ **230400** |
| Motors | M1 front-left, M2 back-left, M3 front-right, M4 back-right. 4WD skid-steer. |
| Chassis | track/wheelbase 170 mm, wheel Ø 70 mm |

**Both CP2102 adapters report an IDENTICAL `ID_SERIAL`.** Only the USB topology
path distinguishes them. **Never bind by `/dev/ttyUSBn`** — enumeration order is
not guaranteed and getting it wrong sends motor commands to the LiDAR.

`Rosmaster_Lib` **cannot** talk to this board (wrong protocol — it is for the
STM32 Rosmaster). It returns version=-1, battery 0.0, encoders 0. Those zeros
mean wrong protocol, not dead hardware.

### Topics (ROS_DOMAIN_ID=20, node `/YB_Car_Node`, QoS RELIABLE/VOLATILE)

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | sub. `linear.y` ignored (skid-steer) |
| `/odom_raw` | `nav_msgs/msg/Odometry` | ~10–11 Hz. **See §3 — twist sign is inverted** |
| `/imu` | `sensor_msgs/msg/Imu` | ~25 Hz. **Orientation NOT fused** (identity quaternion) — integrate `angular_velocity.z` |
| `/battery` | `std_msgs/msg/UInt16` | 1 Hz, **decivolts** (÷10 for volts) |
| `/scan` | `sensor_msgs/msg/LaserScan` | **DEAD — all ranges 0.0. Not a usable LiDAR source.** |
| `/beep`, `/servo_s1`, `/servo_s2` | | sub |

The board **self-reconnects in 90–225 s** after an agent restart. No reset press
needed — wait before touching hardware. If it is genuinely wedged (no session
after ~5 min) a physical reset button press is required.

The board answers its **config protocol** (115200, frames `0xFF/0xF8` out,
`0xF7` back, checksum = sum % 256) **only in the first ~5 s after reset**.

---

## 3. Two measured faults you must design around

### 3a. `/odom_raw` twist.linear.x is SIGN-INVERTED relative to its own pose

Measured wheels-off, nothing else publishing `/cmd_vel`:

| commanded | reported twist | pose displacement | actual |
|---|---|---|---|
| +0.012 | −0.842 | **+1.395** | forward |
| +0.100 | −1.225 | **+3.505** | forward |
| −0.012 | +0.574 | **−1.506** | backward |
| 0 | 0 | 0 | agree |

**TRUST POSE. NEVER TRUST TWIST.** Use `ODOM_TWIST_SIGN = -1` at the single
point twist is read. Any guard comparing a commanded sign against reported twist
will fire on correct motion and stay silent on real reversal.

### 3b. The firmware cannot express slow motion — this is the big one

From the firmware source:

```c
MOTOR_ENCODER_CIRCLE  1040    // counts/rev
MOTOR_WHEEL_CIRCLE    150.8   // mm
MOTOR_PID_PERIOD      10      // ms
PWM_MOTOR_DEAD_ZONE   200     // of a 400-tick scale
```

The velocity loop regulates **integer encoder counts per 10 ms**. One count per
period = **0.0145 m/s on the wire**. Below that the setpoint is under a single
count and feedback is pure quantisation noise. Observations land exactly on it:

| wire cmd | counts/10 ms | outcome |
|---|---|---|
| 0.0041 | 0.28 | stalls, then lurches |
| 0.012 | 0.83 | noise |
| 0.100 | 6.9 | clean and correct |

The 200/400 dead zone is a **50 % duty feed-forward**: commanded duty is either
exactly 0 or 50–100 %. There is no gentle duty.

Also: firmware passes `linear.x` 1:1 to wheel m/s with **no gain**, yet
commanding 0.10 yields ~0.61 m/s actual (~6×). The fitted hardware does not
match those constants — most likely ×4 quadrature decoding plus a wheel/gear
difference.

**CONSEQUENCE: stable control needs ≥ ~0.0145 wire ≈ 0.09 m/s real, and with
margin ~0.18 m/s. "Very slow" below that is physically unavailable.** Do not
try to fix this with PID gains — it is quantisation, not tuning. The real fix is
the encoder/wheel constants, i.e. a firmware rebuild. Design missions to run at
a speed the loop can actually hold, and achieve "slow and controlled" through
**short bounded segments with stops between**, not through a low continuous
setpoint.

---

## 4. Retracted conclusions — do NOT resurrect these

- ❌ "This chassis cannot creep." It cannot creep *below the firmware floor*.
- ❌ "Raise the MIN_CMD floor, never creep slower." The floor was set to 0.0057
  wire, **below** the real 0.0145 floor, so it cleared nothing. Both floors are
  now `0.0` with the mechanism kept and marked unmeasured.
- ❌ "290 mm backward lurch." It was a 290 mm **forward** move read through the
  inverted sign.
- ❌ "car_type (0x05) selects the chassis." It selects the micro-ROS
  **transport** (0=WiFi-UDP, 1=RPi5 serial, 2=RISC-V). It cannot invert motion.
- ❌ "The board needs a reset press after every agent restart." It self-reconnects.

---

## 5. What already exists

On the Pi (services, all `enabled` at boot):

| Service | Owns |
|---|---|
| `fpms-rover-agent` | camera, LiDAR→MQTT, YOLO, fire screen. **No ROS dependency, deliberately** — it keeps working when ROS is broken. Owns `ping`, `status`, `connect`, `disconnect`, `restart`. |
| `micro-ros-agent` | ESP32 link, domain 20, stable by-path. **NEVER RESTART IT** — costs a 90–225 s reconnect. |
| `fpms-teleop` | MQTT → `/cmd_vel` bridge. Owns all motion + `test_motors`, `read_encoders`, `beep`, `servo`, and `drive_status`/`drive_connect`/`drive_disconnect`. |
| `fpms-wifi-powersave-hold` | re-asserts WiFi power-save off every 3 s (the `bcmdhd` driver silently re-enables it every 30–60 s, which caused MQTT to flap on a 47 s cycle) |

In the repo at `cloud/dashboard/rover/`:

- `fpms_teleop.py` — the bridge. 0.6 s jog deadman, closed-loop nudge/turn,
  lurch guard that differentiates **pose** (not twist), full command set.
- `fpms_odom_tf.py` + `fpms-odom-tf.service` — **authored, NOT yet installed.**
  Position from `/odom_raw` pose deltas, heading from integrated gyro Z with
  stationary bias estimation. Publishes `/odom` and `odom`→`base_footprint` TF.
  Stamps with the **Pi's** ROS clock, not the board's (board time falls outside
  tf2's buffer and Nav2 rejects it with a confusing extrapolation error).
- `deadband_sweep.py` — **BROKEN**: reads the inverted twist, so its wrong-way
  detector is backwards. Fix to use pose before re-running.
- `deploy_rover.py` — backup + on-Pi syntax check + rollback. **Refuses to
  restart `micro-ros-agent`.** `--dry-run` is the default; `--live` to apply.
- `test_commands.py` — includes a cross-file check that no command verb is
  answered by both `fpms_rover_agent` and `fpms_teleop`.

Prior working autonomy (mine it, don't reinvent): `/home/ubuntu/fpms_phase6_LATEST.py`
on the Pi. It drove **without ROS**, via turn-then-drive segments with a
retrace-home strategy, and logged **0.6 % distance error** and **±1–4° turns**.
Tuned constants worth reusing: drive power 26, turn power 50, 20 Hz loop,
heading P-gain 10 (clamped ±6), turn coast factor 0.93, front stop 120 mm.
Its localisation was dead reckoning only — no SLAM, no AMCL, no Nav2.

---

## 6. Arena

1200 × 1200 mm. Origin **bottom-left**, +x right, +y up, millimetres.
Rover starts **bottom-right**, facing **forward = 90° (up the arena)**.

Zones (fractions of `ARENA_MM`, so they survive a rescale) are defined in
`frontend/src/lib/arena.ts` — that file is the single source of truth for arena
geometry. Zone A top-left, Zone B top-right, water station bottom-left.

The dashboard arena map is **world-fixed**: it never rotates. Only the rover
glyph moves within it. Do not change that.

**Pose is not published by the fleet** — the rover renders at the assumed start
pose with a `POSE: SIMULATED` badge. Any code reading `x_m`/`y_m`/`heading_deg`
must guard for absence; an unguarded `.toFixed()` on a missing pose field once
whited out the entire dashboard.

---

## 6b. Nav2 is ALREADY INSTALLED — and one landmine

Verified 2026-07-31: **Nav2 1.1.20** is fully installed from apt (arm64, Humble),
including `nav2_bringup`, `nav2_simple_commander`, `nav2_amcl`, `nav2_costmap_2d`,
the DWB / MPPI / Regulated-Pure-Pursuit controllers, `slam_toolbox` 2.6.10 and
`robot_localization`. `import nav2_simple_commander.robot_navigator` works.
Disk is at 49 % of 29 G with ~15 G free. **Do not reinstall it.**

**LANDMINE: do NOT import `tf_transformations`.** It is installed correctly but
fails at runtime:

    AttributeError: `np.maximum_sctype` was removed in the NumPy 2.0 release.

A user-local NumPy 2.x in `/home/ubuntu/.local/lib/python3.10/site-packages/`
shadows the system NumPy and is too new for the system `transforms3d`.

**Do not fix this by changing NumPy on the rover.** The camera/YOLO path in
`fpms-rover-agent` depends on NumPy and currently works; downgrading it to
satisfy a convenience library risks breaking the most reliable part of the
system. Compute quaternions inline instead — for planar motion it is two lines:

```python
qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)                 # yaw -> quat
yaw = math.atan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))  # quat -> yaw
```

`fpms_odom_tf.py` already does this and its pure-math tests pass.

---

## 7. Nav2 prerequisites — the actual blocking chain

Nav2 cannot localise without a laser scan **in ROS**. Today:

- the board's `/scan` is dead (all 0.0)
- the real LiDAR is owned by `fpms-rover-agent`, which publishes to **MQTT only**

So the chain is: **LiDAR→ROS → TF tree → map → AMCL → costmaps → behaviour tree.**
Each link is useless without the one before it. Build in that order and verify
each with `ros2 topic echo` / `ros2 run tf2_ros tf2_echo` before moving on.

Publishing LaserScan: zeros must be `inf`/`nan` (LaserScan's "no return"), **not**
0.0 — Nav2 reads 0.0 as an obstacle at the sensor origin. Use a topic name that
does not collide with the board's dead `/scan`.

---

## 8. Rules for every agent

1. **NEVER restart `micro-ros-agent`.**
2. **Never publish `/cmd_vel` unless your task explicitly authorises motion**,
   and then only bounded, short, with zero Twist after every command.
3. **Only one agent touches the Pi at a time.** Concurrent SSH has already
   caused protocol-banner failures and a corrupted measurement run.
4. **Do not fabricate results.** If you cannot reach hardware, say so and stop —
   a refusal is worth more than a plausible guess. This has already saved this
   project twice.
5. Verify with real command output and paste it. `ast.parse` for Python,
   `npx tsc --noEmit` + `npm run build` for frontend.
6. Comments explain **why**, not what. Match the surrounding file's voice.
7. No secrets in committed files.
