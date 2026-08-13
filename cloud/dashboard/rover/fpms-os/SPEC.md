# FPMS-OS — the contract every builder obeys

This file is the single source of truth for the FPMS rover operating system image.
Six agents build disjoint parts of it. **Where this file and your own judgement
disagree, this file wins** — it was derived from six exhaustive passes over the
repo, the session logs, and the hardware notes. Where this file says
`UNMEASURED`, do not invent a number: encode the refusal instead.

---

## 0. What FPMS-OS is, and what it is not

FPMS-OS is a **reproducible image build system** that produces a flashable
`.img` for the Orange Pi 5B (RK3588S, 16 GB) with the entire FPMS rover stack
preinstalled, preconfigured, and auto-starting from cold boot. The operator
flashes it, powers on, and the rover works. There is no setup step.

It is **not** a from-scratch kernel. On RK3588S that would forfeit the NPU
(`rknpu` + `librknnrt.so`, which only exist in Rockchip's BSP kernel), the Mali
GPU, the `bcmdhd` WiFi driver, and ROS 2 itself. FPMS-OS owns everything above
the vendor kernel: the boot sequence, the service set, the device naming, the
DDS transport, the safety ordering, first-boot provisioning, and the
self-verification that says whether the rover is actually working.

The distinction that matters: **every previous FPMS deployment was a Pi somebody
configured by hand.** `PI_FILE_INVENTORY.md` records that three scripts the whole
stack depends on exist only at `/usr/local/bin` on that Pi, in no repo, and that
reimaging it would break the rover with nothing to restore it from. FPMS-OS
exists so that never happens again.

---

## 1. Fixed platform decisions — do not deviate

| Decision | Value | Why it is not negotiable |
|---|---|---|
| Board | Orange Pi 5B, RK3588S, 16 GB | given |
| Base image | **Ubuntu 22.04 LTS arm64 (jammy)**, Rockchip BSP kernel | ROS 2 Humble has no supported build on any other Ubuntu; `rknpu` has no mainline driver |
| ROS 2 | **Humble Hawksbill**, `/opt/ros/humble` | pinned five independent ways; `nav2_params.yaml` uses the Humble-only singular `progress_checker_plugin`, and Humble's `controller_server` has no `odom_topic` |
| Python | **3.10** (system) | Humble's `rclpy` extensions are `cpython-310-aarch64` |
| Nav2 | ≥ 1.1.20 | version validated on this hardware |
| slam_toolbox | 2.6.10 | configs diffed against this exact version |
| User | **`ubuntu`**, home **`/home/ubuntu`** | hardcoded in every unit and in three Python files with no override |
| Hostname | **`fpms-rover1`** | this image is Rover 1. `fpms_console.py` derives the mDNS URL from `socket.gethostname()`, so the hostname IS the address; see `docs/IDENTITY.md` |
| `ROS_DOMAIN_ID` | **20**, everywhere, no exceptions | the ESP32 declares domain 20 in its CREATE_PARTICIPANT; on any other domain every topic list is empty and looks exactly like dead hardware |
| `RMW_IMPLEMENTATION` | **`rmw_fastrtps_cpp`** on *every* ROS unit | today `fpms-teleop` omits it — a silent no-data failure |
| OS source root | `cloud/dashboard/rover/fpms-os/` | |

### Version pins that are load-bearing

- **`paho-mqtt >= 2.0` from pip, NOT `python3-paho-mqtt` from apt.** Ubuntu
  22.04 ships 1.6.1, which has no `CallbackAPIVersion`. Five files import that
  symbol unguarded — including `fpms_cored.py`, the STOP authority. An image
  built with the apt package has no emergency stop and no telemetry, and fails
  at import with a traceback nobody reads because the unit just restart-loops.
- **`numpy < 2`, system-wide, and no user-local numpy in `~/.local`.** A
  user-local NumPy 2.x on the old Pi shadowed the system one and broke
  `tf_transformations` with `np.maximum_sctype was removed`. ROS Humble's C
  extensions are built against the 1.x ABI.
- **`pyserial`, never the PyPI package named `serial`** — different library,
  silently breaks the agent and the duty driver.
- **`python3-opencv` from apt, not `opencv-python` from pip** — mixing pip
  OpenCV with ROS's `cv_bridge` is a classic ABI break.
- **`websocket-client`, never `websockets`** — different library.

---

## 2. The environment every ROS unit must carry

Exactly this block, in every ROS unit, with no omissions:

```ini
Environment=ROS_DOMAIN_ID=20
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fpms/fastdds_udp_only.xml
Environment=ROS_LOCALHOST_ONLY=1
Environment=PYTHONUNBUFFERED=1
Environment=HOME=/home/ubuntu
EnvironmentFile=-/etc/fpms/config.env
```

`EnvironmentFile=` goes **before** the `Environment=` lines so `config.env`
cannot override the domain or the RMW. That ordering is deliberate and is
already the convention in `fpms-rosbridge.service`.

### Why `FASTRTPS_DEFAULT_PROFILES_FILE` is the most important line in this image

Fast DDS defaults to a shared-memory transport. On this hardware SHM segments do
not survive across "process eras" — a publisher started in one era and a
subscriber started in another complete discovery successfully and then **never
exchange a single byte**. The repo documents this failure twice without
identifying it:

> "the node is active, it logs `scans=2848 dropped=0 rate=9.55 Hz`, and
> `ros2 topic info /scan_lidar` reports 'Publisher count: 1'. And yet NO
> subscriber created afterwards ever receives a single LaserScan."
> — `units/fpms-ros-settle.service`

> "rosbridge only ever delivers topics whose PUBLISHER ALREADY EXISTED WHEN
> ROSBRIDGE STARTED... connected, subscribed, silent — no error anywhere."
> — `ROS_PORT.md`, reproduced three times

Both are the same bug. The current mitigations are a 75-second sleep, a blind
restart of two units, and a rule about deploy ordering. **Forcing UDPv4 removes
the cause.** The profile is at `/etc/fpms/fastdds_udp_only.xml`; every ROS node
on the rover is on the same box, and the operator laptop talks over websockets
and MQTT — neither uses DDS — so `ROS_LOCALHOST_ONLY=1` is safe and makes
discovery deterministic.

Keep `fpms-ros-settle.service` in the image but **disabled by default**, with a
comment saying it is the fallback if the profile turns out not to close it.
Removing a mitigation for a bug you have not yet confirmed fixed on hardware is
how this project has hurt itself before.

---

## 3. The nine boot units, plus what the image adds

Boot set (all enabled, all `Restart=always`), in dependency order:

```
network-online.target
└── mosquitto.service
    ├── micro-ros-agent.service          serial link to the ESP32 — UNTOUCHABLE
    │   └── fpms-uros-supervisor.service
    ├── fpms-cored.service         [1]   STOP authority — Before= anything that moves
    │   ├── fpms-teleop.service    [2]
    │   └── fpms-missions.service  [3]
    ├── fpms-rover-agent.service         camera + LiDAR → MQTT
    ├── fpms-lidar-ros.service           MQTT → /scan_lidar
    ├── fpms-tf.service                  static TF tree
    ├── fpms-odom-tf.service             /odom_raw + /imu → /odom
    └── fpms-wifi-powersave-hold.service
```

**Masked, permanently:** `fpms-ros-tunnel`, `fpms-rtos-follower`. Both can write
`/cmd_vel`. Masked, not disabled — `disable` only drops the `WantedBy` symlink
and anything pulling them in by name still starts them.

**Added by FPMS-OS** (none of these existed): `fpms-firstboot.service`,
`fpms-selftest.service`, `fpms-health-led.service` (optional),
`fpms-rosbridge.service` + `fpms-console.service` installed and enabled properly
rather than "restarted if present".

**Never enabled at boot:** anything that launches Nav2 or slam_toolbox. See §6.

### The one ordering edge that is a safety property

`fpms-cored` declares `Before=fpms-missions.service fpms-teleop.service` with
`Wants=`, not `Requires=`. There must be no window at boot where something that
can move is accepting commands while the stop authority is not yet listening —
but if cored fails, missions must still start, because a rover you cannot drive
is not safer than one you can stop.

### Why almost nothing else declares ordering

The micro-ROS serial link costs **90–225 s** to re-establish. A dependency edge
onto `micro-ros-agent` would turn a trivial restart of a bridge into a
multi-minute outage of every topic on the robot. Each node detects its own
missing input and says so instead. **Do not add `After=micro-ros-agent` to
anything.**

---

## 4. Files that must exist and today do not

These are hard blockers. Units reference them; nothing installs them; the repo
does not contain them.

| Path | Referenced by | Consequence if missing |
|---|---|---|
| `/usr/local/bin/fpms-wait-net` | `ExecStartPre=` on `fpms-lidar-ros`, `fpms-tf`, `fpms-ros-settle` — **no `-` prefix** | `ExecStartPre` fails → the unit never starts → no `/scan_lidar` and no TF tree at all |
| `/usr/local/sbin/fpms-wifi-ps-hold` | `ExecStart=` on `fpms-wifi-powersave-hold` | restart-loops forever; `bcmdhd` re-enables power save every 30–60 s and MQTT flaps on a 47 s cycle |
| `/usr/local/bin/fpms-uros-release-reset` | `fpms-uros-agent-run`, backgrounded so `set -e` does not catch it | the ESP32-S3 is held in hardware reset by the agent's own `open()`, permanently — measured at 56 bytes in 15 minutes |

`fpms-wait-net` exists because of a measured carrier flap: on 2026-08-06 carrier
was gained at 16:07:12, lost at 16:07:12, regained at 16:07:13, and
`network-online.target` was reached at 16:07:14. A DomainParticipant created in
that window binds an interface set that is then invalidated.

`fpms-uros-release-reset` exists because **RTS drives the ESP32-S3's EN pin**.
The kernel raises RTS by default on `open()`, pinning the MCU in reset. And the
ESP32-S3 boots on the **EN rising edge** — so simply clearing the line is not
enough. "A level is not a reset." It needs a sequenced app-mode reset: IO0 high,
EN low, hold 150 ms, release.

---

## 5. Hardware facts the image must encode

### The two CP2102 adapters

**Both report `ID_SERIAL` = `0001`.** `/dev/serial/by-id` therefore cannot tell
them apart and udev can only ever create one link for the pair. **The USB
topology path is the only discriminator.**

| Device | by-path (verbatim) | Baud |
|---|---|---|
| ESP32-S3 drive board | `/dev/serial/by-path/platform-fc880000.usb-usb-0:1.3:1.0-port0` | 921600 factory, **230400 on firmware v3** |
| LiDAR (LD06/LD19 family) | `/dev/serial/by-path/platform-fc880000.usb-usb-0:1.2:1.0-port0` | 230400 (probe 115200, 460800) |

**Never bind `/dev/ttyUSBn`.** Enumeration order is not guaranteed and getting
it wrong sends motor commands to the LiDAR. `config.env.example` still ships
`FPMS_LIDAR_PORT=/dev/ttyUSB0` — the image must not.

**Opening the drive tty resets the board** (handshake lines are wired to
EN/IO0), costing a 90–225 s re-link and taking the rover's only pose source down
with it. ModemManager probing a CP2102 does exactly this, so both ttys need
`ENV{ID_MM_DEVICE_IGNORE}="1"`.

There is a recorded 1.2/1.3 role conflict: `golden_backup/phase6_latest.py` says
the opposite. The later sources agree with each other and one of them is the unit
demonstrably driving the board today, so **1.3 = drive** wins. **Do not resolve
this by trying both** — the wrong one puts motor frames into the LiDAR. The
first-boot check must *verify* topology, not guess.

### Camera

USB UVC, bound by index today, and `/dev/video0` has become `/dev/video1`
mid-run. Several nodes on the same device are metadata-only and open fine while
never producing a frame. Caps at **30 fps in hardware** at every resolution —
that is the sensor's USB descriptor, not a software limit. Ships at
`FPMS_CAMERA_FPS=6`, `FPMS_JPEG_QUALITY=40` deliberately: higher settings
saturated WiFi and starved the LiDAR feed.

### NPU

`rknn-toolkit-lite2` (aarch64, Rockchip, not on PyPI) + `/usr/lib/librknnrt.so`,
and the two **must be version-matched to the kernel `rknpu` driver**. Nobody has
ever written down which version is on the old Pi — `bench_npu.py` reads it at
runtime from `/sys/kernel/debug/rknpu/version`. **The image must pin and record
both halves.** The agent degrades silently on NPU failure — it logs "NPU
unavailable; streaming without detection" and carries on, so a broken NPU
produces a rover that streams video and detects nothing while systemd reports
`active`. The selftest must catch this; "did it boot" will not.

### Battery / motion constants — status matters more than value

| Constant | Value | Status |
|---|---|---|
| counts/mm | **5.5** (`COUNTS_PER_REV` 1120) | MEASURED 2026-08-06, but on a chassis whose front-left hub was working loose. The 2.7× correction is robust; the third digit is not |
| `COUNTS_PER_REV` 3255 | — | **SUPERSEDED, 2.7× too high.** One constant, not three bugs — it explains the 2.3 m overshoot, the 744 mm plan that drove ~2 m, and the 300 mm move that went ~1 m |
| wheel diameter | 0.065 m | MEASURED 2026-08-06 (was assumed 70 mm; `NAV2_BRIEF.md` still says 70 — stale) |
| track width | 0.170 m | **UNMEASURED.** The operator measured 105 mm front-to-back, which is the wheelbase. 105/170/255/271 mm are all in use somewhere |
| `CMD_SCALE` | 6.1 | **A defect artefact, not a calibration** — the loop was saturated. Must be re-measured after the v3 flash. `fpms_missions.py` contradicts itself: line 287 says 1.0, line 704 sets 6.1 |
| `odom_scale` | 1.0 | UNMEASURED; adopted only from a profile carrying `measured: true` |
| `gyro_scale`, `coast_mm`, `min_moving_duty`, `lr_asymmetry` | unset | UNMEASURED |
| `FPMS_MISSION_ODOM_POSE_SIGN` | −1 factory / **+1 on v3** | MEASURED 5/5. Changing it **invalidates the saved anchor** — re-zero with `set_coordinate` |
| `ODOM_TWIST_SIGN` | −1 factory / **+1 on v3** | still −1 in three files |
| `FPMS_MISSION_TURN_WIRE_SIGN` | −1 code default, but `config.env.example` ships `1` | never measured; the two already disagree |

`/odom_raw`'s `twist.linear.x` has an **inverted sign relative to its own
`pose.position`**. Trust pose, never twist. Any guard comparing a commanded sign
against reported twist aborts every *correct* move — that is exactly what killed
the first deadband sweep.

**The image ships `/etc/fpms/calibration.json` absent, not guessed.** A missing
profile changes nothing; a wrong one is silent and total.

---

## 6. Nav2 and SLAM — installed, configured, and NOT started

Ship every Nav2 and slam_toolbox package. Ship the tuned params. Provide
`fpms-nav2.service` and `fpms-slam-mapping.service` / `fpms-slam-localization.service`
as **`systemctl start`-only units, not enabled**, each with a header explaining
the precondition it is waiting on.

Three reasons they must not autostart:

1. **`map→odom` would have two publishers.** `fpms-tf.service`'s `ExecStartPre`
   publishes a static `map→odom` at the arena start pose (0.972, 0.228, yaw
   π/2). slam_toolbox publishes the same edge dynamically. `TF_TREE.md`: "One
   publisher per edge, no exceptions." The image must make these mutually
   exclusive by construction — a `Conflicts=` line, not a comment.
2. **The LiDAR mount transform is all zeros and unmeasured.** Every offset in
   `fpms_tf.launch.py` is a `MEASURE ME` placeholder and the unit launches it
   **with no arguments**, so 0.0 is what gets published. By the file's own
   arithmetic **1° of mount yaw ≈ 10.5 mm of position error**, and both the cone
   guard and the occupancy grid inherit it in the same direction.
3. **`LIDAR_ROTATION_SIGN = -1` is UNVERIFIED.** If the scan is mirrored, no
   value of `laser_yaw` can undo it — a mirror is not a rigid transform.

Also: the default BT XML drives BackUp at `backup_speed="0.025"` m/s, under a
fifth of the firmware floor, and no parameter can override it because the speed
lives in the XML. Ship a patched BT XML with `backup_speed` ≥ 0.18 and point
`default_nav_to_pose_bt_xml` at it.

`arena_map.pgm` is not committed — generate it at build time from
`make_arena_map.py` and verify the emitted `.yaml` diffs clean against the
committed one. `slam/maps/` is empty and **cannot be baked**: `fpms_room.posegraph`
requires a physical mapping run in the actual room, with measured laser offsets.

---

## 7. Network contract

| Service | Port | Bind |
|---|---|---|
| mosquitto | 1883 | 0.0.0.0 |
| rosbridge | 9090 | 0.0.0.0 |
| fpms_console | 8090 | 0.0.0.0 |
| foxglove_bridge | 8765 | 0.0.0.0 |
| sshd | 22 | 0.0.0.0 |

**The Pi listens, the laptop dials out.** The operator laptop's Windows Firewall
has no inbound rule and no admin account to add one, so an outbound connection
from the laptop is the only direction that works. Nothing may bind loopback-only.

- **mosquitto:** `allow_anonymous false`, `password_file /etc/mosquitto/fpms.passwd`,
  bridge to the laptop at `192.168.137.1:1883` (**the hotspot — this is reality**;
  `CONNECT-ROVERS.md`'s `192.168.0.27` is stale). The bridge is `topic # both 0`,
  so QoS-1 guarantees hold only *within* the rover. Do **not** set
  `message_size_limit` below 262144 or camera frames vanish silently.
  **There are no ACLs today** — any authenticated client can publish
  `commands/stop`. Ship an ACL that restricts by client id.
- **avahi-daemon must be installed and enabled.** `.local` resolution is assumed
  by the console, rosbridge, Foxglove, and every deploy script — and avahi is
  installed by nothing in the repo. Also needs `127.0.1.1 <hostname>` in `/etc/hosts` — avahi publishes the `.local` name from there, so that line is not decoration.
  Do not disable IPv6: the Pi's mDNS has historically answered over link-local.
- **rosbridge's whitelist is the safety mechanism.** `/cmd_vel` and `/cmd_duty`
  are absent from `topics_glob` **by construction**, so a stray click or a stale
  browser tab cannot turn a wheel. `params_glob: "[]"`. Do not widen it. It lives
  in a params file rather than `-p` args because a whitelist that fails open when
  you mistype it is not a whitelist.
- **WiFi has no credential mechanism anywhere in the repo.** This is the single
  biggest gap for a headless image. First boot must read credentials from the FAT
  boot partition, support multiple SSIDs with the hotspot winning, fall back to
  an AP if nothing is found, and never gate service startup on association.
- **SSH:** key auth, `PasswordAuthentication no`, no baked default password.
  `ubuntu` keeps passwordless sudo — the deploy tooling requires it. Note the
  repo's own warning that the previously-used shared password should be treated
  as exposed and rotated.
- **Time:** there is no NTP or RTC anywhere in the repo, and the board has no
  RTC backup battery — it boots to the filesystem epoch. Ship `systemd-timesyncd`
  + `fake-hwclock`, peer the operator laptop, and **do not** make
  `time-sync.target` a dependency of any FPMS unit: that would delay STOP
  authority at boot. Step at boot only, slew afterwards — an NTP jump mid-mission
  makes `fpms-cored`'s 2.5 s evidence-freshness window read wrong in both
  directions.

---

## 8. The rules this image exists to enforce

1. **Silence is never success.** Every failure must be loud, named, and visible
   on a topic. The LiDAR publishes on a timer rather than on data specifically so
   that a dead scanner is a *declared fault* instead of a frozen picture — and
   because a dead payload's all-zero ranges convert to `inf`, which reads as a
   completely clear 360°. Fail-open is worse than fail-stale when the guard is
   confidently wrong.
2. **Never claim a measurement you did not take.** Placeholders say
   `MEASURE ME`. A profile is adopted only when it carries `measured: true` *and*
   is inside a sanity range, and refusals are logged.
3. **Judge motion by the operator's eyes, not by odometry.** This rover has
   reported clean travel while spinning in place and has lied about direction.
   Nothing in this image changes that; the residual stream makes it *visible*,
   not trustworthy.
4. **Never turn-test for liveness.** It destroys heading. Check pose changes
   passively.
5. **One publisher per edge, one owner per device, one writer on `/cmd_vel`.**
6. **Do not restart `micro-ros-agent` casually.** 90–225 s to re-link.

---

## 9. File ownership — strictly disjoint

| Agent | Owns |
|---|---|
| A — build pipeline | `build.sh`, `config/`, `scripts/`, `docs/FLASHING.md`, `README.md` |
| B — boot & provisioning | `firstboot/`, `overlay/usr/local/bin/fpms-wait-net`, `overlay/usr/local/bin/fpms-uros-release-reset`, `overlay/usr/local/sbin/fpms-wifi-ps-hold`, `overlay/boot/` |
| C — systemd units | `overlay/etc/systemd/system/**` |
| D — ROS & DDS | `overlay/etc/fpms/fastdds_udp_only.xml`, `overlay/etc/fpms/rosbridge_params.yaml`, `overlay/etc/fpms/nav2/**`, `scripts/40-ros-layer.sh` |
| E — hardware | `overlay/etc/udev/rules.d/**`, `overlay/etc/fpms/config.env`, `overlay/etc/fpms/calibration.json.example`, `overlay/etc/modprobe.d/**`, `overlay/etc/sudoers.d/**`, `overlay/etc/mosquitto/**` |
| F — verification | `selftest/`, `docs/ARCHITECTURE.md`, `docs/RUNBOOK.md`, `overlay/usr/local/bin/fpms-selftest` |

Do not create or edit a file outside your list. If you need something from
another area, assume it exists at the path named here.
