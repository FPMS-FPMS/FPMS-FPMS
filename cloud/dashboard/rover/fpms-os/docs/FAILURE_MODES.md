# Failure modes, and which FPMS-OS mechanism prevents each

The catalogue that justifies this image. Every entry happened. Each says what
went wrong, how it was found, and — the part that matters — whether the OS can
make it structurally impossible rather than merely documented.

Entries marked **NOT FIXED** are not fixed. Read those before a competition.

---

## A. The image exists because of these

### A1. Three scripts the whole stack depended on lived on one SD card

`fpms-wait-net`, `fpms-wifi-ps-hold` and `fpms-uros-release-reset` were
referenced by absolute path from four units. No repository contained them. No
deploy script installed them.

Two of them are `ExecStartPre=` **without** a `-` prefix, so their absence does
not degrade anything — it stops `fpms-lidar-ros` and `fpms-tf` from starting at
all. A rebuild would have produced a rover with no LiDAR and no TF tree, and
the journal line would have blamed `/usr/bin/env`.

**Detected:** by inventorying the Pi's filesystem against the repo, not by any
failure — the rover was working at the time.
**Fixed:** all three reconstructed from the measured evidence and committed.
`test_image_offline.py::exec_paths_exist` fails the build if any `ExecStart`
points at something the image does not contain.

### A2. `fpms-missions` was found disabled after a reboot

Active, healthy, and not enabled. A power cycle removed the mission executor
and nothing said so.

**Fixed:** stage 60 enables every boot unit at image build time and verifies
the state. `fpms-selftest` checks `is-enabled`, not just `is-active`, because a
unit that is running but not enabled is a rover that breaks on its next reboot.

---

## B. Silent data loss

### B1. Discovery succeeds, no data flows — the DDS bug

Diagnosed twice as two separate bugs. `/scan_lidar` with `Publisher count: 1`
and no subscriber ever receiving a scan; and rosbridge delivering 0 messages
from a healthy 6.6 Hz `/odom` because it started 19 minutes earlier —
reproduced three times.

Both are Fast DDS preferring shared memory, whose segments do not survive
across process eras. "The publisher must pre-exist the subscriber" was a
symptom description.

**Detected:** by an operator noticing the map was not updating. No error
anywhere, in any log.
**Fixed:** `/etc/fpms/fastdds_udp_only.xml` forces UDPv4, referenced by every
ROS unit. `fpms-ros-publishers.target` additionally orders consumers after
publishers. The build fails if the profile does not parse or if
`useBuiltinTransports` is not false. `fpms-selftest` runs a cross-era
subscribe on every boot — the one check a topic list cannot fake.
Full write-up: `docs/DDS.md`.

### B2. Boot carrier flap invalidated DDS participants

Measured: carrier gained 16:07:12, lost 16:07:12, regained 16:07:13,
`network-online.target` reached 16:07:14. A participant created in that window
binds an interface set that is immediately invalidated.

**Fixed:** `fpms-wait-net` waits for an address that has *held*, not one that
appeared. It exits 0 on timeout — a rover on a bad network that still publishes
beats one that declined to start its sensors.

### B3. The camera starved the LiDAR without dropping anything

paho serialises all publishes from one client onto one thread, in order. A
40 kB base64 frame queued ahead of a 2.5 kB scan delayed the scan by however
long the frame took to drain. Nothing was dropped — scans arrived *late*.

**Fixed in the application** (separate MQTT clients, camera bounded at
`max_queued=2`). The OS ships `FPMS_CAMERA_FPS=6` / `quality=40` with the
reason in `config.env`, so nobody raises them without re-measuring.

### B4. MQTT flapped on a 47-second cycle

The `bcmdhd` driver silently re-enables WiFi power save every 30–60 s, stalling
packets past mosquitto's 45 s keepalive drop (clients use `keepalive=30`,
mosquitto drops at 1.5×). Observed gaps: 47 s and 47 s.

NetworkManager's `wifi.powersave=2`, an interfaces hook and a udev rule were
all overridden by the driver.

**Fixed:** `fpms-wifi-ps-hold` re-asserts every 3 s and logs only on transition.

---

## C. Wrong-device and wrong-value hazards

### C1. Both CP2102 adapters report `ID_SERIAL 0001`

`/dev/serial/by-id` can name only one of the pair, and it is a coin flip which.
`/dev/ttyUSBn` is enumeration order. Binding wrong sends motor command frames
into the laser scanner.

**Fixed:** `99-fpms-serial.rules` keys on **USB topology** (`KERNELS==`), never
`ATTRS{serial}`. `config.env` ships the by-path string — `config.env.example`
in the repo still ships the dangerous `/dev/ttyUSB0`, and
`test_image_offline.py` fails if that value reappears. `fpms-firstboot`
verifies the topology and **refuses to guess** if it is wrong.

### C2. Opening the drive tty resets the ESP32

Handshake lines are wired to EN/IO0. Any open — including a ModemManager probe
— resets the board and costs a 90–225 s re-link, taking the rover's only pose
source with it.

**Fixed:** ModemManager purged at build time *and* both ttys marked
`ID_MM_DEVICE_IGNORE`. `fpms-selftest` reads port ownership from `/proc` and
never opens the device.

### C3. The agent's own `open()` held the MCU in reset permanently

RTS drives EN; Linux raises RTS on `open()`. Measured: 56 bytes in 15 minutes
with the agent apparently running perfectly. And clearing the line is not
enough — the ESP32-S3 boots on the EN *rising edge*. A level is not a reset.

**Fixed:** `fpms-uros-release-reset` drives a real edge with IO0 held in app
mode, then holds a second descriptor for the agent's lifetime. It must run
*concurrently*: before, and the agent's open re-arms the reset; after as a
one-shot, and the lines snap back the instant its fd closes.

### C4. `COUNTS_PER_REV` was 2.7× too high

3255 against a measured ~1120. One constant explains the 2.3 m overshoot, the
744 mm plan that drove ~2 m, and the 300 mm move that went ~1 m. **One
constant, not three bugs.**

**Partially fixed.** Firmware v3 carries the corrected value, and `odom_scale`
can correct the chain without a reflash. But the profile is only adopted with
`measured: true`, and **the image ships no profile at all** — a missing one
changes nothing, a guessed one is silent and total. The selftest reports which
constants remain unmeasured.

### C5. `/odom_raw`'s twist sign is inverted relative to its own pose

Measured wheels-off: commanded +0.012 gave twist −0.842 and pose +1.395. Any
guard comparing a commanded sign against reported twist aborts every *correct*
move — which is what killed the first deadband sweep and produced a cascade of
retracted conclusions, including a "290 mm backward lurch" that was a 290 mm
forward move.

**NOT FIXED, and not fixable in an OS.** Documented in `fpms-odom-tf.service`,
`config.env` and `docs/HARDWARE.md`. `ODOM_TWIST_SIGN` is still −1 in three
files and must become +1 with firmware v3 — deliberately unchanged, because
changing it, `TURN_WIRE_SIGN` and `CMD_SCALE` piecemeal makes the rover wrong
in a *new* way and none can be measured on factory firmware.

### C6. A firmware header claimed a compensation the Pi never applied

`firmware_v3/fpms_config.h` asserted the Pi compensated with
`FPMS_MISSION_ODOM_POSE_SIGN=-1`. The constant did not exist anywhere in
`fpms_missions.py`. A comment nobody executes and the running code disagreed
about a sign, invisibly, until the rover drove the wrong way.

**Fixed:** the constant exists, is applied at exactly one place, and is now
explicitly present in the shipped `config.env` with its status and the
30-second `--push-check` that verifies it without driving.

---

## D. Fail-open guards

### D1. A dead LiDAR read as a completely clear 360°

Publishing on a timer means arrival no longer implies freshness, and a dead
payload carries all-zero ranges. Zero converts to `inf` — "no return" — which
reads as clear in every direction. The never-stale fix would have turned "the
sensor is broken" into "full speed ahead": worse than the old fail-open,
because the guard is *confidently* wrong.

**Fixed in the application** — every consumer drops `stale: true` rather than
acting on it. The OS ships `seq`-based diagnosis in the selftest: `seq`
climbing with `stale` true means the scanner; `seq` frozen means the process.

### D2. Obstacle and battery guards still fail open on *absent* data

`clear is None` and `v is None` are treated as permissive.

**NOT FIXED.** A behavioural change to a gate-verified file that could not be
tested. Recorded here so it is not mistaken for handled.

---

## E. Two publishers, two writers

### E1. `map→odom` would have two publishers the moment SLAM started

`fpms-tf.service`'s `ExecStartPre` published a static anchor; slam_toolbox
publishes the same edge dynamically. `TF_TREE.md`: "One publisher per edge, no
exceptions." Two publishers do not error — they produce a pose that flickers
between two self-consistent answers.

**Fixed:** split into `fpms-map-anchor.service` with `Conflicts=` against both
SLAM units. Mechanically impossible, not forbidden by a comment. This also
retired the `setsid` + PID-file hack that STACK.md called the least tested
thing in the stack.

### E2. Two writers on `/cmd_vel`

`fpms-ros-tunnel` and `fpms-rtos-follower` can both write it.

**Fixed:** both **masked**, not disabled — `disable` only drops the `WantedBy`
symlink. Both are *shipped* so they can be masked; masking a unit that does not
exist is not the same guarantee, because a restore or a copy from the old Pi
would put an unmasked writer back. The selftest verifies `masked`, not
`disabled`, every boot.

---

## F. Dependency and environment

### F1. apt's `paho-mqtt` has no `CallbackAPIVersion`

Ubuntu 22.04 ships 1.6.1. Five files import that symbol unguarded — including
`fpms_cored.py`, the STOP authority. An image built on the apt package has no
emergency stop and no telemetry, and the only evidence is a unit quietly
restart-looping.

**Fixed:** pinned `>=2.0`, and stage 20 **verifies the import** and fails the
build. The selftest re-checks at boot.

### F2. A user-local NumPy 2.x shadowed the system one

Broke `tf_transformations` with `np.maximum_sctype was removed`, and the
failure surfaced far from its cause.

**Fixed:** pinned `<2` at both apt and pip level, any user-local copy removed
at build time, and the selftest checks for both the version and the shadow.

### F3. `fpms-teleop` was the only ROS unit with no `RMW_IMPLEMENTATION`

A mismatched RMW discovers nothing and reports no error.

**Fixed:** every ROS unit carries the full env block, and
`test_image_offline.py::ros_units_have_dds_profile` fails the build otherwise.

### F4. `avahi` was assumed by four components and installed by nothing

`fpms-rover1.local` is the documented address for the console, rosbridge, Foxglove
and every deploy script. The Pi's address changed more than seven times.

**Fixed:** installed and enabled; `/etc/hosts` and the hostname set at first
boot. Note mDNS resolution is **per-resolver** — it has worked from one
language's resolver and failed from another's on the same machine at the same
time — so an IP fallback is always needed.

### F5. No WiFi credential mechanism existed anywhere

**Fixed:** `fpms-wifi.conf` on the FAT partition, priority-ordered, with a
fallback AP.

### F6. No time sync, and no RTC

The board has no RTC backup battery and boots to the filesystem epoch.

**Fixed:** `systemd-timesyncd` + `fake-hwclock`, UTC, and deliberately **not**
a boot dependency of any FPMS unit — gating on `time-sync.target` would delay
STOP authority at boot. Note `fpms-cored`'s escalation compares timestamps
across processes with a 2.5 s freshness window, so a *stepped* clock mid-mission
remains a hazard.

### F7. No MQTT ACLs

Any authenticated client could publish `commands/stop` to any rover.

**Fixed:** per-client-id ACL, erring permissive where the topic map was
uncertain — an over-tight ACL drops publishes silently, which is the same
failure shape this whole document is about.

---

## G. Still open

| | Status |
|---|---|
| **`fpms-cored` has never run on hardware** | offline assertions only. Additional protection over the executor's own stop, never a replacement, until verified. |
| **LiDAR mount transform unmeasured** | 1° of yaw ≈ 10.5 mm, inherited in the same direction by the cone guard and the occupancy grid. Nav2 and SLAM refuse to start. |
| **`LIDAR_ROTATION_SIGN = -1` unverified** | if the scan is mirrored, no TF value can fix it. Check this *before* measuring the mount. |
| **`CMD_SCALE = 6.1`** | a defect artefact, not a calibration — the loop was saturated. `fpms_missions.py` contradicts itself (line 287 says 1.0, line 704 sets 6.1). |
| **Track width 0.170 m unmeasured** | 105/170/255/271 mm are all in use somewhere. 105 is the *wheelbase*. |
| **`/battery` type conflict** | missions subscribes `std_msgs/UInt16`, teleop `sensor_msgs/BatteryState`. A wrong type matches nothing, silently. On v3, missions will never see battery. |
| **Two agent files** | `fpms_rover_agent.py` and `fpms-rover-agent.py` have drifted. Stage 30 installs the longer one and says so loudly. |
| **The model is not in git** | `yolo26n-rk3588.rknn` and `~/fpms_console/` exist only on the old Pi. `fpms_yolo_npu.py` *is* in the repo at `rover/rescued/`, but it targets `yolov8n.rknn` and does not rescue the v26 path. Unlike the three `/usr/local/bin` scripts, **a model cannot be reconstructed from prose** — copy it off that card before it dies. `npu/models/README.md` has the procedure; `npu/convert/` rebuilds one from ONNX. |
| **The rover can go blind and look healthy** | `fpms_rover_agent.py` wraps the whole NPU block in one try/except, logs "NPU unavailable; streaming without detection" **once**, and runs blind forever with every unit `active`. `fpms-selftest`'s two shallow NPU checks (`import rknnlite`, model file exists) both PASS in that state, because the failure is at `init_runtime()`, downstream of both. **Addressed** by `fpms-npud` + `fpms-npu-selftest`, which run a real inference and check the output shape — but unverified, see below. |
| **Two thirds of the NPU may be idle** | `init_runtime()` with no `core_mask` uses core 0 only. The one place in this codebase that ever set a mask (`rescued/fpms_yolo_npu.py:130`) pinned `NPU_CORE_0`. FPMS-OS defaults to `0_1_2`, which is a **policy default, not a measurement** — multi-core is sublinear and could lose on a nano model. Run `npu/bench/fpms_npu_bench.py`. |
| **The NPU driver version is pinned by nothing** | `librknnrt.so` and the wheel both derive from one `RKNN_TOOLKIT2_TAG`, so those two agree by construction. The **kernel `rknpu` driver ships inside the vendor image** and nothing we control pins it. A mismatch fails at `init_runtime()`, which the agent swallows. |
| **Nothing here has run on the Orange Pi 5B** | the board has not arrived. That includes every NPU device path, every sysfs node, and every latency threshold in the NPU layer. |
