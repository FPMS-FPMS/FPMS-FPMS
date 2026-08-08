# FPMS-OS architecture

## The boot sequence, end to end

```
power on
  │
  ├─ u-boot ─ Rockchip BSP kernel 5.10/6.1 (rknpu, Mali, bcmdhd)
  │
  ├─ fpms-firstboot.service            ONCE, before anything reads config.env
  │     grow rootfs · SSH host keys · WiFi from the FAT partition
  │     hostname + /etc/hosts · GENERATE the broker password
  │     verify USB topology (and refuse to guess)
  │
  ├─ network-online.target
  ├─ mosquitto.service                 :1883, 0.0.0.0, ACL'd
  │
  ├─ fpms-cored.service          [1]   STOP AUTHORITY
  │     Before= anything that can move. Wants=, not Requires=.
  │
  ├─ micro-ros-agent.service           serial link — 90-225s to establish
  │     └─ fpms-uros-release-reset     holds EN clear for the agent's lifetime
  │
  ├─ publishers (order among themselves does not matter)
  │     fpms-rover-agent               camera + LiDAR → MQTT
  │     fpms-lidar-ros                 MQTT → /scan_lidar
  │     fpms-tf                        static TF below base_footprint
  │     fpms-map-anchor                static map→odom  (XOR slam_toolbox)
  │     fpms-odom-tf                   /odom_raw + /imu → /odom
  │     fpms-teleop               [2]  MQTT → /cmd_vel
  │     fpms-missions             [3]  planner + measured-segment executor
  │
  ├─ fpms-ros-publishers.target        ← the ordering barrier
  │
  ├─ consumers
  │     fpms-rosbridge                 :9090   the browser's only way into ROS
  │     fpms-console                   :8090   the UI, served FROM the rover
  │
  └─ fpms-selftest.service             +90s, publishes to telemetry/health
```

### The two ordering edges that exist, and why almost nothing else does

Nearly every unit declares only `After=network-online.target mosquitto.service`.
That is deliberate. **The micro-ROS serial link costs 90–225 seconds to
re-establish**, so a dependency edge that transitively reaches
`micro-ros-agent` would turn a routine restart of a bridge into a multi-minute
outage of every topic on the robot. Each node detects its own missing input and
says so instead.

The exceptions:

| Edge | Why |
|---|---|
| everything → `mosquitto` | a genuine cold-boot race. The broker takes a moment to bind :1883, and clients that lost sat in their own retry loops while the dashboard showed nothing. |
| `fpms-cored` **before** missions/teleop | **a safety property.** There must be no window at boot where something that can move accepts commands while the stop authority is not listening. `Wants=`, not `Requires=` — if cored fails, missions must still start, because a rover you cannot drive is not safer than one you can stop. |
| publishers → `fpms-ros-publishers.target` → consumers | rosbridge only ever delivers topics whose publisher already existed when it started. Ordering without coupling: a publisher restarting later does not take the operator's view with it. |

---

## Data paths

```
  ESP32-S3 drive board                         LiDAR (LD06/LD19)
  USB 1.3, 230400                              USB 1.2, 230400
  micro-ROS / XRCE-DDS                         raw LD frames
         │                                            │
         ▼                                            ▼
  micro-ros-agent                              fpms-rover-agent ──┐
         │                                            │           │ camera
    ROS topics, domain 20, UDPv4 only                 │           │ (bounded
         │                                            │           │  client)
    /odom_raw  /imu  /battery                         │           │
    /cmd_vel (sub)                                    ▼           ▼
         │                                     MQTT  telemetry/lidar
         ├──► fpms-odom-tf ──► /odom, odom→base_footprint          telemetry/camera
         ├──► fpms-missions ──► /cmd_vel      │                    │
         └──► fpms-teleop   ──► /cmd_vel      ▼                    │
                                       fpms-lidar-ros              │
                                              │                    │
                                       /scan_lidar                 │
                                              │                    │
         ┌────────────────────────────────────┴────────────────────┘
         ▼                                     ▼
   rosbridge :9090  ◄── whitelist ──►    mosquitto :1883
         │                                     │  bridge, QoS 0
         ▼                                     ▼
   browser (console :8090)            operator laptop 192.168.137.1
```

Two things worth noticing:

- **The drive board's own `/scan` is dead** — every range is exactly 0.0. It is
  not a usable LiDAR source. The real scanner is the second CP2102, owned
  exclusively by `fpms-rover-agent`, republished over MQTT, and converted back
  to a `LaserScan` by `fpms-lidar-ros`. That loop through MQTT looks redundant
  and is not: only one process may hold the scanner tty.
- **The bridge to the laptop is QoS 0.** The QoS-1 guarantees on `events/#`,
  `commands/*` and `telemetry/mission_plan` hold only *within* the rover. Across
  WiFi, on a saturated link, messages are dropped rather than queued — which is
  why the dashboard ages telemetry forward from receipt rather than trusting a
  timestamp inside the payload.

---

## The TF tree

```
map                        [DYNAMIC  /tf         slam_toolbox]
 └── odom                                        ─── XOR ───
      │                    [STATIC   /tf_static  fpms-map-anchor]
      └── base_footprint   [DYNAMIC  /tf         fpms-odom-tf]
           └── base_link   [STATIC   /tf_static  fpms-tf]   z=0.035 MEASURED
                ├── laser_frame                             ALL PLACEHOLDERS
                └── imu_frame                               ALL PLACEHOLDERS
```

One publisher per edge, no exceptions. `map→odom` is the one edge with two
candidate owners, and `Conflicts=` makes them mutually exclusive at the systemd
level rather than by convention.

Only `base_footprint→base_link` is a real measurement. Everything below
`base_link` is a placeholder, and `FPMS_TF_OFFSETS_MEASURED=0` makes Nav2 and
both SLAM units refuse to start rather than navigate on it.

---

## Safety architecture

### Four independent STOP paths

`fpms-cored` subscribes to `stop`/`estop`/`auto_off` on **its own MQTT client,
its own socket, its own network thread, and nothing else**. That is the whole
trick: paho delivers a client's messages on one thread in order, and inside
`fpms_missions.py` that same thread also runs a 360-bin occupancy integration
and an A\* search under a lock. Neither is time-bounded, so a stop arriving
behind either of them waits. Here nothing can ever be in front of a stop.

The callback does no work — no JSON parse before the latch, no lock, no I/O.
Measured at **3.9 µs/call over 10 000 calls**, and a deliberately malformed
payload still latches.

Fan-out then runs on a pre-started worker thread over four paths, because the
point is not to depend on any one surviving:

| | Path | Covers |
|---|---|---|
| a | re-publish `commands/stop` at QoS 1 | the executor missed the original |
| b | ROS `/estop`, RELIABLE + **TRANSIENT_LOCAL** | firmware v3 cuts motors in its control task, independent of Pi logic. Transient-local so a node subscribing *after* the stop still gets it. Harmless no-op on factory firmware. |
| c | `events/stop_asserted` + a receipt | the dashboard shows STOP LATCHED without waiting for the executor to agree |
| d | **escalation**: SIGTERM `fpms-missions` | the one case a–c cannot cover — they all assume the executor is still running its loop |

Escalation is the only destructive action in the stack and it is fenced: only
if `telemetry/mission` still reports a driving phase 3 s after the stop, only
on evidence ≤2.5 s old, an absent phase counts as *not* driving and an unknown
phase counts as driving, once per stop, logged loudly, disableable.

SIGTERM is chosen, not blunt: `fpms-missions` handles it by aborting the
mission and publishing ten zero Twists before exiting, and `Restart=always`
brings it back. SIGKILL would leave the last non-zero Twist as the most recent
thing the drive board heard.

### Defence in depth elsewhere

- **rosbridge whitelist** — `/cmd_vel` and `/cmd_duty` absent by construction,
  `params_glob: "[]"`. In a params *file*, because a whitelist that fails open
  when you mistype it is not a whitelist.
- **Masked second writers** — shipped so they can be masked, because masking a
  unit that does not exist is not the same guarantee.
- **MQTT ACL** — rover services can publish only the three stop verbs, not
  missions or jogs.
- **Arm expiry** — missions require an explicit arm that times out.

---

## Build pipeline

```
config/fpms-os.conf   every version pin, in one file
        │
build.sh
   ├─ verify base image SHA256           refuses on mismatch or if unset
   ├─ loop-mount, grow, chroot           trap always unmounts and detaches
   ├─ 00-base-system    identity, time, avahi, mosquitto, sshd, purge ModemManager
   ├─ 10-ros-humble     ROS + Nav2 + SLAM + bridges + build the micro-ROS ws
   ├─ 20-python-deps    pins, then VERIFIES every import — fails the build
   ├─ 30-fpms-payload   the rover code, incl. the nav2/ tree nothing ever deployed
   ├─ 40-ros-layer      DDS profile parse-check, arena map, patched BT XML
   ├─ 50-overlay        modes, visudo -cf, CRLF guard
   ├─ 60-enable-units   enable · mask · deliberately-not-enable, then verify
   └─ 90-finalise       versions.json, clear machine-id, zero free space
        │
   fpms-os-<v>-<date>.img.xz + .sha256
```

Every stage is idempotent and the build fails loudly rather than producing an
image with a missing dependency. `selftest/test_image_offline.py` runs the
structural checks with no hardware at all.
