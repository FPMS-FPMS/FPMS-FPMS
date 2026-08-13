# FPMS-OS

A custom operating system image for the FPMS rover's brain — an Orange Pi 5B
(RK3588S, 16 GB). Flash it, power on, and the rover works. There is no setup
step, nothing to install, and nothing to code.

```sh
sudo ./build.sh                 # produces fpms-os-1.0.0-<date>.img.xz
```

---

## What this is, and what it deliberately is not

FPMS-OS is a **reproducible image build system**. It takes a vendor Ubuntu
22.04 arm64 image for the Orange Pi 5B and turns it into an FPMS rover: ROS 2
Humble, Nav2, the whole FPMS service stack, every dependency, every device
rule, every safety guarantee, baked in and auto-starting from cold boot.

It is **not** a kernel written from scratch, and that is a deliberate
engineering decision rather than a shortcut. On RK3588S:

- the **NPU has no mainline driver**. `rknpu`, and therefore YOLO26 detection,
  exists only in Rockchip's BSP kernel;
- so does the **Mali GPU** and the **`bcmdhd` WiFi** driver;
- **ROS 2 Humble** is pinned to Ubuntu 22.04 and its `rclpy` extensions are
  `cpython-310-aarch64` binaries.

Writing a kernel would cost all three and gain nothing this rover needs. So
FPMS-OS owns everything *above* the vendor kernel — the boot sequence, the
service set, the device naming, the DDS transport, the safety ordering, the
provisioning, and the self-verification — which is every layer where this
project has actually had problems.

## Why it exists

Every FPMS rover so far has been a Pi somebody configured by hand. The cost of
that is recorded plainly in `PI_FILE_INVENTORY.md`:

> `fpms-uros-release-reset`, `fpms-wait-net` and `fpms-wifi-ps-hold` exist only
> on the Pi at `/usr/local/bin`. No deploy script installs them and they are
> not in the repo. **If the Pi is reimaged, auto-connect breaks and nothing
> here restores it.**

Two of those three are `ExecStartPre=` on units declared *without* a `-`
prefix, which means their absence does not degrade the rover — it stops the
LiDAR and the entire TF tree from starting at all. The rover was one card
failure away from being unrecoverable, and nobody could have rebuilt it from
the repository.

FPMS-OS is the answer to that. Everything is in version control, the build is
reproducible, and the image verifies itself on every boot.

---

## What it fixes that the hand-built Pi never had

| | Before | In FPMS-OS |
|---|---|---|
| **The three missing scripts** | referenced by four units, in no repo, on one SD card | written from the measured evidence, in `overlay/usr/local/` |
| **DDS shared memory** | discovery succeeds, no data flows; papered over with a 75 s sleep and a blind restart | `fastdds_udp_only.xml` forces UDPv4 and removes the cause |
| **Serial device identity** | both CP2102s report `ID_SERIAL 0001`; by-id is a coin flip | udev keyed on **USB topology**, ModemManager purged |
| **`paho-mqtt`** | apt's 1.6.1 has no `CallbackAPIVersion`; five files import it unguarded, incl. the STOP authority | pinned `>=2.0`, and the build **verifies every import** |
| **`avahi`** | `fpms-rover1.local` assumed everywhere, installed by nothing | installed and enabled |
| **WiFi** | no credential mechanism anywhere in the repo | a text file on the FAT boot partition, plus a fallback AP |
| **Broker password** | shared, and flagged as exposed | generated per board at first boot; no default is ever baked in |
| **`map→odom`** | two publishers the moment SLAM starts, forbidden by a comment | `Conflicts=`, enforced by systemd |
| **LiDAR mount transform** | unmeasured zeros, published silently | Nav2 and SLAM **refuse to start** until it is measured |
| **Time** | no NTP, no RTC, boots to the filesystem epoch | `timesyncd` + `fake-hwclock`, and never a boot dependency |
| **MQTT ACLs** | none; any client could publish `commands/stop` | per-client-id ACL |
| **"Is it working?"** | a list of green units, which has meant nothing | `fpms-selftest`, every boot |

---

## What happens when you power it on

1. **First boot only** — grows the rootfs, generates unique SSH host keys,
   reads WiFi credentials from the FAT partition, sets the hostname, generates
   the broker password and prints it once, verifies the USB topology.
2. `mosquitto` comes up, then **`fpms-cored`** — the STOP authority — *before*
   anything that can move.
3. The publishers start: micro-ROS agent, rover agent (camera + LiDAR), the TF
   tree, odometry, teleop, the mission executor.
4. `fpms-ros-publishers.target` is reached, and only then do rosbridge and the
   console start — because rosbridge only ever delivers topics whose publisher
   already existed when it started.
5. About 90 seconds in, **`fpms-selftest`** runs and publishes its verdict to
   `telemetry/health`.

Then open **`http://fpms-rover1.local:8090/`**.

> Always type the name, never an IP. The rover's address has changed more than
> seven times in this project and every note that wrote one down was wrong the
> next day.

The rover **will not drive itself at boot**. Missions must be armed
deliberately. That is not an oversight.

---

## Layout

```
fpms-os/
  SPEC.md                the contract — read this first
  build.sh               the image builder
  config/fpms-os.conf    every version pin, in one file
  scripts/               chroot build stages, in numeric order
  overlay/               everything that lands on the rootfs verbatim
    etc/fpms/            config.env, the DDS profile, rosbridge whitelist
    etc/systemd/system/  20 units and one ordering target
    etc/udev/rules.d/    serial identity by USB topology
    usr/local/           the three reconstructed scripts, selftest, firstboot
    boot/                what the operator edits from Windows
  selftest/              offline checks — no Pi needed
  docs/                  DDS, hardware, architecture, runbook, failure modes
```

## Verify before you build

```sh
python3 selftest/test_image_offline.py
```

Twelve checks that need no hardware. The one that justifies the file is
`exec_paths_exist`: every `ExecStart` and `ExecStartPre` must point at
something the image actually contains. That is precisely the defect that
shipped for months.

## Honest status

- **No part of this has run on the Orange Pi 5B.** The board has not arrived.
  Everything here is derived from the repository, the session logs, and the
  measured hardware notes — the same sources that produced the working stack,
  but this specific assembly of them is unproven.
- **`fpms-cored` has still never run on hardware.** Its stop path is covered by
  offline assertions only. Treat it as *additional* protection over the mission
  executor's own stop, never as a replacement, until it has been verified.
- **Three values need a person and a ruler**: the LiDAR mount offsets,
  `LIDAR_ROTATION_SIGN`, and the calibration profile. The image ships them
  unmeasured and *says so*, and the units that would act on them refuse to
  start. It does not guess.
- **The base image is pinned and verified** — `ubuntu-22.04-preinstalled-server-arm64-orangepi-5b.img.xz`
  at `v2.4.0`, sha256 checked against upstream's own published sidecar.
  The gate that requires this caught a real bug: the URL previously named
  `ubuntu-22.04.4-…`, a point release upstream does not publish, and it 404'd.
  Pinned URLs rot; that is why the gate exists.
- **`docs/FAILURE_MODES.md`** lists what is fixed and what is not. Read it
  before a competition.

Judge motion by the operator's eyes, not by odometry. This rover has reported
clean travel while spinning in place and has lied about direction. Nothing in
this image changes that — the residual stream makes it *visible*, not
trustworthy.
