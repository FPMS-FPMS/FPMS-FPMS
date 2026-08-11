# Verification

Three layers, each answering a different question:

| | Runs on | Looks at | Answers |
|---|---|---|---|
| `test_image_offline.py` | build machine, before the build | the **source tree** | will a build of this tree be coherent? |
| `verify_image.sh` | build machine, after the build | the **produced artefact** | did the image actually come out right? |
| `fpms-selftest` | the rover | the **running system** | is this rover working? |

They do not overlap much, and the gaps between them are the point. The offline
checks can pass on a tree whose build then falls over in stage 10. The image
checks can pass on an image that has never had a kernel run against it. Only
the third one has ever seen the hardware, and even it defers the last word to
the operator's eyes.

## The source tree — on the build machine, before the build

```sh
python3 selftest/test_image_offline.py
```

Twelve structural checks over the overlay and the units. Run it before every
build; it takes under a second.

The check that justifies the file is **`exec_paths_exist`**: every `ExecStart`
and `ExecStartPre` must point at something the image actually contains. That is
exactly the defect that shipped for months — `fpms-wait-net` was named by three
units as an `ExecStartPre` *without* a `-` prefix, and no repository contained
it, so two units could not start at all and the only symptom was a missing
LiDAR and a missing TF tree.

The others: units parse and declare `Restart=`; every ROS unit carries
`ROS_DOMAIN_ID=20`, `RMW_IMPLEMENTATION` and `FASTRTPS_DEFAULT_PROFILES_FILE`;
`EnvironmentFile=` precedes `Environment=` so `config.env` cannot override the
domain; the DDS profile parses *and* has `useBuiltinTransports` false; the
sudoers rule names `fpms-missions.service` exactly and passes `visudo -cf`;
the rosbridge whitelist contains no drive topic and `params_glob` is empty;
nothing is both enabled and masked, and every masked unit is actually shipped;
no secret-shaped string is baked in; `FPMS_LIDAR_PORT` is not a `ttyUSBn`; and
no real `calibration.json` is present.

Writing this file caught two bugs in itself — it was matching directive names
inside comment prose, and it did not know stage 30 installs the supervisor from
the repo rather than the overlay — and one real gap: four variables the units
expand that `config.env` never defined.

**What it cannot prove:** it never opens an image. Everything it verifies is a
property of the tree — so a stage that silently installed nothing, a colcon
build that produced no binary, and a `.wants` symlink that was never written all
pass it comfortably. That is the whole reason the next section exists.

## The produced image — on the build machine, as root

```sh
sudo selftest/verify_image.sh fpms-os-1.0.0-<date>.img.xz
sudo selftest/verify_image.sh --json fpms-os-1.0.0-<date>.img
```

Exit codes: `0` all pass, `1` warnings only, `2` at least one failure — the same
convention as `fpms-selftest`, deliberately.

**Why this layer exists at all.** The build takes fifteen hours; stage 10 alone
has been measured at 14h20m. Until this script, *nothing anywhere inspected the
artefact*. `test_image_offline.py` reads `overlay/` and `scripts/` and never
opens the `.img`. And the target cannot be booted here — it is aarch64 and the
Orange Pi 5B has not arrived — so mounting the image and reading it is the only
verification that exists between "the build said OK" and someone flashing a
card.

It **mounts read-only and never writes**. The loop device is attached with
`losetup -r`, so the kernel refuses writes at the block layer; both filesystems
are mounted `-o ro`, with `noload` as the fallback so that an unclean journal is
never replayed. Unmount and detach happen in a trap that also fires on `INT` and
`TERM` — a leaked loop device makes `losetup --find` hand out a second device
for the same backing file, and the next build then has two mounts fighting over
one ext4.

The rootfs is found **by identity, not by number**: label `writable`,
`cloudimg-rootfs`, `rootfs` or `ROOTFS` wins outright, otherwise the largest
`ext[234]` filesystem. This mirrors `build.sh`'s `detect_partitions()` and for
the same reason — ubuntu-rockchip is GPT, the partition set is not fixed across
board variants, and loader/uboot/trust entries appear *ahead* of the
filesystems. `docs/BUILDING.md`'s hand recipe assumes `p2`; this does not.

Options: `--json`, `--quiet`, `--tmpdir DIR` (the decompressed image is 10–11 GB
and a short write produces a truncated image and a page of confident FAILs about
files that are really there), `--keep`, `--no-checksum`.

### What it checks, and why each one is there

| Check | What it catches |
|---|---|
| `boot units enabled` | **the load-bearing one.** Not "the unit file exists" — the `.wants` **symlink on disk**, for all 18 of stage 60's `BOOT_UNITS`. `fpms-missions` was once found active-but-disabled on the old Pi; on an image there is no earlier boot to have been running from, so a missing symlink means the unit is simply never there |
| `no dangling .wants links` | link targets are absolute *against the image root*. A check that forgets to rebase them reports whatever happens to be installed on the build machine |
| `second /cmd_vel writers masked` | `fpms-ros-tunnel` and `fpms-rtos-follower` must be symlinks to `/dev/null` in `/etc/systemd/system`. `disabled` is not `masked` — disable only drops the `WantedBy` symlink and anything pulling the unit in by name still starts it |
| `masked units still shipped` | the real bodies must remain in `/usr/lib/systemd/system`. Masking a unit that does not exist is a weaker guarantee: a restore from backup puts an unmasked writer back on the robot |
| `shebangs are LF, not CRLF` | a `\r` on the shebang fails as `/usr/bin/env: bad interpreter`, which is spectacularly misleading. On `fpms-wait-net` that takes down the LiDAR and the whole TF tree; on `fpms-uros-supervisor`, the drive link |
| `installed scripts present` | `fpms-wait-net`, `fpms-wifi-ps-hold`, `fpms-uros-release-reset` existed in **no repository** while three units named the first as an `ExecStartPre` with no `-` prefix |
| `uros_ws setup.bash` | `micro-ros-agent.service` sources it under `set -e`. Without it the wrapper dies instantly with no useful message and the rover cannot move |
| `micro_ros_agent is aarch64` | **the nightmare case.** This is a cross build. A silently-x86 binary compiles, installs, is the right size, and passes every existence check ever written — and on the board it is `Exec format error`. Checked by reading `e_machine` out of the ELF header |
| `/etc/fpms/nav2 populated` / `/etc/fpms/slam populated` | a previous bug created both **empty**. `install -d` succeeds, the build reports success, and three units name files inside them by absolute path |
| `firstboot placeholder intact` | the broker password placeholder MUST still be unreplaced in a shipped image. If it is gone, either a real credential is baked into an artefact that gets copied around, or firstboot ran during the build |
| `config.env present` | mode `0640`. `0644` makes the broker password world-readable on every board flashed from this image |
| `STOP escalation sudoers rule` | mode `0440` and the argv `kill -s SIGTERM fpms-missions.service`, byte for byte. sudo matches literal argv, and it silently *ignores* a group-writable file in `sudoers.d`. Either mismatch turns the last-resort stop into a password prompt on a moving rover |
| `rosbridge whitelist` | no `/cmd_vel` or `/cmd_duty` in the globs, so a stray click cannot turn a wheel |
| `DDS useBuiltinTransports=false` | Fast DDS silently ignores a malformed profile and falls back to defaults, reintroducing the shared-memory-across-eras bug the file exists to fix |
| `machine-id cleared` | present and **empty** — absent is a different marker. A baked id means every board flashed from this image shares one |
| `no SSH host keys` / `no random seed` / `firstboot not yet run` | the same shipped-identity class |
| `operator files on FAT` | `README-FPMS.txt` and `fpms-wifi.conf.example` must be on the **FAT** partition. On ext4 Windows cannot see them, and every WiFi instruction in `docs/FLASHING.md` then points at a file the operator cannot reach — which means the rover cannot be provisioned at all |
| `no build scaffolding left` | `policy-rc.d` returns 101 for *everything*, so a leftover copy means every future `apt install` on the board silently declines to start what it installed. Also `qemu-aarch64-static`, and gigabytes of `uros_ws/build` |
| `versions.json` / `npu-versions.json` | exist and parse. The only record of what went into a card that is already in a rover |
| `paho is 2.x` | apt's 1.6.1 has no `CallbackAPIVersion`; `fpms_cored.py`, the STOP authority, imports it unguarded. Verified by reading the module — we cannot import an aarch64 build here |

Two implementation notes, both of which were bugs first:

- **The carriage-return check reads bytes, not text.** The obvious
  `line="$(sed -n 1p "$f")"` was written first and was wrong: several `sed`s
  open files in text mode and silently strip the `\r`, so the check reported
  "no carriage returns" on a file that plainly had one. A permanent silent PASS
  on the most load-bearing check in the file. It uses `od -tx1` now.
- **No pipeline ends in `grep -q`, `head`, or an `awk` with `exit`.** Under
  `set -euo pipefail` that SIGPIPEs the producer, `pipefail` reports 141, and a
  plain assignment adopts it. That bug class has killed this build twice;
  `selftest/test_shell_hazards.py` exists because of it, and this script is
  clean under it.

### What it cannot establish

Say this out loud before quoting a PASS at anybody. The script prints an
abbreviated version of it at the bottom of every run.

- **It does not boot the image.** Not on the board, not in a VM. No kernel runs,
  no systemd runs, no unit is ever observed to start. "Enabled" here means a
  symlink exists on disk — a different claim from "starts".
- **It executes nothing from inside the image.** The binaries are aarch64 and
  the host is not. Every check is file existence, mode, content, or ELF header.
  An import that would fail at runtime passes here as long as the file is there.
- **No NPU.** No device to test against and no `.rknn` model in the repository.
  Whether `librknnrt.so` matches the kernel `rknpu` driver is unknowable until
  the board boots.
- **No serial devices.** Neither CP2102, the udev topology rules, the ESP32
  reset behaviour, nor the 90–225 s micro-ROS re-link.
- **No DDS.** Nothing publishes or subscribes. Whether data crosses process eras
  is `fpms-selftest`'s job, on hardware.
- **No network, no MQTT, no WiFi.**
- **It does not tell you whether the rover drives.** Nothing short of the
  operator's eyes does — the odometry has reported clean travel while the rover
  span in place.

### Warnings that are correct on a fresh image

- `known-absent payload` — `~/yolo/yolo26n-rk3588.rknn` and `~/fpms_console/`
  exist only on the old Pi and are in no repository, so no build can produce
  them. `fpms-console.service` is nonetheless enabled and will fail every boot.
- `optional payload` — `fpms_ros_tunnel.py` / `fpms_rtos_follower.py` are
  installed best-effort and their units are masked anyway.
- `rknnlite present` — reported as a warning because `npu-versions.json` is the
  fuller record.

## On the rover

```sh
fpms-selftest            # human readable
fpms-selftest --json     # machine readable
fpms-doctor <symptom>    # what to do about a failure
```

Runs automatically 90 seconds after every boot and publishes to
`telemetry/health`. The 90 s is not arbitrary: the micro-ROS link takes up to
225 s to establish from cold, so a check at 30 s would report a healthy rover
as broken every single boot, and an alarm that cries wolf is worse than none.

It exists because **a list of green units has repeatedly meant nothing.** Every
check maps to something that happened:

| Check | What it catches |
|---|---|
| `DDS carries data across eras` | the one that matters. Subscribes from a *freshly started* process — the case that fails while every topic list passes |
| `paho-mqtt >= 2.0` | apt's 1.6.1 has no `CallbackAPIVersion`; the STOP authority imports it unguarded |
| `numpy is 1.x` / `no shadowing user-local numpy` | a user-local 2.x shadowed the system one and broke things far from the cause |
| `boot units enabled` | `fpms-missions` was once found active-but-disabled — fine until the next reboot |
| `second /cmd_vel writers masked` | `disabled` is not `masked` |
| `broker password provisioned` | an auth failure crashes nothing; services retry forever and the dashboard is empty |
| `STOP escalation sudoers rule` | sudo matches literal argv; a mismatch is a silent password prompt |
| `NPU runtime` / `NPU model present` | the agent degrades silently and streams video while detecting nothing |
| `LiDAR healthy` | distinguishes a dead *scanner* from a dead *process* via `seq` |
| `rosbridge whitelist` | a drive topic reachable from a browser |
| `clock sane` | no RTC backup battery; boots to the filesystem epoch |

**It never moves the rover.** It does not turn-test for liveness — that
destroys heading — and it does not open the drive board's tty, because opening
it resets the ESP32 and costs a 90–225 s re-link. Port ownership is read from
`/proc` instead.

Exit codes: `0` all pass, `1` warnings only, `2` at least one failure.

### Warnings that are correct on a fresh image

- `calibration profile: none` — a missing profile changes nothing; a guessed
  one is silent and total.
- `LiDAR mount transform: unmeasured` — Nav2 and SLAM refuse to start. Intended.
- `NPU model present: FAIL` — the `.rknn` is not in the repository.

**What it cannot prove:** that the rover drives correctly. It is the only layer
that has seen the hardware, and it still stops short of the thing that matters —
deliberately, because the two ways to find out both do harm. Turn-testing for
liveness destroys heading, and the odometry has reported clean travel while the
rover span in place. The last verdict is the operator's eyes, and there is no
third layer that can take that job.
