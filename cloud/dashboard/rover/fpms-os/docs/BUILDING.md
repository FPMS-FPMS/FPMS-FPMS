# Building an FPMS-OS image

`docs/FLASHING.md` tells you what to do with a finished `.img.xz`. This tells
you how to produce one, and how to work out what went wrong when it stops four
hours in.

Read `README.md` for what the image *is*. This document is only about the
build.

> **Estimates are marked as estimates.** Nothing in the timing or disk section
> below is a stopwatch measurement of a completed build on this hardware. Where
> a number comes from a file in this repository it is cited; where it is
> arithmetic or judgement it says so. That distinction is SPEC.md rule 2 and it
> applies to build documentation as much as to calibration.

---

## 1. The host

`build.sh` loop-mounts a disk image, chroots into an **aarch64** rootfs, and
runs eight stages inside it. That dictates everything below.

| Requirement | Why |
|---|---|
| **x86_64 Linux, or WSL2** | Anything that can run `losetup` and `chroot`. macOS and native Windows cannot: no loop devices, no binfmt, no chroot. |
| **root** | `build.sh:61` — `must run as root (loop devices and chroot)`. Not `sudo` on individual commands; the whole script. |
| **`qemu-user-static` + `binfmt_misc`** | Every command in every stage is an aarch64 binary. Without the binfmt handler they all fail with `Exec format error`. |
| **aarch64 host** | Optional alternative. `build.sh:89` takes a native path and skips qemu entirely. Nobody on this project has one. |
| **~20 GB free, 25 GB to be comfortable** | See §2. |
| **Hours of wall clock** | See §3. |

### Packages

```sh
apt install -y qemu-user-static binfmt-support \
               gdisk parted e2fsprogs xz-utils wget ca-certificates
```

`build.sh`'s own header (lines 30–31) lists:

```
qemu-user-static binfmt-support xz-utils parted e2fsprogs dosfstools kpartx wget
```

Two corrections, from reading what the script actually calls:

- **`gdisk` is missing from that list and is needed.** `prepare_image()` calls
  `sgdisk -e` to move the backup GPT header after growing the file. The call is
  written `sgdisk -e '$LOOPDEV' >/dev/null 2>&1 || true`, so a missing `sgdisk`
  is **silent** — and the `parted resizepart` on the next line is then working
  against a disk whose backup header is in the wrong place. See §5, row *G*.
- **`kpartx` and `dosfstools` are listed but never invoked.** `build.sh` uses
  `losetup --partscan` for partition nodes and never formats a FAT partition.
  Installing them is harmless; their absence is not the cause of anything.

### Verifying binfmt before you start

```sh
cat /proc/sys/fs/binfmt_misc/qemu-aarch64      # must exist and be "enabled"
ls -l /usr/bin/qemu-aarch64-static             # must exist and be executable
```

`build.sh:94–100` checks exactly these two things and refuses to start
otherwise. They are **separate failures**: the binfmt registration can be
present (registered by a previous `docker run multiarch/qemu-user-static`, by
`systemd-binfmt`, or left behind by another toolchain) while
`/usr/bin/qemu-aarch64-static` is not installed at all. Fix both:

```sh
apt install -y qemu-user-static binfmt-support
# if the registration is still absent or points at a path that no longer exists:
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

The `-p yes` sets the `F` (fix-binary) flag, which makes the interpreter
resolve outside the chroot. `build.sh` does not depend on that flag — it copies
`/usr/bin/qemu-aarch64-static` into the image at `enter_chroot_mounts()` and
removes it again in `finalise()` — but with the flag set, a chroot you enter by
hand also works.

### WSL2, specifically

This is where the build has actually been driven from, and the details below
were established by doing it rather than by reading documentation.

**WSL2 Ubuntu 22.04 works.** A systemd-enabled WSL2 distro has
`/proc/sys/fs/binfmt_misc` mounted, loop devices, and a kernel with ext4 and
vfat. That is everything `build.sh` needs.

**Getting root without a password.** `sudo` inside the distro may prompt for a
password you never set. From Windows:

```powershell
wsl -d Ubuntu-22.04 -u root -- /bin/bash -lc 'cd /root/rover/fpms-os && ./build.sh'
```

`-u root` gives you root with no password prompt, which also matters because a
password prompt in a build that runs for hours is a build that stops the moment
the terminal loses focus.

**`qemu-aarch64` may already be registered, and you still need the package.**
On a WSL2 distro that has ever run Docker Desktop with multi-arch enabled, the
binfmt entry exists while `/usr/bin/qemu-aarch64-static` does not. `build.sh`
catches this at line 99 and says so; it is not a mystery, but it looks like one
if you only check the first of the two.

**Git Bash mangles `/mnt/...` paths.** If you drive WSL from Git Bash rather
than PowerShell, MSYS rewrites anything that looks like a Unix absolute path
into a Windows path before the argument reaches `wsl.exe`. A command that reads
`wsl -u root -- ls /mnt/c/Users` arrives as `ls C:/Users`. Set:

```sh
MSYS_NO_PATHCONV=1 wsl -d Ubuntu-22.04 -u root -- ls /mnt/c/Users
```

or use PowerShell, where the problem does not exist.

**Build in WSL's native ext4. Never on `/mnt/c`.** This is the one that costs
real time rather than a confusing error message:

- `/mnt/c` is a **9p** filesystem. A chroot doing an apt install performs tens
  of thousands of small file creates, and each one takes a 9p round trip. The
  same stage that takes tens of minutes on ext4 does not finish on `/mnt/c`.
- **Loop-mounting a file across 9p is unreliable.** `losetup` may attach and
  the resulting filesystem may behave, or `e2fsck`/`resize2fs` may report
  damage that is not in the file. Do not spend an afternoon debugging it.

So put the tree inside the distro:

```sh
# from WSL, as root
mkdir -p /root/work
cp -a /mnt/c/Users/<you>/…/cloud/dashboard/rover /root/work/
cd /root/work/rover/fpms-os
```

**Copy the parent `rover/` directory, not just `fpms-os/`.** `build.sh:226`
stages the rover source from **two levels up**:

```sh
cp -a "$HERE/.."/*.py "$HERE/../stack" "$HERE/../nav2" "$HERE/../slam" \
      "$HERE/../STACK.md" "$MNT/opt/fpms-os/src/" 2>/dev/null || true
```

Note the `|| true`. If those paths are absent the copy fails **silently** and
the build carries on for however long stages 00–25 take, then dies in stage 30
with `MISSING: fpms_missions.py`. Copying only `fpms-os/` into WSL is the
easiest way to buy yourself that.

**Line endings.** The tree is authored on a Windows workstation.
`.gitattributes` forces `* text eol=lf` for exactly this reason, and stage 50
has a CRLF guard — but that guard covers the *overlay*, not `build.sh` or
`scripts/*.sh`. A CRLF `scripts/00-base-system.sh` fails inside the chroot with
`/usr/bin/env: bad interpreter: No such file or directory`, which is
spectacularly misleading. Check before a long build:

```sh
grep -rlc $'\r' build.sh scripts/ || echo "clean"
```

---

## 2. Disk

The build needs four things on disk, and for a while it needs all four at once.

| | Size | Where | Note |
|---|---|---|---|
| Base image, compressed | **~837 MB** | `.cache/<base>.img.xz` | downloaded once; `--no-download` reuses it |
| Base image, decompressed | **~4–5 GB** | `.cache/<base>.img` | kept, so a rebuild does not re-decompress |
| Working copy | **base + `ROOTFS_GROW_MB`** = ~10–11 GB | `fpms-os-<v>-<date>.img` | created sparse; materialises as the chroot writes |
| Output | **~2–3 GB** | `fpms-os-<v>-<date>.img.xz` | plus a `.sha256` |

`ROOTFS_GROW_MB=6144` (`config/fpms-os.conf:98`) is headroom **inside** the
image — ROS Humble, Nav2 and the micro-ROS workspace do not fit in the stock
rootfs. It is not host disk budget; it is added to the host cost on top of the
base.

**Peak is roughly 15–17 GB** — arithmetic, not a measurement. Two moments push
toward the upper end:

1. **Stage 90's zero-fill.** `90-finalise.sh:114` does
   `dd if=/dev/zero of=/EMPTY bs=1M` to write the image's free space to zero so
   it compresses. That is deliberate and correct — without it the `.xz` is
   several GB larger — but it converts the sparse working image into a nearly
   fully-materialised ~10–11 GB file on the host.
2. **The final compress.** `xz -T0 -6 -f "$OUT"` writes the `.xz` while the
   `.img` still exists. Both are on disk simultaneously.

`build.sh`'s header says **~25 GB free**. Plan for that. **Under about 20 GB
free the build will probably fail**, and here is where:

| Free space | Where it dies |
|---|---|
| < ~6 GB | `fetch_base()`, decompressing the base — clean, early, obvious |
| ~6–12 GB | stage 10, mid-apt. dpkg reports `No space left on device` writing to the loop file. The image filesystem is *fine*; the host ran out under it |
| ~12–17 GB | stage 90's zero-fill, or the final `xz`. The `dd` is `|| true`, so an early stop is swallowed and you get a larger `.xz` rather than an error |

**On WSL2 all of this lands on your Windows C: drive.** The distro's ext4 lives
in a dynamically-growing `ext4.vhdx` under
`%LOCALAPPDATA%\Packages\<distro>\LocalState\`. Two consequences:

- `df -h` inside WSL tells you the ext4 has room. It does not tell you whether
  C: can grow the VHDX to match. **Check both.** When C: runs out, the guest
  sees I/O errors that look like filesystem corruption.
- The VHDX **does not shrink** when you delete the build tree. Reclaiming it
  needs `wsl --shutdown` then `Optimize-VHD` / `diskpart compact vdisk`.

---

## 3. Time

**Hours, not minutes.** On an x86_64 host every command inside the chroot —
`dpkg`, every maintainer script, `pip`, `cc1plus` — is an aarch64 binary
executed by `qemu-aarch64-static` instruction by instruction. The usual figure
for qemu-user is **5–20× slower than native**, workload-dependent: syscall-heavy
work sits at the low end, tight compute at the high end.

That multiplier applies to the two expensive things this build does.

| Stage | Dominated by | Order of magnitude (**estimate**) |
|---|---|---|
| download + decompress base | network, then `xz -d` | minutes — **native**, not emulated |
| `prepare_image` | copying 4–5 GB, `resize2fs` | a few minutes — native |
| **10 — ROS/Nav2 apt** | ~1.5–2.5 GB of packages; hundreds of emulated maintainer scripts | tens of minutes to a couple of hours |
| **10 — micro-ROS colcon build** | **the long pole.** C++ compiled under emulation | **one to several hours** |
| 00, 20, 25 | apt and pip, no compiling | tens of minutes total |
| 30, 40, 50, 60 | file copies, checks, `systemctl enable` | minutes |
| 90 | `apt clean`, then writing ~6–10 GB of zeros | minutes to tens of minutes, disk-bound |
| finalise | `xz -T0 -6` of a ~10 GB image | 10–40 minutes — **native**, scales with cores |

**Why the colcon build dominates.** `10-ros-humble.sh:92–93` runs
`create_agent_ws.sh` and then `build_agent.sh`, which fetch and compile the
Micro XRCE-DDS Agent along with Fast-CDR and Fast-DDS **from source**. That is
real C++ compilation, the workload qemu-user is worst at, and there is no apt
package that avoids it — the script says so at line 72: *"NOT an apt package."*

**It will look frozen.** Both commands are redirected to `/dev/null`
(`>/dev/null` on lines 89, 92 and 93), so a build that is compiling normally
prints nothing for a very long time. Before concluding it has hung, check that
it is doing work:

```sh
top -b -n1 | head -20            # expect qemu-aarch64-static burning a core
ls -l --time-style=+%H:%M:%S .build/mnt/home/ubuntu/uros_ws/build 2>/dev/null
```

## MEASURED, 2026-08-11 — not an estimate

Stage 10 completed on a WSL2 host (x86_64, qemu-user emulation) in:

```
==> stage 10-ros-humble.sh OK in 859m57s
```

**14 hours 20 minutes, for that one stage.** The estimate above said "one to
several hours" for the colcon build. It was wrong by roughly a factor of four,
and the table is left in place unchanged so the gap between a reasoned guess
and a stopwatch stays visible.

Stage 00 was measured at **20 minutes** on the same host.

What this changes:

- **Plan for a whole build in the region of 16–20 hours**, not 3–8. Start it
  when you can leave it overnight and then some.
- **Do not casually kill a build.** Fourteen hours is not something you
  re-run because a log looked quiet. Check it is compiling (see above) before
  concluding anything.
- **`--from N` is not a convenience, it is the difference between a two-minute
  retry and another overnight run.** Every stage failure after this point
  should be fixed and resumed, never restarted.
- **The single highest-value optimisation available** is caching the built
  `uros_ws` between builds. It is ~14 of those hours, it does not change
  between builds unless the micro-ROS sources move, and nothing in the current
  pipeline preserves it. If this image is going to be rebuilt more than once,
  do that first. **This is now implemented — see §3.1.**

A cautionary note on how this measurement was obtained: the run reached stage
20 and was then deliberately torn down to reclaim disk, which threw all 14
hours away. The teardown was correct and asked for — but it is worth knowing
that `/root/fpms-build` holding an in-progress image is not scratch space.

---

## 3.1 The micro-ROS workspace cache

> **Status: half-landed, and it says so at runtime.** The caching logic is in
> `scripts/10-ros-humble.sh`. It needs **one two-line change to `build.sh`**,
> which has **not** been made — `build.sh` was not in scope for the change that
> added this. Until that lands, stage 10 prints the reason it could not cache
> and builds from source exactly as it always did. **Nothing fails without it.**

### What it does

After a **verified successful** micro-ROS build, stage 10 packs
`/home/ubuntu/uros_ws` to a directory on the **build host** — outside the
image, so it survives `rm`-ing the `.img`. On the next build it restores that
tarball instead of compiling, **if and only if the cache key matches**.

Three files make up an entry:

| File | What it is |
|---|---|
| `uros_ws.tar.gz` | the packed workspace: `install/` and `src/`, never `build/` or `log/` (those are deleted before the pack, and are the multi-GB part) |
| `uros_ws.key` | the cache key, in full, as plain text. **This file is the authority.** |
| `uros_ws.info` | human-readable provenance: when it was written, which agent binary, and the resolved commit of every source repo. Read by people; never used to decide a hit. |

An entry is only usable when **both** the tarball and the key are present.
Publishing writes them in the order key-deleted → tarball-moved → key-written,
so a build killed mid-write leaves a **miss**, never a mismatch. That matters
here: this build has been killed three times.

### What it saves, honestly

The cache removes the colcon C++ build. It does **not** remove the ROS/Nav2
apt install, which still runs on every build. So stage 10 goes from a
**measured** 859m57s to *the apt time alone* — "tens of minutes to a couple of
hours" per the table in §3, which is an **estimate**, not a stopwatch figure.
Roughly 13–14 hours of the 14h20m come back; the stage does not become
instant.

Packing costs a `tar -czf` of a few hundred MB **under emulation** — an
**estimated** ten to twenty minutes, paid once, on the build that populates it.

### The `build.sh` change this needs

The chroot cannot see the host. `/opt/fpms-os` is a `cp -a` **copy** made by
`stage_all()`, not a mount — anything written there lands inside the image and
dies with it. The only writable host path a stage can reach is one `build.sh`
mounts. Two lines:

```sh
# in enter_chroot_mounts(), alongside the other bind mounts:
mkdir -p "$CACHE/uros" "$MNT/opt/fpms-cache"
mount --bind "$CACHE" "$MNT/opt/fpms-cache"

# in cleanup()'s unmount loop — it MUST be released before "$MNT" itself:
for m in opt/fpms-cache dev/pts dev proc sys run boot/firmware; do
```

`$CACHE` is already `$HERE/.cache`, where the base image lives. Add
`opt/fpms-cache` to `release_stale()`'s loop too, for the same reason the
other mounts are there.

**One hazard worth stating**, because §5 row D already warns about the
related case: while this is mounted, the host cache is reachable from inside
`.build/mnt`. If cleanup fails to unmount it, an `rm -rf .build` walks into
your cache — exactly as it can already walk into `/dev` and `/proc`. Unmount
before any `rm -rf`, and check with `mount | grep fpms-cache`.

Stage 10 also honours `$FPMS_UROS_CACHE_DIR` if it is set, which is how to use
the cache from inside `build.sh --shell` without the bind mount.

### What the key covers

The key is recomputed from scratch on every build and compared **byte for
byte** against the stored one. Any difference is a miss and a full rebuild.

| Key line | Why it is in the key |
|---|---|
| `schema` | bumped by hand when the cache *format* changes, so an entry written by older code can never be misread by newer code |
| `distro` | `$ROS_DISTRO` |
| `arch`, `machine` | `dpkg --print-architecture` and `uname -m`. An x86_64 tarball unpacked into an aarch64 image is the nightmare case |
| `os` | `ID-VERSION_ID` of the base rootfs — the glibc/libstdc++ era. `build.sh` pins the base by sha256, so this moves only when someone changes the base deliberately |
| `gcc` | the compiler that produced the objects |
| `pkg:ros-humble-fastrtps` | **Fast-DDS.** The library the agent links and whose headers it compiled against |
| `pkg:ros-humble-fastcdr` | Fast-CDR, same reasoning |
| `pkg:ros-humble-rmw-fastrtps-cpp` | the RMW the image actually runs (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) |
| `pkg:ros-humble-rmw-fastrtps-shared-cpp` | its shared half, versioned separately |
| `pkg:ros-humble-rclcpp` | the rest of the C++ ABI the agent is built against |
| `micro_ros_setup` | the exact commit. Resolved with `git ls-remote` *before* the clone when deciding whether to restore, and re-read with `git rev-parse HEAD` from the real checkout when an entry is *written* — so a stored key always names the commit that was really built |

If the commit cannot be resolved (no network), **no restore happens**. An
entry that cannot be keyed is never used.

### What the key does **not** cover

`create_agent_ws.sh` vcs-imports Micro-XRCE-DDS-Agent, micro-ROS-Agent and
micro_ros_msgs **by branch**, from a `.repos` file inside `micro_ros_setup`.
Pinning that repo's commit pins the *file*, not where the branches it names
point today. So a cache entry can hold an agent built from slightly older
upstream commits than a fresh build would produce.

That is staleness of **degree, not of ABI**. It cannot produce the
wrong-Fast-DDS failure below, because every library the agent links comes from
the apt packages that *are* keyed. The resolved commit of every imported repo
is recorded in `uros_ws.info` and printed on every restore, so it is visible
rather than assumed. When upstream moves and you want it, clear the cache.

### Clearing it, and turning it off

```sh
# clear it entirely — the next build pays the ~14 hours and repopulates
rm -rf fpms-os/.cache/uros

# keep the tarball but force a rebuild anyway (the file is checked first)
touch fpms-os/.cache/uros/NOCACHE      # works from the host, today
rm    fpms-os/.cache/uros/NOCACHE      # ...and puts it back

# the env-var opt-out, honoured by both restore and populate
FPMS_UROS_NOCACHE=1
```

**On the env var, specifically:** `build.sh`'s `in_chroot()` uses `env -i` with
an explicit variable list, and `FPMS_UROS_NOCACHE` is not on it — so exporting
it in your shell does **not** reach stage 10. It works when you run the stage
by hand inside `build.sh --shell`, and it would work if it were added to
`CHROOT_ENV`. **The `NOCACHE` marker file is the switch that works from the
host today**, which is exactly why it exists.

### Proving a restore was used rather than a rebuild

Three independent ways, cheapest first.

**1. The build log.** Stage 10 prints exactly one of these:

```
    micro-ROS workspace: RESTORED FROM THE HOST CACHE - no C++ was compiled
    micro-ROS workspace: already present in this image (resumed build)
    micro-ROS workspace: built from source
```

and, on the way there, `cache: HIT`/`cache: MISS` with a reason. On a miss it
prints **both keys**, so the differing line names the cause:

```sh
grep -E 'cache: |micro-ROS workspace:' .build/build-*.log
```

**2. The stage's own elapsed time**, from `build.sh`:

```
==> stage 10-ros-humble.sh OK in 859m57s      <- compiled
==> stage 10-ros-humble.sh OK in 41m18s       <- restored (illustrative)
```

**3. The finished artefact.** A restored workspace says so in its stamp file,
and the agent binary can be checked for architecture directly:

```sh
cat /tmp/chk/home/ubuntu/uros_ws/.fpms-built
# 2026-08-11T02:14:07Z (restored from cache 2026-08-12T19:40:55Z)

AGENT=$(find /tmp/chk/home/ubuntu/uros_ws/install -type f -name micro_ros_agent)
od -An -tx1 -N20 "$AGENT"
#  7f 45 4c 46 02 01 01 00 00 00 00 00 00 00 00 00
#  03 00 b7 00
#              ^^^^^ b7 00 = EM_AARCH64. 3e 00 would be x86-64.
```

Stage 10 makes that same check itself, on both paths, and **fails the build**
rather than shipping a wrong-architecture agent.

### What a stale cache would look like on the rover

This is the failure the key exists to prevent, and it is worth knowing by
sight because **nothing on the rover will say "stale cache"**.

An agent compiled against different Fast-DDS headers than the image ships is
not a build error. It is either:

- **A clean loader failure** — `micro-ros-agent.service` restarts in a loop and
  `journalctl -u micro-ros-agent` shows
  `error while loading shared libraries: libfastrtps.so.2.6: cannot open
  shared object file`. This one is merciful: it names itself.
  Confirm with `ldd "$(command -v micro_ros_agent)"` — look for `not found`.
- **Or, much worse, a link that never establishes.** The agent starts, the
  ESP32 connects at 230400 baud, and no micro-ROS topics ever appear.
  `ros2 node list` is empty of the rover's nodes, `/cmd_vel` is accepted and
  goes nowhere, and the rover simply does not move. This looks identical to
  a cable fault, a wrong serial device, an ESP32 that did not reset, and the
  Fast DDS shared-memory failure in `docs/DDS.md` — all of which this project
  has spent real time on.

**If the micro-ROS link misbehaves after a rebuild, clear the cache and
rebuild before diagnosing anything else.** It costs fourteen hours and it
removes an entire class of cause:

```sh
rm -rf fpms-os/.cache/uros
```

### Disk

The entry is an **estimated** 100–200 MB compressed, and it lives beside the
~837 MB base `.img.xz` in `.cache/`. Add it to §2's budget. Stage 10 reports
the free space on the host filesystem holding the cache after it writes —
that number is *not* the same as the `disk:` lines elsewhere in the stage,
which report the **image's** filesystem.

---

## 4. The stages

`build.sh` globs `scripts/[0-9]*.sh` and runs them in numeric order inside the
chroot. Each is idempotent by intent, and each `die`s the whole build on
failure rather than continuing.

| Stage | Produces | Slow? |
|---|---|---|
| `00-base-system` | user `ubuntu`, hostname, UTC + timesyncd + fake-hwclock, avahi, mosquitto, sshd key-only, ModemManager **purged** | moderate (apt) |
| `10-ros-humble` | ROS 2 Humble, Nav2, slam_toolbox, rosbridge, foxglove-bridge, both RMWs, **and `~/uros_ws` built from source** | **yes — the long pole** |
| `20-python-deps` | apt opencv/pyserial/numpy, pinned `paho-mqtt>=2.0`, `websocket-client`, `/etc/pip.conf` constraints, then **verifies every import and fails the build** | moderate |
| `25-npu-runtime` | `/usr/lib/librknnrt.so` (ELF-checked), `rknn-toolkit-lite2`, `/etc/fpms/npu-versions.json` | minutes |
| `30-fpms-payload` | the rover Python into `/home/ubuntu`, `fpms-rover-agent` and `fpms-uros-supervisor` into `/usr/local/bin`, the `nav2/` and `slam/` trees | fast |
| `40-ros-layer` | DDS profile **parse-checked**, nav2/slam params, generated `arena_map.pgm` diffed against the committed yaml, patched BT XML (`backup_speed 0.025 → 0.18`), `/etc/fpms/ros-versions.txt` | fast |
| `50-overlay` | the overlay onto `/`, load-bearing modes, `visudo -cf`, CRLF guard | fast |
| `60-enable-units` | 18 boot units enabled, 2 masked, the rest deliberately not enabled — then re-verified | fast |
| `90-finalise` | `/etc/fpms/versions.json`, motd, os-release, **machine-id cleared**, apt/log clean, free space zeroed | moderate (the zero-fill) |

Then `finalise()` in `build.sh` unmounts, detaches the loop, compresses, and
writes the `.sha256`.

Three stages fail the build on a *content* check rather than a command error,
and they are the ones worth knowing about:

- **20** verifies that `numpy` is 1.x and that `from paho.mqtt.client import
  Client, CallbackAPIVersion` resolves. An image that fails this boots with the
  STOP authority crash-looping.
- **25** verifies the downloaded `librknnrt.so` really is an aarch64 ELF64
  (not a 404 page with a `.so` name), and that `RKNNLite()` constructs.
- **40** asserts `<useBuiltinTransports>false</useBuiltinTransports>` is in the
  DDS profile. See `docs/DDS.md` — a profile without that line looks correct,
  parses correctly, and fixes nothing.

---

## 5. When it fails

| | Symptom | Cause | Fix |
|---|---|---|---|
| **A** | `Exec format error` on the first command of stage 00, or `chroot: failed to run command` | binfmt handler not registered, **or** `/usr/bin/qemu-aarch64-static` not installed, **or** the interpreter never got copied into the image | `apt install qemu-user-static binfmt-support`; check *both* `/proc/sys/fs/binfmt_misc/qemu-aarch64` and `/usr/bin/qemu-aarch64-static`; confirm `ls .build/mnt/usr/bin/qemu-aarch64-static` exists. `docker run --rm --privileged multiarch/qemu-user-static --reset -p yes` if the registration is stale |
| **B** | `base image checksum MISMATCH - refusing to build` | Upstream re-cut the release under the same filename, or the download truncated | `rm .cache/*.img.xz`, re-download, compare against upstream's published `.sha256` sidecar, then update `BASE_IMAGE_SHA256`. **Never** set `BASE_IMAGE_REQUIRE_SHA=0` to get past it |
| **C** | `no such option: --break-system-packages`, stage 20 or 25 | Ubuntu 22.04 ships pip **22.0.2**. That flag arrived in pip 23.0.1 (it exists to satisfy PEP 668, which 22.04 predates) | The flag is unnecessary on this base. Either `pip3 install -U pip` inside the chroot before the install, or drop the flag. See the note below — this is a live contradiction in the scripts, not a host problem |
| **D** | Second run behaves impossibly: `mount: device busy`, `resize2fs: device in use`, or writes that vanish | A loop device leaked. `build.sh`'s trap covers a normal failure but **not** `SIGKILL`, not a `wsl --shutdown`, and not a host reboot mid-build | `losetup -a`; `mount | grep .build/mnt`; unmount the binds in reverse (`dev/pts, dev, proc, sys, run, boot/firmware`, then the root), then `losetup -d /dev/loopN`. Do this **before** any `rm -rf` on `.build` — the bind mounts walk into the host's `/dev`, `/proc`, `/sys` |
| **E** | `No space left on device` from dpkg or pip, mid-stage-10 | Host disk, not image disk. `ROOTFS_GROW_MB` governs space *inside* the image | Free host space (see §2), then rebuild. Do not raise `ROOTFS_GROW_MB` — it makes the problem worse |
| **F** | `FAILED to enable fpms-<x>.service`, stage 60 exits 1 | Almost always a unit with no `[Install]` section, or a name typo. `fpms-nav2` and both `fpms-slam-*` deliberately have no `[Install]` and are **not** in `BOOT_UNITS` for that reason | Reproduce the exact error interactively (§6) with `systemctl enable <unit>`; check the unit has `[Install] WantedBy=` |
| **G** | `parted` errors about the backup GPT table, or stage 10 runs out of space in an image that should have 6 GB spare | `sgdisk` is not installed. The call is `|| true`, so its absence is silent, and the rootfs was never actually grown | `apt install gdisk`, rebuild from scratch. Confirm with `parted -s <img> print free` that the last partition reaches the end |
| **H** | HTTP 404 during download; or `is not an aarch64 ELF (a 404 page?)` from stage 25 | A pinned URL rotted — the release was renamed, the tag moved, or the asset path changed | Resolve the URL by hand (`curl -sIL <url>`), find the correct asset on the upstream releases page, update `config/fpms-os.conf`, and update the SHA with it |
| **I** | `/usr/bin/env: bad interpreter: No such file or directory` running a stage script that plainly exists | CRLF line endings on a script, from a Windows checkout | `sed -i 's/\r$//'` the offending file; check `.gitattributes` is being honoured (`git config core.autocrlf`) |
| **J** | `FATAL: the Fast DDS profile does not parse` (stage 40), or `useBuiltinTransports is not false` | The DDS profile was edited and broken | Fix `overlay/etc/fpms/fastdds_udp_only.xml`. This check is doing its job — read `docs/DDS.md` before changing it |
| **K** | `MISSING: fpms_missions.py` and friends, stage 30 | `build.sh:226`'s copy of the parent `rover/` tree found nothing, and swallowed the error with `2>/dev/null || true` | The `fpms-os/` directory must sit inside a full `rover/` tree. See §1, WSL |
| **L** | Stage 10 says `cache: NOT populated - no cache directory is visible in the chroot`, and every build pays 14 hours | The `build.sh` bind mount in §3.1 has not been added. This is **not** an error — the stage builds normally and the image is correct | Add the two lines in §3.1. Until then the message is accurate and expected |
| **M** | Stage 10 says `cache: MISS - the key moved` and prints two keys | A keyed input changed: a Fast-DDS/rmw/rclcpp package version, the base OS, gcc, or the `micro_ros_setup` commit | Nothing to fix. **This is the cache working.** The differing line names the cause; the build recompiles and writes a fresh entry |
| **N** | `micro_ros_agent was built, but it is not an aarch64 ELF64` | A host toolchain leaked into the chroot, or binfmt is misconfigured, and the agent was compiled for the build host | Row **A**. Do not work around it — an x86_64 agent in an aarch64 image is undetectable on the rover except as a link that never comes up |
| **O** | The rover's micro-ROS link never establishes after an otherwise-successful rebuild, with no error naming a cause | Possibly a stale `uros_ws` cache: an agent linked against different Fast-DDS than the image ships | `rm -rf .cache/uros` and rebuild **before** diagnosing cables, the ESP32, or DDS. See §3.1, "What a stale cache would look like on the rover" |

### The worked example: a pinned URL that rotted

`config/fpms-os.conf` named the base image

```
ubuntu-22.04.4-preinstalled-server-arm64-orangepi-5b
```

which is a plausible-looking, carefully-typed **404**. Upstream names the asset
with the Ubuntu **series** (`22.04`), never the point release (`22.04.4`). The
file records the correction itself, at lines 30–34:

> An earlier revision of this file said "ubuntu-22.04.4-..." which is a 404 —
> upstream names the asset with the SERIES (22.04), not the point release. That
> is exactly the kind of plausible-looking wrong value the
> `BASE_IMAGE_REQUIRE_SHA` gate exists to stop, and it did.

Two things to take from it.

**Pinned URLs rot, and they rot quietly.** Every URL in `config/fpms-os.conf`
is a bet that a third party will not move a file: the base image, the
`rknn-toolkit-lite2` wheel, `librknnrt.so`, and the ROS apt key. A GitHub
release asset can be re-cut under the same name, a tag can be re-pointed, and a
`/raw/` path can be reorganised. The failure is not always a clean 404 either —
`wget -O dest` **creates `dest` before it knows the request failed**, which is
why stage 25 downloads to a temp file and checks the ELF magic before
installing, and cleans up the zero-byte `librknnrt.so` that stage 20's
`wget -q -O /usr/lib/librknnrt.so` leaves behind on failure
(`25-npu-runtime.sh:254–270`).

**The checksum gate is the thing that made it cheap.** A 404 on the base image
is loud. The dangerous version of the same mistake is a URL that resolves to
*something* — a different point release, a re-cut image, a mirror's stale copy —
and builds fine. `BASE_IMAGE_REQUIRE_SHA=1` is what turns that into a stop.
Leave it at 1.

### The `--break-system-packages` contradiction

Worth stating on its own because it will bite the first person who builds on a
clean 22.04 base. `20-python-deps.sh:66` and `25-npu-runtime.sh:370` both run:

```sh
pip3 install --no-cache-dir --break-system-packages "${PIP_PAHO}" …
```

and stage 20 comments that the flag *"is required on this base"*. On Ubuntu
22.04 the opposite is true: 22.04's `python3-pip` is 22.0.2, the flag was
introduced in pip 23.0.1, and an unrecognised long option makes pip exit 2 —
which under `set -euo pipefail` fails the stage and therefore the build. The
flag is required on 23.04+ / 24.04 bases, where PEP 668 marks the system
environment externally-managed. It is not required here and is not accepted
here.

Until the owner of those scripts resolves it, the way through is §6: enter the
chroot and either upgrade pip or run the installs without the flag.

---

## 6. Resuming — and what `--stage` actually does

`build.sh`'s header advertises:

```
sudo ./build.sh --stage 20      run one stage against the existing chroot
sudo ./build.sh --shell         drop into the chroot to poke at it
sudo ./build.sh --no-download   reuse the cached base image
```

**`--no-download` does what it says.** With `.cache/<base>.img` present it skips
the download and the decompress, which saves several minutes and the 837 MB.

**`--stage N` and `--shell` do not resume anything.** Read `main()`:

```
check_arch → fetch_base → prepare_image → enter_chroot_mounts → …
```

`prepare_image()` runs on *every* invocation, and its first action is
`cp --sparse=always "$base" "$OUT"` — an unconditional copy of the pristine
base image over the working image, followed by `truncate`, `resizepart` and
`resize2fs`. So:

- `--stage 25` runs stage 25 against a **virgin Ubuntu rootfs**, not against
  your five-hour-old chroot. Anything earlier stages had installed is gone.
- `--shell` drops you into that same virgin rootfs.
- `build.sh:237`'s advice — `Fix and re-run with --stage ${base%%-*}` — is
  therefore misleading as written.
- `25-npu-runtime.sh:186–193`'s recipe for a deliberate stream-only image
  (`touch .build/mnt/opt/fpms-os/ALLOW_NPU_MISSING` then `--stage 25`) cannot
  work for two independent reasons: the re-copy above, and `stage_all()`'s
  `rm -rf "$MNT/opt/fpms-os"` which deletes the marker before the stage runs.
  The `FPMS_ALLOW_NPU_MISSING` env var is not in `in_chroot()`'s `env -i` list
  either, so it does not reach the script.

**So when a stage fails after ninety minutes, there is no resume.** Your options
are, in order of preference:

**1. Fix the cause and rebuild.** Correct for anything except a very long
stage-10 failure, and the only path that produces an image whose provenance you
can state.

**2. Re-enter the image by hand and finish it manually.** The working `.img` is
still on disk with everything the completed stages did. Nothing below edits any
build script.

```sh
cd /root/work/rover/fpms-os
IMG=fpms-os-1.0.0-<date>.img
MNT=.build/mnt

LOOP=$(losetup --find --show --partscan "$IMG")
ROOT=${LOOP}p2; [ -e "$ROOT" ] || ROOT=${LOOP}p1
mount "$ROOT" "$MNT"
[ "$ROOT" = "${LOOP}p2" ] && mount "${LOOP}p1" "$MNT/boot/firmware"

mount -t proc  proc  "$MNT/proc"
mount -t sysfs sys   "$MNT/sys"
mount --bind /dev    "$MNT/dev"
mount --bind /dev/pts "$MNT/dev/pts"
mount -t tmpfs tmpfs "$MNT/run"
cp /usr/bin/qemu-aarch64-static "$MNT/usr/bin/"
cp /etc/resolv.conf "$MNT/etc/resolv.conf"
printf '#!/bin/sh\nexit 101\n' > "$MNT/usr/sbin/policy-rc.d"
chmod +x "$MNT/usr/sbin/policy-rc.d"
```

Then enter it with **the same environment `in_chroot()` builds**, because the
stages read those variables and stage 25 hard-fails on an unset
`RKNN_LITE_WHEEL_URL`:

```sh
. config/fpms-os.conf
chroot "$MNT" /usr/bin/env -i \
    HOME=/root PATH=/usr/sbin:/usr/bin:/sbin:/bin TERM=xterm \
    DEBIAN_FRONTEND=noninteractive LC_ALL=C \
    FPMS_OS_VERSION="$FPMS_OS_VERSION" \
    ROS_DISTRO="$ROS_DISTRO" ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
    FPMS_USER="$FPMS_USER" FPMS_HOME="$FPMS_HOME" \
    FPMS_HOSTNAME="$FPMS_HOSTNAME" \
    PIP_PAHO="$PIP_PAHO" PIP_NUMPY="$PIP_NUMPY" \
    PIP_WEBSOCKET="$PIP_WEBSOCKET" \
    RKNN_LITE_WHEEL_URL="$RKNN_LITE_WHEEL_URL" \
    LIBRKNNRT_URL="$LIBRKNNRT_URL" \
    RKNN_TOOLKIT2_TAG="$RKNN_TOOLKIT2_TAG" \
    /bin/bash
```

Inside, run the remaining stages by hand — they are already staged at
`/opt/fpms-os/scripts/` from the failed run:

```sh
/opt/fpms-os/scripts/30-fpms-payload.sh
/opt/fpms-os/scripts/40-ros-layer.sh
…
```

Then exit and finish exactly what `finalise()` does, in this order:

```sh
rm -f "$MNT/usr/sbin/policy-rc.d" "$MNT/usr/bin/qemu-aarch64-static"
for m in dev/pts dev proc sys run boot/firmware; do umount -lf "$MNT/$m"; done
umount -lf "$MNT"
losetup -d "$LOOP"
xz -T0 -6 -f "$IMG"
sha256sum "$IMG.xz" > "$IMG.xz.sha256"
```

**An image finished this way is not a clean build.** Say so wherever it goes.
It is a debugging tool for getting to a testable image after a five-hour stage
10, not a release path.

**3. If the failure is in the micro-ROS build specifically**, `--shell` plus the
manual mount above lets you run `build_agent.sh` interactively and *see* its
output, which `10-ros-humble.sh` discards. That alone is often the whole
diagnosis.

---

## 7. Verifying the output before you flash it

Four checks, cheapest first. None needs a Pi.

**1. The structural checks — before you build, and again after.**

```sh
python3 selftest/test_image_offline.py
```

Twelve checks against the tree. `exec_paths_exist` is the one that justifies the
file: every `ExecStart`/`ExecStartPre` must point at something the image
contains. That is the defect that shipped for months.

**2. The artefacts exist and match.**

```sh
ls -l fpms-os-1.0.0-<date>.img.xz fpms-os-1.0.0-<date>.img.xz.sha256
sha256sum -c fpms-os-1.0.0-<date>.img.xz.sha256
xz -t fpms-os-1.0.0-<date>.img.xz          # integrity of the container itself
```

A `.xz` in the 2–3 GB range is the expected shape. **A much smaller one is a
signal**, not a win: it usually means the rootfs never actually grew (§5 row G)
or a stage silently installed nothing.

**3. Mount the produced image and spot-check it.** Decompress a copy, loop-mount
it read-only, and confirm the things the whole build exists to guarantee:

```sh
xz -dkc fpms-os-1.0.0-<date>.img.xz > /tmp/check.img
LOOP=$(losetup --find --show --partscan /tmp/check.img)
mkdir -p /tmp/chk && mount -o ro ${LOOP}p2 /tmp/chk

# units enabled — symlinks under multi-user.target.wants
ls /tmp/chk/etc/systemd/system/multi-user.target.wants/ | grep fpms

# the two /cmd_vel writers must be masked, i.e. symlinks to /dev/null
ls -l /tmp/chk/etc/systemd/system/fpms-ros-tunnel.service \
      /tmp/chk/etc/systemd/system/fpms-rtos-follower.service

# payload landed
ls -l /tmp/chk/home/ubuntu/fpms_missions.py \
      /tmp/chk/usr/local/bin/fpms-rover-agent \
      /tmp/chk/usr/local/bin/fpms-uros-supervisor \
      /tmp/chk/home/ubuntu/uros_ws/install/setup.bash

# the build record, and the NPU chain
cat /tmp/chk/etc/fpms/versions.json
cat /tmp/chk/etc/fpms/npu-versions.json      # status must be INSTALLED
cat /tmp/chk/etc/fpms/ros-versions.txt

# the DDS profile, the one line that matters
grep useBuiltinTransports /tmp/chk/etc/fpms/fastdds_udp_only.xml

# machine-id must be EMPTY, or every flashed board shares one
wc -c /tmp/chk/etc/machine-id

umount /tmp/chk && losetup -d "$LOOP" && rm /tmp/check.img
```

`~/uros_ws/install/setup.bash` deserves the explicit check.
`micro-ros-agent.service` sources it under `set -e`, so if stage 10's colcon
build silently produced nothing, the wrapper dies instantly and the drive link
never comes up — with no useful message. Stage 10 asserts this at line 99; check
it again on the artefact.

**But `setup.bash` alone is not enough, and never was.** The *first* colcon
build — `micro_ros_setup` itself — creates it, so it exists even when the agent
never built. The thing `fpms-uros-agent-run` actually execs is the
`micro_ros_agent` executable, so check for that, and check its architecture:

```sh
AGENT=$(find /tmp/chk/home/ubuntu/uros_ws/install -type f -name micro_ros_agent)
echo "${AGENT:?no micro_ros_agent in the image}"
od -An -tx1 -N20 "$AGENT"      # byte 4 = 02, bytes 18-19 = b7 00 (EM_AARCH64)

# and where this workspace came from — source build, or restored cache (§3.1)
cat /tmp/chk/home/ubuntu/uros_ws/.fpms-built
```

Stage 10 enforces both of these itself now, on every path including a cache
restore, and fails the build rather than shipping a workspace that does not
verify.

**4. Then `docs/FLASHING.md`,** and `fpms-selftest` on the board.

---

## 8. What the build does not do

The build produces an image that **has never been booted**. Not on the target
board, not in a VM. Everything above is structural: files exist, imports
resolve, XML parses, units are enabled. That is worth a lot, and it is not the
same thing as working.

This belongs in `docs/FAILURE_MODES.md`'s register — reproduced here in its
format so the boundary is stated where the build is documented:

| | Status |
|---|---|
| **The image has never been booted** | Not once, by anything. No stage runs the kernel, no stage runs systemd. Stage 60 writes `enable` symlinks; it never observes a unit start. |
| **No NPU** | There is no NPU device in a chroot and no `.rknn` model in the repository. Stage 25 proves the wheel imports and `RKNNLite()` constructs — that is *all*. `load_rknn()`, `init_runtime()`, core masks, latency, and thermals are untested. `25-npu-runtime.sh:450–472` says the same thing at length. |
| **The kernel `rknpu` driver version cannot be read at build time** | It is exposed at `/sys/kernel/debug/rknpu/version` on a **live** kernel, and the chroot's `/sys` is the **build host's**, bind-mounted by `build.sh`. Reading it here reports the x86 workstation. `npu-versions.json` records `driver.expected: null` and defers the comparison to `fpms-selftest` on the board. |
| **No serial devices** | Both CP2102s, the udev topology rules, the ESP32 reset behaviour, and the 90–225 s micro-ROS re-link are all untestable here. The rules are installed and syntactically valid; that is the entire claim. |
| **No DDS** | Nothing publishes or subscribes during the build. Stage 40 checks the profile **parses** and that `useBuiltinTransports` is false. Whether data crosses process eras is `fpms-selftest`'s `DDS carries data across eras`, on hardware, and `docs/DDS.md` says explicitly that a malformed profile is silently ignored. |
| **No network, no MQTT, no WiFi** | `policy-rc.d` returns 101 for the whole build precisely so nothing starts. mosquitto is installed and enabled and has never accepted a connection. The broker password does not exist until first boot. |
| **No hardware-dependent measurement** | The LiDAR mount transform, `LIDAR_ROTATION_SIGN`, the calibration profile and the track width remain unmeasured. The image ships them unmeasured and **says so**; the units that would act on them refuse to start. A build cannot change that, and should not try. |
| **A successful build is not a working rover** | It is a rover-shaped image whose dependencies resolve. The first real verdict is `fpms-selftest` on the board — and the second is the operator's eyes, not odometry. |
