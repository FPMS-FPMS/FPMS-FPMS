# Updating FPMS-OS

> **Ownership note.** The sections between the `apt/holds` markers below were
> written by the agent that owns `overlay/etc/apt/**` and
> `scripts/70-update-policy.sh`. Other sections of this file (the image-update
> tooling, `fpms-update`, the systemd side) belong to another owner. **Add
> alongside; do not rewrite across the markers.**

<!-- BEGIN apt/holds sections — owner: scripts/70-update-policy.sh + overlay/etc/apt/** -->

## Why `apt upgrade` is dangerous on this board

FPMS-OS is built on Joshua Riek's `ubuntu-rockchip` Ubuntu 22.04 arm64 image.
**Its kernel is a Rockchip BSP kernel, and that kernel is the only reason this
hardware works:**

| Kernel component | What it is | What its loss looks like |
|---|---|---|
| `rknpu` | the NPU driver | no `/sys/kernel/debug/rknpu`, `librknnrt` cannot init, **no fire detection** — and the agent degrades *silently*: it logs "NPU unavailable; streaming without detection" and carries on, so systemd still says `active` |
| Mali GPU stack | vendor GPU blobs, version-matched to the driver | GPU-accelerated paths fail |
| `bcmdhd` | the WiFi chip driver | **no association at all**: no `FPMS_Net`, no MQTT, no rosbridge, no SSH. The rover is simply unreachable, and it looks exactly like a flat battery |

Stock Ubuntu kernels contain none of these. So a routine `apt upgrade` that
pulls a generic `linux-image-*` produces a rover with no NPU, no WiFi and
possibly no boot — and it does it **silently, on a machine with no monitor
attached.** The same applies to `u-boot-*`: its postinst writes the bootloader
to raw sectors of the boot device.

This is not hypothetical, and it is not solved by "just being careful": the
whole point of a policy is that it holds when someone is tired, at a venue, the
night before a run.

## The policy, in four files plus one generated

| File | Written by | What it does |
|---|---|---|
| `/etc/apt/preferences.d/fpms-kernel` | overlay (stage 50) | every version of the kernel / bootloader / BSP package families → **priority −1** (never installable) |
| `/etc/apt/preferences.d/fpms-kernel-installed` | **generated** by `scripts/70-update-policy.sh` | the exact versions *this image shipped with* → **priority 1000** (the candidate) |
| `/etc/apt/preferences.d/fpms-numpy-ceiling` | overlay (stage 50) | `python3-numpy` 2.x and 3.x → **priority −1** |
| `/etc/apt/apt.conf.d/70fpms-update-policy` | overlay (stage 50) | no timer-driven apt activity at all |
| `/etc/apt/apt.conf.d/71fpms-unattended-upgrades` | overlay (stage 50) | if unattended-upgrades runs: **security-only, never reboots** |

Plus `apt-mark hold` on the same package set, applied at build time.

### Pins, not just holds — and why that distinction matters

`apt-mark hold` is a **dpkg selection flag** living in `/var/lib/dpkg/status`.
It is state, not configuration:

- `apt-get --ignore-hold`, `-o APT::Ignore-Hold=1` and
  `apt --allow-change-held-packages` all walk straight past it;
- it does not survive a rootfs that is restored, re-flashed or rebuilt;
- nothing in this repository would recreate it.

A **pin file is part of the image**. It is in git, the overlay copies it in on
every build, and apt honours it for `install`, `upgrade`, `full-upgrade`,
`dist-upgrade` *and* for unattended-upgrades, which resolves through the same
engine. `--ignore-hold` does not touch it.

So: **the hold is a courtesy** (it makes `apt-mark showhold` print the
protected set in one line, which a pin file cannot); **the pin is the
guarantee.** Stage 70 proves this distinction rather than asserting it — it
runs `apt-get -s dist-upgrade --ignore-hold` at build time and fails the build
if apt still wants to touch a frozen package with every hold disregarded.

### How the two pin files cooperate

`apt_preferences(5)` resolves **specific-form** records (`Pin: version …`)
before **general-form** records (`Pin: release …`), regardless of filename
order. That is the whole design:

```
fpms-kernel             general form,  Pin: release *      → -1   every version
fpms-kernel-installed   specific form, Pin: version <exact> → 1000  the one installed
```

Result: the version this image was built with is the candidate; every other
version — including every generic Ubuntu kernel — is un-installable.

`fpms-kernel-installed` is pinned at **1000, not 1001**. Above 1000 is what
permits a *downgrade*. `scripts/20-python-deps.sh` wants that for numpy (if a
2.x has already landed, apt must be willing to go backwards). Here the opposite
is true: if an operator has deliberately upgraded the kernel, a 1001 pin would
make the very next `apt install anything` quietly propose downgrading the
running kernel back. A kernel downgrade nobody asked for, on a headless board,
is the same catastrophe arriving through the door marked "safety".

### Filenames have no extension, deliberately

apt reads a file in `preferences.d` only if it ends in `.pref` **or has no
extension at all**; anything else is skipped, and for `.disabled`, `.bak`, `~`
and `.dpkg-*` it is skipped *in silence*. Same rule in `apt.conf.d` with
`.conf`. Never rename these files to `.conf` — they would stop working and say
nothing. (This is also what makes the escape hatch below a one-liner.)

## The frozen package set

The **exact** names are discovered at build time from the image itself
(`dpkg-query`), not guessed in this repository, and they are recorded in two
places you can read on the rover:

```bash
apt-mark showhold                                # the names
cat /etc/apt/preferences.d/fpms-kernel-installed  # the names AND the versions
```

The **patterns** the freeze covers, and why each is there:

| Pattern | Why |
|---|---|
| `linux-image-*` `linux-headers-*` `linux-modules-*` | the kernel itself, BSP flavour and every generic Ubuntu kernel |
| `linux-generic*` `linux-virtual*` `linux-lowlatency*` | the metapackages — how a generic kernel arrives as *somebody else's dependency* rather than as a direct install |
| `linux-rockchip*` `linux-*-rockchip*` | Riek's flavour naming, both orders |
| `linux-firmware` | **UNVERIFIED**, see below |
| `u-boot*` `flash-kernel*` | postinsts that write raw boot sectors / regenerate `extlinux.conf` |
| `*rockchip*` | `rockchip-multimedia-config`, `librockchip-mpp*`, … |
| `libmali*` `mali-*` | vendor GPU blobs, version-matched to the BSP driver |
| `*bcmdhd*` | speculative; a no-op if no such package exists |

**Deliberately not frozen:** `linux-libc-dev` (userspace headers; freezing it
blocks unrelated builds and it cannot break the board), `initramfs-tools`, and
every network-facing daemon — `mosquitto`, `openssh-server` and friends are
exactly the things you *want* patched. The freeze protects what breaks the
board, not what makes updating worthwhile.

### `linux-firmware` is UNVERIFIED

The `bcmdhd` firmware blobs this board's WiFi needs are **not** in Ubuntu's
`linux-firmware`, so the base image supplies them from somewhere. If it
supplies them at paths `linux-firmware` also owns, then upgrading
`linux-firmware` replaces them with Ubuntu's versions and the WiFi stops.
**That provenance cannot be determined from this repository** — the base `.img`
is not a text file anyone can grep. It is frozen on the pessimistic reading:
the cost of freezing is a firmware update this rover does not need; the cost of
being wrong the other way is a rover that never joins the hotspot again.

To settle it on a real board:

```bash
dpkg -S /lib/firmware/brcm/ 2>/dev/null | sort -u   # which package, if any, owns them
dpkg -l | grep -iE 'firmware|bcm|brcm|ap6'
```

If the blobs turn out to belong to a Rockchip/Riek package (already covered by
`*rockchip*`) or to no package at all, `linux-firmware` can be dropped from the
freeze.

## Deliberately upgrading a held package

There is no situation in which this should be done for the first time on the
morning of a run. Do it on the bench, with the board reachable over a wired
serial console if you have one, and **have a flashable image ready** — the
recovery path for a bad kernel on this board is re-flashing the card, not
fixing it in place.

```bash
# 1. See exactly what is protected and what apt would do without the holds.
apt-mark showhold
sudo apt-get -s dist-upgrade --ignore-hold

# 2. Drop the dpkg hold for the one package you mean to move.
sudo apt-mark unhold <package>

# 3. Disable the pin. Renaming it OUT of the recognised extensions is the
#    reversible way; apt ignores '.disabled' silently and completely.
sudo mv /etc/apt/preferences.d/fpms-kernel \
        /etc/apt/preferences.d/fpms-kernel.disabled
#    ...and, if the package you are moving is the one named there:
sudo mv /etc/apt/preferences.d/fpms-kernel-installed \
        /etc/apt/preferences.d/fpms-kernel-installed.disabled

# 4. Check apt now sees what you expect BEFORE installing anything.
apt-cache policy <package>

# 5. Do it, naming the exact version. Never a bare `apt upgrade` here.
sudo apt-get install <package>=<version>

# 6. PUT THE POLICY BACK. This is the step that gets skipped.
sudo mv /etc/apt/preferences.d/fpms-kernel.disabled \
        /etc/apt/preferences.d/fpms-kernel
sudo apt-mark hold <package>
```

If you changed a kernel/bootloader/BSP package, `fpms-kernel-installed` now
names a version that is no longer installed. That is not fatal — the new
package stays installed and the rover still boots — but it will show
`Candidate: (none)`, and the freeze no longer names reality. Regenerate it:

```bash
dpkg-query -W -f='${Package}|${Version}\n' | while IFS='|' read -r p v; do
  case "$p" in
    linux-image-*|linux-headers-*|linux-modules-*|linux-generic*|\
    linux-virtual*|linux-lowlatency*|linux-rockchip*|linux-*-rockchip*|\
    linux-firmware|u-boot*|flash-kernel*|*rockchip*|libmali*|mali-*|*bcmdhd*)
      printf 'Package: %s\nPin: version %s\nPin-Priority: 1000\n\n' "$p" "$v" ;;
  esac
done | sudo tee /etc/apt/preferences.d/fpms-kernel-installed >/dev/null

apt-cache policy <package>     # Installed: and Candidate: must now match again
```

(That loop is the same pattern list `scripts/70-update-policy.sh` uses. Note it
lists *installed* packages only, which is correct for this file — the
`-1` blanket for everything else lives in `fpms-kernel` and needs no
regeneration.)

Then verify the whole policy is intact again with the commands under
*Verifying the policy on a running rover*.

Anything you change by hand on the rover is lost at the next re-flash. **If a
package genuinely must move, change it in `scripts/70-update-policy.sh` /
`overlay/etc/apt/` and rebuild the image** — otherwise the next flash silently
reverts you and nobody will remember why.

## numpy stays on 1.x, and apt is only half of that

ROS Humble's C extensions are built against the **NumPy 1.x ABI**. A 2.x breaks
`tf_transformations` with `np.maximum_sctype was removed in the NumPy 2.0
release`, reported from a module nowhere near the cause;
`selftest/verify_image.sh` fails such an image outright, because 2.x "would
break every ROS C extension in the image".

Three files, three different jobs:

| File | Written by | Job |
|---|---|---|
| `/etc/apt/preferences.d/fpms-numpy` | stage 20 | pins the **1.x** versions to 1001 — makes apt *want* the 1.x |
| `/etc/apt/preferences.d/fpms-numpy-ceiling` | overlay (stage 50), verified by stage 70 | pins **2.x and 3.x** to −1 — makes apt *refuse* the 2.x |
| `/etc/pip.conf` → `/etc/fpms/pip-constraints.txt` | stage 20 | `numpy<2` for **every pip install on this image** |

The ceiling exists because a pin can only assign priority to versions that
**exist**. If an archive ever offers a 2.x and no 1.x — a rebuild, an operator's
PPA, a moved base image — the 1001 pin matches nothing, the 2.x sits at the
default 500, the installed 1.21.5 sits at 100, and `apt upgrade` walks straight
over it. The ceiling names the versions that must never be installed, so it
keeps working on exactly the day the other pin stops. The two are disjoint by
construction (1.x versus 2.x/3.x), which is why they cannot fight each other —
keep them disjoint if you edit either.

Both spellings are pinned (`2.*` and `1:2.*`): Debian's numpy carries **epoch
1**, apt matches the version string exactly as it prints it, and apt's version
matcher understands exactly one wildcard — a *trailing* `*`. `1:2.0.0` does not
begin with `2`.

### Can the pip numpy and the apt numpy fight? Yes — and apt always loses

They are different files in different directories and **both can be installed
at once**:

```
/usr/local/lib/python3.10/dist-packages/numpy     <- pip     (comes FIRST on sys.path)
/usr/lib/python3/dist-packages/numpy              <- apt     (python3-numpy)
```

pip does **not** uninstall apt's copy, and `/usr/local/...` precedes
`/usr/lib/python3/...` on `sys.path`. So a single `pip install -U numpy` gives
every FPMS service a NumPy 2.x, while `apt` continues to report
`python3-numpy 1.21.5` installed and **every apt-side check in this document
still passes.** The apt pins have no authority there at all; `/etc/pip.conf`'s
constraint is the only thing standing in the way, and a user-local copy under
`~/.local` beats both (that is the failure that broke `tf_transformations` on
the old Pi).

The only check that answers the real question asks Python, not apt:

```bash
python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"
sudo -u ubuntu python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"
```

Both must print a `1.x` **and** a path under `/usr/lib/python3/dist-packages`.

## Automatic updates: the decision, and the argument for it

**Decision: no apt activity on a timer. Ever. No automatic reboots, ever. The
security-only unattended-upgrades configuration is installed, live and
verifiable — but the trigger is a human, not a clock.**

Concretely, `/etc/apt/apt.conf.d/70fpms-update-policy` sets
`APT::Periodic::Update-Package-Lists`, `Download-Upgradeable-Packages` and
`Unattended-Upgrade` to `0` (plus `APT::Periodic::Enable "0"` as a master
switch), so `apt.systemd.daily` does nothing when the timers fire.

The argument is not "updates are bad". It is that on **this** board every
property that makes unattended updating safe on a server is missing:

1. **Power is a switch.** This is a battery robot that gets switched off
   abruptly by a human at the end of a run. dpkg unpacking at that moment
   leaves a half-configured package and `/var/lib/dpkg/updates`, and the next
   apt refuses outright with *"dpkg was interrupted"*. `scripts/00-base-system.sh`
   carries a whole repair block for that state because it keeps happening **on
   the build host, where somebody is watching.** On the rover the symptom
   surfaces as a service that will not reinstall on race morning.
2. **The link is already saturated.** `FPMS_CAMERA_FPS=6` and
   `FPMS_JPEG_QUALITY=40` are shipped deliberately low because higher settings
   saturated WiFi and **starved the LiDAR feed**. A background apt download is
   tens to hundreds of megabytes over that same hotspot. The failure it
   produces is not "the update was slow", it is an obstacle guard going stale
   mid-mission.
3. **Restarts are not cheap here.** A package upgrade restarts services. The
   micro-ROS serial link costs **90–225 s** to re-establish and takes the
   rover's only pose source with it; a restart of `fpms-cored` is a window with
   no STOP authority. Neither is acceptable at a moment chosen by a timer.
4. **No console.** Anything that goes wrong unattended is invisible until
   someone notices the rover is unreachable — which is the same symptom as a
   flat battery, a wrong SSID, and a dead WiFi driver.
5. **The venue is exactly when it would fire.** The rover normally lives on the
   operator laptop's hotspot at `192.168.137.1` with no route to the internet.
   The days it *does* have internet are the days someone took it somewhere.

**And the other side, honestly:** never patching is a real cost. `mosquitto`,
`rosbridge`, `fpms_console` and `sshd` all bind `0.0.0.0`, and a shared venue
network is genuine exposure. So the policy removes the **timer**, not the
capability. `71fpms-unattended-upgrades` still defines exactly what a security
update may touch:

- **Allowed origins: security pockets only** — `${distro_codename}-security`
  and the two ESM security pockets. Not `-updates`, not `-backports`, and
  **not** `packages.ros.org` (an automatic `ros-humble-*` upgrade would change
  Nav2 / slam_toolbox / rclpy underneath configs that were diffed against exact
  versions).
- **Package blacklist** covering the kernel / bootloader / BSP families and
  `python3-numpy` — defence in depth behind the pins. Note its limit: these are
  Python regexes applied with `re.match()`, i.e. anchored at the *start* of the
  name, so `rockchip` matches `rockchip-multimedia-config` but **not**
  `librockchip-mpp1`. The pin file's `*rockchip*` glob is what catches that one.
  Read `apt-mark showhold`, not this list, as the protected set.
- `Automatic-Reboot false`, `Automatic-Reboot-WithUsers false`,
  `InstallOnShutdown false`, and all three `Remove-*` options false —
  `Remove-Unused-Kernel-Packages` in particular, because the "unused" kernel it
  would remove is the BSP kernel (nothing declares a dependency on it).
- `OnlyOnACPower false` and `Skip-Updates-On-Metered-Connections false`. Both
  default to `true`, and on a battery robot on a phone hotspot that means a
  manual run would do **nothing and report success** — the one outcome this
  project bans. With the timer off, every run is a human waiting for an answer,
  and they must get the real one.

So, to take security updates deliberately:

```bash
sudo apt-get update
sudo unattended-upgrade --dry-run -v     # exactly what it WOULD do
sudo unattended-upgrade -v               # do it
sudo systemctl status fpms-cored fpms-missions micro-ros-agent   # then check
```

Do this on the bench, between runs, with time to power-cycle and re-verify —
never between heats.

If `unattended-upgrades` is not installed on your image (the build log from
stage 70 says which), the equivalent deliberate path is a plain
`sudo apt-get update && sudo apt-get upgrade`: the pins protect it identically,
because they protect apt itself rather than any one front end.

### What is deliberately *not* done

- **`apt-daily.timer` / `apt-daily-upgrade.timer` are left enabled.** They are
  systemd units and this image's units have a single owner. They still fire —
  and `apt.systemd.daily` reads the config above and exits without touching the
  network or dpkg. Confirm with `systemctl list-timers 'apt-daily*'` and
  `journalctl -u apt-daily-upgrade`.
- **`do-release-upgrade` is not blocked.** A release upgrade would be
  catastrophic here (Ubuntu 22.04 and Python 3.10 are load-bearing for ROS
  Humble), and `/etc/update-manager/release-upgrades` with `Prompt=never` is
  the way to stop it — but that path is outside this policy's ownership. **It
  is an open gap; nothing on the rover currently prevents an operator running
  `do-release-upgrade`.**
- **`needrestart` is not configured.** If it is installed, it can restart
  services after an upgrade — including `micro-ros-agent` (90–225 s) — on its
  own judgement. `/etc/needrestart/` is outside this policy's ownership. Check
  `dpkg -l needrestart` and consider `$nrconf{restart} = 'l';` (list only).

## Verifying the policy on a running rover

None of this is worth anything if it is assumed rather than checked. Every one
of these is safe to run on a live rover — the simulations change nothing:

```bash
# What is frozen, and at exactly what version
apt-mark showhold
cat /etc/apt/preferences.d/fpms-kernel-installed

# Does apt agree? Installed: and Candidate: must be IDENTICAL for each.
apt-cache policy $(apt-mark showhold)

# THE test. --ignore-hold disregards every dpkg hold, so anything that still
# refuses to move is the PIN doing the work. This must list no kernel,
# bootloader or BSP package at all.
sudo apt-get -s dist-upgrade --ignore-hold

# numpy: apt's view, then the only view that matters
apt-cache policy python3-numpy
python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"

# The automatic-update policy as apt actually merged it
apt-config dump | grep -E 'APT::Periodic|Unattended-Upgrade'

# The timers exist and do nothing
systemctl list-timers 'apt-daily*'
journalctl -u apt-daily-upgrade --no-pager | tail
```

`scripts/70-update-policy.sh` runs the same checks at build time and **fails
the build** if any of them comes out wrong — including a build-time
`apt-get -s dist-upgrade --ignore-hold` that must not name a single frozen
package. If it passed at build time and fails on the rover, something changed
the image after flashing; the pin files and `apt-mark showhold` are the first
two places to look.

<!-- END apt/holds sections -->

<!-- BEGIN payload sections — owner: overlay/usr/local/bin/fpms-update -->

## The other half: updating the FPMS payload

Everything above concerns **apt**. This half concerns **our code**, and the two
are deliberately never conflated — different subcommands, different risk,
different cadence:

| | what changes | how often | tool |
|---|---|---|---|
| **the FPMS payload** | `fpms_missions.py`, `fpms_lidar_ros.py`, `stack/`, `nav2/`, `slam/`, the dashboard, the two `/usr/local/bin` entry points | constantly, including at a competition | `fpms-update payload` |
| **the OS** | apt packages: kernel, ROS Humble, numpy, the RKNN runtime | rarely, and dangerously | `fpms-update os`, under the pins and holds documented above |

Conflating them is how "I fixed one line in the mission executor" becomes "and
it also pulled a generic kernel and now there is no NPU, no WiFi and no boot".

`fpms-update os` **reads and obeys** the policy documented above. It prints
`apt-mark showhold` before it does anything and refuses to continue if it
cannot read it; it uses `upgrade`, never `full-upgrade`, so apt may not remove
a package to satisfy a dependency; it never passes
`--allow-change-held-packages`; it writes no apt configuration of any kind; and
it compares the hold set before and after, shouting if it changed. To move a
held package deliberately, follow *"Deliberately upgrading a held package"*
above — not this tool.

Until `fpms-update` existed, the only way to change one line of
`fpms_missions.py` on a flashed rover was to build a new image and reflash the
card: a fourteen-hour answer to a one-character question, on a competition
machine running code that is still being fixed.

---

## Operator workflow: updating the FPMS payload

### Before you touch anything

```bash
fpms-update status          # what is live, and what you could roll back to
fpms-selftest               # the baseline. Write down the FAIL/WARN counts.
```

Take the `fpms-selftest` baseline **first**. This rover legitimately reports
FAILs that have nothing to do with any update — a missing
`yolo26n-rk3588.rknn` (it is in no repository), an unmeasured LiDAR mount
transform, a broker password first boot never provisioned. Without a baseline
you cannot tell which of those your update caused, and `fpms-update` will not
guess for you: **it never rolls back on a self-test result.** It rolls back on
a unit that was active before and is not active after, which is a causal link.

### From a USB stick — the arena path

`FPMS_Net` is an arena network and may have **no route to the internet at
all**. The USB path is the first-class one, not a fallback.

On the laptop, put the tree on a stick as either:

* a directory named `fpms-payload/` (or `rover/`) containing `fpms_missions.py`
  and `stack/`; **or**
* a whole checkout — `fpms-update` finds `cloud/dashboard/rover/` inside it and
  the dashboard at its sibling `cloud/dashboard/fpms-dashboard/`; **or**
* a tarball named `fpms-payload*.tar.gz`.

On the rover:

```bash
sudo fpms-update check   --from usb      # rehearse it. Changes nothing.
sudo fpms-update payload --from usb      # do it.
```

`check` runs the *same code path* as `payload` up to the moment of the swap —
same staging, same CRLF normalisation, same byte-compile, same diff, same list
of units that would be restarted. It is a rehearsal, not a different program.

### From a local path or a git checkout

```bash
sudo fpms-update payload --from /home/ubuntu/incoming/rover
sudo fpms-update payload --from /media/ubuntu/STICK/fpms-payload.tar.gz
sudo fpms-update payload --from git:https://example/fpms.git --ref fix/turn-sign
```

`git:` is the **last** option, not the first. It runs a network preflight
before doing anything: four TCP connects spread over about 25 seconds, because
the measured behaviour of this board's WiFi is a carrier flap at association
(gained 16:07:12, lost 16:07:12, regained 16:07:13 — measured 2026-08-06) and
`bcmdhd` re-enabling power save every 30–60 s, which flapped MQTT on a measured
47 s cycle. A single connect is not a fair test, and a tool that hangs on a
socket at a competition is worse than one that says no. Every network call has
an explicit timeout; nothing in `fpms-update` can block forever.

If the preflight fails you get a message naming the host and telling you to use
the stick. You do not get a hang.

### What the tool actually does, in order

1. **Materialise the source** — mount-point scan, tarball unpack (absolute
   paths and links inside the archive are refused), or `git clone --depth 1`.
2. **Resolve the layout** — staged (`fpms_missions.py` at the top) or repo
   (`cloud/dashboard/rover/`). Both work; neither is guessed silently.
3. **Stage** into a scratch tree. No destination is touched. CRLF to LF on
   text-shaped files only (`.py .yaml .json .xml .md .html` and `fpms-*`),
   never on binaries — the payload comes off a Windows checkout and
   `fpms-os/.gitattributes` does not reach the rover tree.
4. **Verify** — `py_compile` with `doraise=True` on **every** Python file, into
   a scratch cache so no `__pycache__` is left beside a source. Zero-byte files
   are rejected. Anything systemd `ExecStart=`s directly must still have an
   intact `#!` with no CR on it.
5. **Diff** against what is live, and print it.
6. **The mission gate** (its own section below). If it refuses, nothing has
   been touched.
7. **Snapshot** every destination it is about to write, into a new generation
   under `/var/lib/fpms/updates/`. **If the snapshot fails, the update does not
   happen** — an update with no rollback is not an update this tool performs.
8. **Install** — every file written as `<dst>.fpms-new`, `chmod`'d, `chown`'d
   and `fsync`ed *first*, then a tight loop of `os.replace()`.
9. **Restart** only the units whose files actually changed, `fpms-cored` first,
   verifying each is `active` a couple of seconds later — `systemctl restart`
   returning 0 means systemd accepted the job, not that the unit is running.
10. **Self-test**, after a settle delay, and report.

If a unit that was **active before** the update is not active after, the update
is **rolled back automatically** and the units restarted again. A unit that was
already broken beforehand does not trigger that; a false alarm is what teaches
an operator to stop believing the tool.

### Rolling back

```bash
sudo fpms-update rollback                  # to the state before the last update
sudo fpms-update rollback --to gen-20260813T101500
fpms-update status                         # lists every generation
```

Rollback restores the **exact bytes, modes and owners** that were on this
rover, and **removes files the update created**. That last part matters: a
rollback that left new files sitting beside old ones would leave the rover in a
state neither version was ever tested in.

Nothing is deleted automatically. Generations accumulate (a few MB each) until
you run `sudo fpms-update prune --keep 5`. A rollback target you did not know
had been deleted is exactly what you do not want at a competition.

### If something goes wrong

```bash
fpms-update verify        # does the LIVE payload compile? does it match its manifest?
fpms-update status
journalctl -u fpms-missions -b --no-pager | tail -60
fpms-doctor wont-move
```

`fpms-update verify` also reports **drift**: files that differ from what the
last update installed. A hand edit or a stray `scp` is not necessarily wrong,
but it means rollback will not put back what you think it will.

---

## The FPMS payload: what it is and where it goes

### The manifest

`fpms-update` installs to **exactly the destinations, modes and owners that
`scripts/30-fpms-payload.sh` uses at image build time.** That is not a
courtesy: a file this tool puts somewhere the image build does not is a file
that survives until the next reflash and then vanishes with no message.

| source | destination | mode | owner |
|---|---|---|---|
| `fpms_missions.py` | `~/fpms_missions.py` | 0755 | `ubuntu` |
| `fpms_lidar_ros.py` | `~/fpms_lidar_ros.py` | 0755 | `ubuntu` |
| `fpms_teleop.py` | `~/fpms_teleop.py` | 0755 | `ubuntu` |
| `fpms_odom_tf.py` | `~/fpms_odom_tf.py` | 0755 | `ubuntu` |
| `fpms_duty_driver.py` | `~/fpms_duty_driver.py` | 0755 | `ubuntu` |
| `fpms_cloud_uplink.py` | `~/fpms_cloud_uplink.py` | 0755 | `ubuntu` |
| `fpms_yolo26_npu.py` | `~/yolo/fpms_yolo26_npu.py` | 0755 | `ubuntu` |
| `stack/fpms_cored.py` | `~/fpms_cored.py` | 0755 | `ubuntu` |
| `stack/fpms_charact.py` | `~/fpms_charact.py` | 0755 | `ubuntu` |
| `STACK.md` | `~/STACK.md` | 0644 | `ubuntu` |
| `fpms_ros_tunnel.py`, `fpms_rtos_follower.py` | `~/` | 0755 | `ubuntu` |
| `fpms-rover-agent.py` **or** `fpms_rover_agent.py` | `/usr/local/bin/fpms-rover-agent` | 0755 | `root` |
| `stack/fpms-uros-supervisor` | `/usr/local/bin/fpms-uros-supervisor` | 0755 | `root` |
| `fpms-dashboard/` | `~/fpms-dashboard/` | tree | `ubuntu` |
| `nav2/`, `slam/` | `~/nav2/`, `~/slam/` | tree | `ubuntu` |

The two `/usr/local/bin` entries are **root-owned on purpose**: they are
executed, not imported, and nothing running as `ubuntu` should be able to
rewrite the process that owns the drive link.

`ubuntu`'s primary group is **asked for** (`pwd.getpwnam(...).pw_gid`), never
assumed to be named after the user — the same question stages 00 and 30 ask, so
three places cannot disagree about the group of one home directory.

**The rover agent is picked by line count**, loudly, exactly as stage 30 picks
it: the repo carries both `fpms-rover-agent.py` and `fpms_rover_agent.py`, they
have drifted, and the longer one wins. The tool prints both candidates and
which it chose. Silently picking is how they drifted in the first place.

### Trees are merged, never mirrored

`~/nav2/`, `~/slam/` and `~/fpms-dashboard/` are **merged over**. Files present
on the rover but absent from the new payload are **kept**, and listed.

This is not tidiness; it is the difference between an update and a disaster:

* `~/nav2/arena_map.pgm` is **generated at image build time** by
  `make_arena_map.py` and is in **no repository**.
* `~/slam/maps/fpms_room.posegraph` requires a physical mapping run in the
  actual room with measured laser offsets. It cannot be rebuilt from source and
  it cannot be baked into an image.

A tree sync that mirrored deletions would destroy both, from a source tree that
looks completely correct. Rollback still handles the other direction properly:
it removes files the update *added*.

### Which units get restarted, and which never do

Only the units whose files actually changed, in this order:

```
fpms-cored -> fpms-rover-agent -> fpms-lidar-ros -> fpms-odom-tf ->
fpms-tf -> fpms-teleop -> fpms-missions -> fpms-telemetry-ros -> fpms-dashboard
```

`fpms-cored` goes **first**. There must be no window in which something that
can move is accepting commands while the stop authority is not listening.

**`micro-ros-agent` is never restarted by this tool, under any flag.** The
serial link costs 90–225 s to re-establish and takes the rover's only pose
source down with it.

**`fpms-uros-supervisor` is held back** even when its own file changed. The new
file is on disk and takes effect at the next restart or reboot; the tool says
so, by name. Pass `--restart-uros` if you want it now, and expect to wait.

---

## How "is the rover mid-mission?" is answered

Every mutating subcommand goes through the same gate. It uses **two independent
sources and trusts neither alone.**

**(A) MQTT `fpms/<thing>/telemetry/mission`.** The executor's own view, and the
one the rest of the stack already uses: `fpms_teleop.py` decides whether it owns
`/cmd_vel` purely by whether it has seen one of these within
`MISSION_OWNS_WIRE_S` (12 s), and `fpms-doctor` reads `.armed` out of it. The
gate uses the executor's own definition of busy, verbatim from
`fpms_missions.py::_telemetry_tick`:

```python
active = st.phase not in ("idle",)
```

`armed: true` blocks as well — an armed rover is one an operator has
deliberately given permission to move, and the arm latch expiring is not
something this tool may race.

That payload is published **on a timer**: 2 Hz (`TELEM_HZ`) during a mission and
one heartbeat every `FPMS_MISSION_IDLE_TELEM_S` while idle. So a live executor
produces a message either way within a couple of seconds — and **not hearing
one means the executor or the broker is down, not that the rover is idle.**

**(B) A passive ROS probe on `/cmd_vel`, `/odom` and `/odom_raw`,** for four
seconds, carrying `ROS_DOMAIN_ID=20`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
`ROS_LOCALHOST_ONLY=1` and
`FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fpms/fastdds_udp_only.xml`. Without that
profile Fast DDS prefers shared memory, whose segments do not survive across
process eras — and this tool is *by definition* a new era, starting long after
every publisher. Discovery would succeed, no data would flow, and the probe
would report a perfectly still rover on a rover that is driving. That is the
exact failure this gate exists to prevent, so the profile is not a detail.

The probe **subscribes only.** It publishes nothing, it never turn-tests for
liveness (that destroys heading), and it opens no tty.

A non-zero `Twist` on `/cmd_vel` is direct evidence and blocks — and it catches
what (A) cannot: teleop's jog/nudge, a stale browser tab, anything driving the
rover that is not the mission executor. Odometry is **corroboration only, and
the asymmetry is deliberate**: this rover has reported clean travel while
spinning in place, so a pose that *changes* proves something moved, while a pose
that does not change proves nothing at all. Absence of odom motion never
authorises an update; presence of it always blocks.

### The policy, including the inconclusive case

| A (`telemetry/mission`) | B (`/cmd_vel` + odom) | result |
|---|---|---|
| either says **MOVING** | | **refuse** |
| IDLE | IDLE | proceed |
| IDLE | UNKNOWN (or the reverse) | proceed, printing which evidence was missing |
| UNKNOWN | UNKNOWN | **refuse** |

The last row is the one worth arguing for. Two dead channels mean we have **no
evidence of stillness at all**, and in this stack silence is never success — a
tool that reads "I heard nothing" as "nothing is happening" is the exact failure
mode that has cost this project whole sessions. Fix a channel (`fpms-selftest`
will say which), or override.

The override is spelled `--i-am-standing-next-to-the-rover`. It is not a flag
anybody types by accident or pastes out of a runbook without reading it, and it
prints in full the evidence it is overriding.

Gated by the same mechanism, and easy to miss: **`config.env` provenance.** The
file is `0640 root:root`; a tool reading it as `ubuntu` gets an empty config, an
empty `FPMS_MQTT_PASS`, an unauthenticated subscribe that returns nothing — and
concludes the rover is idle. `fpms-update` uses `fpms-selftest`'s
file-plus-environment loader, falls back to `/var/lib/fpms/broker-password` when
running as root, reports which source each value came from, and treats "no
password available" as **UNKNOWN**, never as idle.

---

## Automatic payload updates: there is no `fpms-update.timer`

**There is no `fpms-update.timer` and no `fpms-update.service` in this image.
That is a decision, not an omission.** It is the same decision *"Automatic
updates"* above reaches for apt, for overlapping but not identical reasons.

1. **A competition robot that changes its own code unattended is a robot whose
   behaviour you cannot reason about.** The single most valuable property during
   a run is that the code on the rover is the code you last tested. A timer
   trades that away for convenience nobody needs — the payload changes because a
   human decided it should.

2. **The gate cannot be automated.** Every safety property above rests on an
   operator who knows whether the rover is about to be driven. A timer would have
   to either refuse constantly (useless) or act on the gate's own evidence — and
   the gate's honest answer at a venue is frequently UNKNOWN, because the broker
   or DDS is exactly what is being debugged at the time.

3. **The network a timer would need is the one that is not there.** The arena may
   have no internet. A unit that can only ever fail fills the journal and makes
   `systemctl --failed` useless as a signal, which this project already has on
   record for `fpms-console`.

4. **The OS half already made this call, and a payload timer would undo it.**
   `70fpms-update-policy` sets `APT::Periodic::Enable "0"`,
   `Update-Package-Lists "0"`, `Download-Upgradeable-Packages "0"` and
   `Unattended-Upgrade "0"`. Nothing on this rover changes itself. An FPMS
   payload timer would be the only unattended code-changer on the box — and it
   would be changing the *fastest-moving and least-reviewed* code on it.

5. **The cost of not having one is zero.** Updating is one command from a USB
   stick and takes seconds. There is no fleet here; there is one rover, in front
   of you.

A **disabled-by-default** timer was considered and rejected too. Shipping one
would mean shipping a mechanism whose only effect, when enabled, is the thing
this section argues against — one `systemctl enable` away, with none of the
`mask`-strength guarantee that protects the second `/cmd_vel` writers. The
repo's precedent for shipping-and-disabling (`fpms-ros-settle`, the Nav2/SLAM
units) covers things you *need* under a known condition. An automatic updater is
never needed.

Consequently `scripts/60-enable-units.sh` is **unmodified** by the payload
layer: there is no unit to enable, none to mask, and none to assert as
deliberately-not-enabled.

If a future maintainer disagrees, the shape is specified here so it does not
have to be invented: a `.timer` with `Persistent=false`, no `.wants` symlink
created by `scripts/60-enable-units.sh`, and a `.service` running
`fpms-update check --from ...` — **`check`, never `payload`** — so the worst it
can do is tell you something is available. Argue it in this section before
adding it.

---

## Reference: `fpms-update` subcommands and exit codes

```
fpms-update status                     what is live; rollback generations; last run
fpms-update verify                     byte-compile the LIVE payload; report drift
fpms-update check   --from SRC         rehearse an update; change nothing
fpms-update payload --from SRC         stage, verify, snapshot, swap, restart, self-test
fpms-update rollback [--to GEN]        restore a generation
fpms-update prune --keep N             remove old generations (never automatic)
fpms-update os [--check|--apply]       apt, under the holds and pins documented above
```

`SRC` is a directory, a `.tar.gz`/`.tgz`/`.tar`, the word `usb`, or
`git:<url-or-path>` (with `--ref`).

Common flags on the mutating subcommands:

| flag | effect |
|---|---|
| `--i-am-standing-next-to-the-rover` | override a mission-gate refusal; prints the evidence it is overriding |
| `--restart-uros` | also restart `fpms-uros-supervisor` (90–225 s re-link) |
| `--settle N` | seconds to wait before `fpms-selftest` (default 20; its DDS cross-era probe needs the restarted publishers up, and asking too early reports impatience as a failure) |

**Exit codes**

| code | meaning |
|---|---|
| 0 | success |
| 1 | **refused** — nothing was changed |
| 2 | **failed** — the payload was either never touched or was rolled back automatically; read the message |

`root` is required for `payload`, `rollback`, `prune` and `os`. `status`,
`verify` and `check` run as `ubuntu`, and say so when `config.env` was not
readable.

**State on the rover**

```
/var/lib/fpms/updates/gen-<UTC timestamp>/   one generation
                       before/               the exact prior bytes
                       manifest.json         dst, mode, uid, gid, sha before/after
                       meta.json             when, from where, by whom, mission evidence
                       COMPLETE              written LAST; without it the generation is
                                             never offered as a rollback target
/var/lib/fpms/updates/current                the newest generation's name
/var/lib/fpms/update.json                    the last run's result
```

Every run also announces itself on `fpms/<thing>/events/update` — best effort,
and the one swallowed exception in the whole tool, because an update that worked
must not be reported as failed just because the broker was down.

### What "atomic" does and does not mean here

Each individual file replacement **is** atomic: a reader sees either the whole
old file or the whole new one, never a truncated mixture. That is the property
that matters — a half-written `fpms_missions.py` on disk is what turns an update
into an unbootable rover.

A multi-file payload **cannot** be made atomic as a set on a POSIX filesystem
without an indirection this tool is not allowed to introduce: the units
`ExecStart` absolute paths like `/home/ubuntu/fpms_missions.py`, so a
generation-symlink root would mean editing unit files, which belong to another
owner. Claiming set-atomicity would be a lie, so it is not claimed.

What you get instead:

* every byte is copied, `chmod`'d, `chown`'d and `fsync`ed **before** any
  destination is touched, so the slow and failure-prone part happens outside the
  window entirely;
* the window is then a loop of `rename(2)` calls, milliseconds long, in which
  every file has already been byte-compiled;
* a rename that fails part-way is undone immediately from the snapshot, and the
  tool exits 2 saying so;
* and the whole transaction is reversible afterwards from the snapshot, which is
  the guarantee that actually matters at a competition.

<!-- END payload sections -->
