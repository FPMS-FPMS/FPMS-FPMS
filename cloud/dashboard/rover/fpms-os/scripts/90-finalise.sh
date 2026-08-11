#!/usr/bin/env bash
# Stage 90 - record what was built, then clean up for compression.
#
# THIS IS THE LAST STAGE BEFORE THE xz PASS. Everything below runs after ~14
# hours of emulated build time, so a bug here does not cost a stage, it costs
# the night. Three rules for anyone editing this file:
#
#  1. NO EARLY-EXITING CONSUMER INSIDE A PIPELINE WHOSE STATUS IS TESTED.
#
#         v="$(cmd | awk '/x/{print $2; exit}')"
#
#     awk exits after the first match, cmd is still writing, cmd dies of
#     SIGPIPE -> 141. `pipefail` is on, so 141 becomes the status of a plain
#     assignment and `set -e` kills the stage. This has silently killed this
#     build twice. `grep -q`, `grep -m N`, `head -N`, `sed …q` and
#     `awk …{exit}` are all the same bug.
#
#     The three safe shapes, all of which appear below: capture the WHOLE
#     output and `case` on it; feed it from a herestring; or put `|| true`
#     INSIDE the command substitution. And note that `local v="$(…)"` reports
#     `local`'s status (always 0), so it hides the SIGPIPE - and every other
#     real failure with it. Where a status is wanted, `local` is on its own
#     line. There is not one pipeline left in this file.
#
#  2. CLEANING UP IS BEST-EFFORT; PER-BOARD IDENTITY IS NOT. Nothing in the
#     "clean" section may fail the stage - `apt-get clean` returning non-zero
#     is not a reason to throw away an overnight build. The identity section
#     is the opposite: if machine-id is not what it must be, the whole fleet
#     shares one and the stage MUST fail, because `--from 90` costs minutes.
#
#  3. IT MUST BE RE-RUNNABLE. `sudo ./build.sh --from 90` re-does every step
#     here against a resumed image. Each block below says how it re-runs.
set -euo pipefail
echo "--- 90-finalise"

# /etc/fpms must exist NOW. The overlay that creates it is applied in stage 50,
# and on a full build stage 00 made it already - but `build.sh --stage 90` runs
# this stage ALONE against a freshly copied base image, where neither has run.
# Same reasoning as scripts/25-npu-runtime.sh line 66, and the same one-liner.
install -d -m 0755 /etc/fpms

# build.sh's in_chroot() runs `env -i` with an explicit list (CHROOT_ENV, and
# every variable expanded below is on it), so on the normal path these are all
# in the environment already. This block is for the OTHER path: `build.sh
# --shell`, or this file run by hand, where any of them may be missing.
#
# THE PREVIOUS VERSION OF THIS COMMENT HAD IT BACKWARDS, and it was a `set -u`
# landmine. `export X` on an unset X does not DEFINE X - it only marks the name
# for export. The export line itself is not an expansion, so `set -u` stays
# quiet there, which is what made it look safe:
#
#     $ bash -c 'set -u; export FOO; echo "${FOO}"'
#     bash: line 1: FOO: unbound variable        <- measured, not assumed
#
# so the death lands on the first `${…}` instead: the motd heredoc, ~230 lines
# down. That is the worst possible place in this file to stop - the manifest
# has been written, and NOT ONE piece of per-board identity has been cleared
# yet, so what is left on disk is an image that looks finished and ships a
# shared machine-id. Define first, export second.
#
# Empty is the honest default for the three recorded values (the manifest
# records null, os-release records an empty version) but it must not be a
# SILENT default - the whole point of this stage is that nobody wrote down what
# the old Pi was running.
FPMS_OS_VERSION="${FPMS_OS_VERSION:-}"
ROS_DISTRO="${ROS_DISTRO:-}"
RKNN_TOOLKIT2_TAG="${RKNN_TOOLKIT2_TAG:-}"
export FPMS_OS_VERSION ROS_DISTRO RKNN_TOOLKIT2_TAG
[ -n "$FPMS_OS_VERSION" ] \
    || echo "    NOTE: FPMS_OS_VERSION is empty - the manifest and /etc/os-release will say so." >&2

# These two are only expanded to print a URL in the motd and to delete a shell
# history. Neither is worth stopping the stage over, so both get a default
# rather than an assertion - but they need one, for the same reason as above.
FPMS_HOSTNAME="${FPMS_HOSTNAME:-fpms}"
FPMS_HOME="${FPMS_HOME:-/root}"

# --- the version manifest ---------------------------------------------------
#
# Nobody ever wrote down what the old Pi was running. bench_npu.py reads the
# NPU driver version at RUNTIME from /sys/kernel/debug/rknpu/version, which is
# the tell: there was no build record, so there was nothing to compare a
# working rover against a broken one.
#
# THE WRITE IS OWNED BY PYTHON, NOT BY A REDIRECT. The obvious shape,
#
#     python3 - <<'PY' > /etc/fpms/versions.json
#
# truncates the destination before python has run a line. A python that then
# dies leaves a ZERO-BYTE manifest that looks like a manifest - and this stage
# is the last thing anyone would think to re-run. Redirecting to a temp file
# and `mv`-ing fixes the truncation but still leaves the temp file behind on
# failure. So: python builds the whole document in memory, parses back what it
# is about to write, writes it to a temp file, fsyncs, and os.replace()s it
# over the destination. os.replace is atomic within a filesystem, so the
# manifest is either the old one or the complete new one, never a torn one -
# which matters because the very next thing that happens to this image is a
# multi-hour xz pass that must not be handed a half-written file.
#
# Python's own failure is detected by `set -e`: this is a simple command, not
# an assignment and not a condition, so a non-zero exit stops the stage. That
# includes the case where python exits 0 on the last line but fails to flush -
# CPython exits 120 when the final stdout flush fails.
python3 - <<'PY'
# Written under `env -i ... LC_ALL=C LANG=C.UTF-8` (build.sh CHROOT_ENV) and
# under qemu-user-static. Both facts constrain this file:
#
#   ENCODING. CPython skips PEP 538 locale coercion when LC_ALL is set, so the
#   preferred encoding here is ANSI_X3.4-1968 (ASCII), not UTF-8. `text=True`
#   would therefore decode every subprocess pipe as STRICT ASCII, and a single
#   non-ASCII byte in a package version or a module's __version__ would raise
#   UnicodeDecodeError - which the helpers below would catch and quietly record
#   as null. Every subprocess names encoding="utf-8", errors="replace"
#   explicitly instead. The output side is safe by construction: json.dumps
#   defaults to ensure_ascii=True, so the blob written is pure ASCII whatever
#   goes into it.
#
#   EXEC COST. Every process spawned here is an emulator start-up. Nothing
#   spawns a process for something python can answer itself (hashlib, not
#   sha256sum; time.strftime, not date(1) - the same trade
#   scripts/25-npu-runtime.sh made), the seven dpkg-query calls are one call,
#   and everything that IS spawned has a timeout. A dpkg-query that wedges at
#   this point would hang the last stage of an overnight build forever, with
#   no output and nothing to look at.
import hashlib, json, os, subprocess, time

MANIFEST = "/etc/fpms/versions.json"
TMP      = "/etc/fpms/.versions.json.new"


def run(cmd, timeout=180):
    """A subprocess that can never raise and can never hang."""
    try:
        return subprocess.run(cmd, capture_output=True,
                              encoding="utf-8", errors="replace",
                              timeout=timeout)
    except Exception:          # OSError, TimeoutExpired, anything
        return None


def dpkg_versions(names):
    """One dpkg-query for the whole list, not one per package.

    dpkg-query exits 1 when ANY pattern matched nothing and still prints the
    ones that matched, so the returncode is deliberately not consulted - the
    stdout lines are the measurement. A package purged to config-files still
    reports a Version here; ${db:Status-Status} would distinguish that, but it
    is a newer format field and a dpkg that does not know it fails the WHOLE
    query, which would silently null out this entire section. Not a trade
    worth making in the last stage of the build.
    """
    found = dict.fromkeys(names)
    r = run(["dpkg-query", "-W", "-f=${Package} ${Version}\n"] + list(names))
    if r is not None:
        for line in r.stdout.splitlines():
            parts = line.split()
            if len(parts) == 2 and parts[0] in found:
                found[parts[0]] = parts[1]
    return found


def pymod(mod):
    """Tri-state, for the same reason stage 25's flag() is tri-state: "we did
    not look" and "we looked and it is not there" are different statements and
    a build record must not blur them. A module that imports but carries no
    __version__ (paho.mqtt, depending on release) is importable with a null
    version - which is still a measurement, and a more honest one than the
    empty string the old code recorded."""
    # 300s, not run()'s 180. This is the only thing here that executes a large
    # amount of foreign code under qemu-user: importing cv2 dlopen's a long
    # chain of .so files and every instruction in every one of them is
    # emulated, so an import that is 0.3s on the board can be minutes here. A
    # timeout does not fail the stage - it records {"importable": null}, "we
    # did not look" - which for a module that IS installed is precisely the
    # blurred measurement pymod() exists to avoid. A missing module still
    # fails fast (ImportError), so the higher ceiling costs nothing normally.
    r = run(["python3", "-c",
             "import %s as _m; print(getattr(_m, '__version__', ''))" % mod],
            timeout=300)
    if r is None:
        return {"importable": None, "version": None}
    if r.returncode != 0:
        return {"importable": False, "version": None}
    return {"importable": True, "version": r.stdout.strip() or None}


def sha256(path):
    """In-process. The old version shelled out to sha256sum and took
    .stdout.split()[0], which is an IndexError on a missing file - caught by a
    bare `except Exception` and recorded as null, i.e. indistinguishable from
    "the file is not there"."""
    try:
        h = hashlib.sha256()
        with open(path, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def uname_release():
    """Guarded, because "this cannot raise" has to be true of every line that
    builds `doc`, not just the ones that touch the filesystem. os.uname() is
    Unix-only (it does not exist at all on some platforms, which is also what
    makes this heredoc impossible to dry-run anywhere but Linux), and an
    AttributeError here would abort the whole manifest for a field that is
    explicitly labelled as the least trustworthy one in it. Under qemu-user
    the call succeeds and returns the BUILD HOST's release - see the note."""
    try:
        return os.uname().release
    except Exception:
        return None


def modules_in_image():
    try:
        return sorted(d for d in os.listdir("/lib/modules")
                      if os.path.isdir(os.path.join("/lib/modules", d)))
    except OSError:
        return []


doc = {
    "schema": 1,
    "generated_by": "fpms-os scripts/90-finalise.sh",
    "fpms_os_version": os.environ.get("FPMS_OS_VERSION"),
    "built": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "ros_distro": os.environ.get("ROS_DISTRO"),

    # THE OLD "kernel" FIELD WAS A LIE, and exactly the lie stage 25 refuses to
    # tell about the NPU driver. It recorded `uname -r`, but this stage runs in
    # a chroot on the build host under qemu-user-static: uname reports the
    # HOST's kernel release - the x86 workstation, or WSL2's kernel - never the
    # kernel this image will boot. Anyone comparing a working rover against a
    # broken one using that field would have been comparing two build hosts.
    #
    # What CAN be measured from in here is which kernel the image carries
    # modules for, which is the number that actually has to match the rknpu
    # driver. The host's uname is kept, clearly labelled as the build host's,
    # because it is still useful in a post-mortem - just not as "the kernel".
    "kernel": {
        "modules_in_image": modules_in_image(),
        "uname_at_build": uname_release(),
        "note": ("uname_at_build is the BUILD HOST's kernel (chroot +"
                 " qemu-user-static), not this image's. The rover's running"
                 " kernel is `uname -r` on the board; it should match one of"
                 " modules_in_image."),
    },

    "apt": dpkg_versions((
        "ros-humble-ros-base", "ros-humble-navigation2",
        "ros-humble-slam-toolbox", "ros-humble-rosbridge-suite",
        "ros-humble-rmw-fastrtps-cpp", "python3-opencv", "mosquitto")),

    "python": {m: pymod(m) for m in ("numpy", "cv2", "serial", "paho.mqtt")},

    "npu": {
        "librknnrt_sha256": sha256("/usr/lib/librknnrt.so"),
        "rknn_toolkit2_tag": os.environ.get("RKNN_TOOLKIT2_TAG"),
        "note": "librknnrt.so must match the kernel rknpu driver. Compare "
                "against /sys/kernel/debug/rknpu/version on the running board.",
        "see": "/etc/fpms/npu-versions.json  (stage 25, the fuller record)",
    },
}

blob = json.dumps(doc, indent=2) + "\n"
json.loads(blob)                       # parse what is about to be written,
                                       # not what we hope was written
try:
    with open(TMP, "w", encoding="utf-8") as fh:
        fh.write(blob)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(TMP, MANIFEST)          # atomic; also removes TMP
except BaseException:
    # The comment above promises the temp file is not left behind on failure.
    # It was not true: a write that dies part-way - ENOSPC is the realistic
    # one, since nothing has reclaimed a block of this rootfs yet at this
    # point in the stage - leaves `.versions.json.new` sitting in /etc/fpms.
    # That is a truncated manifest with a different name, next to the real
    # one, shipped in the image and never looked at again. Remove it and
    # re-raise: the failure itself still has to stop the stage.
    try:
        os.unlink(TMP)
    except OSError:
        pass
    raise
print("    %s  (%d bytes)" % (MANIFEST, len(blob)))
PY
chmod 0644 /etc/fpms/versions.json
chown root:root /etc/fpms/versions.json

# Belt and braces, and deliberately NOT `grep -q '{' file`: a `grep -q` in a
# tested position is rule 1 at the top of this file. Capture the whole thing
# and pattern-match it - no pipeline, no early exit, nothing to take a SIGPIPE.
# `$(cat …)` strips trailing newlines, so a complete document ends in `}`.
manifest="$(cat /etc/fpms/versions.json 2>/dev/null || true)"
case "$manifest" in
    '{'*'}') : ;;
    *) echo "FATAL: /etc/fpms/versions.json is not a complete JSON document." >&2
       echo "       The build record is the whole point of this stage. Fix and" >&2
       echo "       re-run:  sudo ./build.sh --from 90" >&2
       exit 1 ;;
esac
unset manifest

# --- os-release -------------------------------------------------------------
#
# Strip any previous FPMS block before appending rather than skipping the whole
# thing when one is present. A `grep -q FPMS` guard makes re-running this stage
# after a version bump silently leave the OLD version in /etc/os-release (and
# is rule 1 besides).
#
# Resolve the symlink first. On Ubuntu /etc/os-release is a symlink to
# ../usr/lib/os-release, and `sed -i` does NOT follow symlinks: it would
# REPLACE the link with a regular file, after which /etc/os-release and
# /usr/lib/os-release disagree - the appended VARIANT lines exist in one and
# not the other, and anything reading the /usr/lib copy (systemd's own
# fallback) never sees them. Editing the resolved path keeps both views
# identical because one is the other.
OSR=/etc/os-release
[ -e "$OSR" ] || : > "$OSR"
OSR_REAL="$(readlink -f "$OSR" 2>/dev/null || true)"
[ -n "$OSR_REAL" ] || OSR_REAL="$OSR"
sed -i '/^VARIANT="FPMS-OS"$/d; /^VARIANT_ID="fpms-os"$/d; /^FPMS_OS_VERSION=/d' \
    "$OSR_REAL"

# `cat >>` onto a file that does not end in a newline SPLICES, it does not
# append: the result is
#     PRETTY_NAME="Ubuntu 22.04.5 LTS"VARIANT="FPMS-OS"
# - one corrupt line, no VARIANT, and a PRETTY_NAME that every tool reading
# os-release now gets wrong. GNU sed -i faithfully preserves a missing final
# newline, so the sed above does not fix it and can even create it by deleting
# what used to be the last line. This is base-image-dependent, which is the
# worst kind of bug to find at the last stage: it works on the image you tested
# and splices on the one you shipped.
#
# `$(…)` strips trailing newlines, so `tail -c 1` comes back EMPTY exactly when
# the file already ends in a newline (or is empty, where there is nothing to
# separate). One command, no pipeline, nothing to take a SIGPIPE.
osr_tail="$(tail -c 1 "$OSR_REAL" 2>/dev/null || true)"
[ -z "$osr_tail" ] || printf '\n' >> "$OSR_REAL"
unset osr_tail

cat >> "$OSR_REAL" <<EOF
VARIANT="FPMS-OS"
VARIANT_ID="fpms-os"
FPMS_OS_VERSION="${FPMS_OS_VERSION}"
EOF

cat > /etc/motd <<EOF

  FPMS-OS ${FPMS_OS_VERSION}   -   Fire Prevention & Management System rover

    fpms-selftest            is this rover actually working?
    fpms-doctor <symptom>    guided troubleshooting
    systemctl list-units 'fpms-*'
    journalctl -u fpms-missions -f

    console:  http://${FPMS_HOSTNAME}.local:8090/
    docs:     /opt/fpms-os/docs/
    built:    /etc/fpms/versions.json

  Before trusting a mission, run:  python3 ~/fpms_charact.py --push-check
  Pushed forward, x must INCREASE. Everything downstream is meaningless if
  the sign is wrong.

EOF

# --- per-board identity: everything that must NOT be shared across a fleet ---
#
# machine-id MUST be cleared, and cleared to an EMPTY FILE, not deleted. A
# machine-id baked into an image means every flashed board has the same one,
# which breaks DHCP leases (identical client identifiers), systemd journal
# identity, and anything else keyed on it.
#
# Empty rather than absent is deliberate and is what systemd documents: an
# empty /etc/machine-id is the marker that makes systemd generate one on first
# boot AND is what ConditionFirstBoot=yes tests. A missing file also gets
# regenerated, but on a read-only or not-yet-writable /etc systemd falls back
# to a transient id in /run and ConditionFirstBoot never fires.
#
# 0444 is not a typo and is not "make it read-only for safety": it is the mode
# systemd-machine-id-setup itself creates the file with, and root writes it on
# first boot regardless of the mode bits. Shipping it 0644 would be the odd
# one out, not the safe one.
#
# Re-run safe: `rm -f` on a 0444 file succeeds - the permission that matters is
# on /etc, not on the file - so `--from 90` clears an already-cleared id.
rm -f /etc/machine-id            # in case it is a symlink into /var
: > /etc/machine-id
chmod 0444 /etc/machine-id

# The dbus link must be made AFTER the file exists, and /var/lib/dbus may not
# exist at all if dbus was pulled in only as a dependency -- `ln` into a
# missing directory fails, and with set -e that kills the last stage of the
# build over a symlink nobody would miss.
install -d -m 0755 /var/lib/dbus
rm -f /var/lib/dbus/machine-id
ln -sf /etc/machine-id /var/lib/dbus/machine-id

# Same class of bug, same fix.
#
#   random-seed: a seed shipped in an image is the SAME seed on every board.
#   credential.secret: systemd's per-machine key for encrypted credentials;
#     baked in, it is one key for the fleet. systemd regenerates it on demand.
#   ssh host keys: removed in stage 00 too - re-asserted here because anything
#     installed since could have put them back. VERIFIED they are regenerated
#     on the board and this does not lock anyone out: overlay's fpms-firstboot
#     line 95 runs `ssh-keygen -A` when no host key exists, and scripts/00
#     also installs an `ExecStartPre=-/usr/bin/ssh-keygen -A` on sshd.
#
# Every one of these is `rm -f`, which succeeds on an absent file and on an
# unmatched glob (bash passes the literal pattern through and -f swallows it),
# so none of them can fail the stage or need a `|| true`.
rm -f /var/lib/systemd/random-seed /var/lib/urandom/random-seed
rm -f /var/lib/systemd/credential.secret
rm -f /etc/ssh/ssh_host_*

# NetworkManager keeps its OWN per-machine secret, and this is a NetworkManager
# image - overlay/usr/local/sbin/fpms-wifi-provision drives the radio with
# nmcli, so NM is what brings up the interface every rover is reached on.
#
# /var/lib/NetworkManager/secret_key is the seed NM derives two things from:
# ipv6.addr-gen-mode=stable-privacy addresses, and the stable DHCP DUID /
# client identifier. Baked into the image it is the SAME seed on every board,
# so two rovers on one network derive the SAME IPv6 address (duplicate address
# detection then takes one of them off the network) and present the SAME DHCP
# client id (the second lease overwrites the first). That is exactly the
# fight-over-a-lease failure the machine-id block above exists to prevent,
# arriving by a route machine-id does not cover - NM does not derive this key
# from machine-id, it generates and stores its own. It writes a fresh one the
# first time it starts without it.
#
# The rest is staleness rather than identity - leases from the build host's
# network, a scan cache of the wrong building's access points - but it is the
# same `rm` and none of it should ship either.
#
# NOTE THE ABSENCE OF A BARE `*` GLOB over any of these directories: `rm -f` on
# a DIRECTORY fails with "Is a directory" and exits 1, which `-f` does NOT
# swallow, which under `set -e` kills the stage. Every pattern here can only
# match regular files. (`/var/lib/dhcpcd/*` would have been the trap: dhcpcd
# keeps subdirectories there on some releases.)
rm -f /var/lib/NetworkManager/secret_key
rm -f /var/lib/NetworkManager/seen-bssids /var/lib/NetworkManager/timestamps
rm -f /var/lib/NetworkManager/*.lease
rm -f /var/lib/dhcp/*.leases /var/lib/dhcp/*.leases~
rm -f /var/lib/dhcpcd/duid /var/lib/dhcpcd/secret

# Now prove it, because "the whole fleet shares one machine-id" is discovered
# by a person watching two rovers fight over a DHCP lease, months later. These
# are the only checks in this file allowed to fail the stage: they mean the
# image is wrong, and `--from 90` costs minutes against the 14 hours already
# spent.
identity_ok=1
[ -f /etc/machine-id ] || { echo "FATAL: /etc/machine-id is not a regular file (systemd needs it present and EMPTY - absent is not the same marker)" >&2; identity_ok=0; }
[ -s /etc/machine-id ] && { echo "FATAL: /etc/machine-id is not empty - every board flashed from this image would share one id" >&2; identity_ok=0; }
[ -L /var/lib/dbus/machine-id ] || { echo "FATAL: /var/lib/dbus/machine-id is not a symlink to /etc/machine-id" >&2; identity_ok=0; }
[ -e /var/lib/NetworkManager/secret_key ] && { echo "FATAL: /var/lib/NetworkManager/secret_key survived - every board would derive the same stable-privacy IPv6 address and the same DHCP client id" >&2; identity_ok=0; }
# The host keys were rm'd above but never checked, and this is the one piece of
# identity whose failure is silent for months: two rovers with the same host
# key do not break, they just make every operator laptop's known_hosts wrong,
# and the usual response to that is to delete the known_hosts entry rather than
# to wonder why. scripts/00-base-system.sh line 514 asserts the same thing with
# the same idiom, and the idiom matters: `ls` inside an `if` is a CONDITION,
# not a pipeline, so rule 1 does not apply and there is nothing to SIGPIPE.
# (A bare glob would not do: unmatched, bash hands through the literal pattern,
# and `[ -e '/etc/ssh/ssh_host_*' ]` is a much less obvious way to be right.)
if ls /etc/ssh/ssh_host_* >/dev/null 2>&1; then
    echo "FATAL: SSH host keys survived the rm - every board flashed from this image would present the same host key" >&2
    identity_ok=0
fi
# `[ ] && …` and `[ ] || …` are safe here ONLY because each is followed by
# another statement; as the LAST statement of a script or function a false
# test makes the whole list non-zero and `set -e` exits. Hence the explicit
# `if` below rather than a trailing `[ "$identity_ok" = 1 ]`.
if [ "$identity_ok" != 1 ]; then
    echo "       Per-board identity is wrong. Fix and re-run:" >&2
    echo "           sudo ./build.sh --from 90" >&2
    exit 1
fi
echo "    per-board identity cleared (machine-id empty, seeds and host keys gone)"

# --- first-boot marker ------------------------------------------------------
# Make sure firstboot actually runs on the flashed image. Both paths verified
# against the unit and the script that own them: fpms-firstboot.service line 26
# gates on `ConditionPathExists=!/var/lib/fpms/.firstboot-done`, and
# usr/local/sbin/fpms-firstboot lines 16-17 define DONE_MARK and REPORT as
# exactly these two.
rm -f /var/lib/fpms/.firstboot-done /var/lib/fpms/firstboot.json

# The MQTT broker password is the one shared secret this stage deliberately
# does NOT touch. fpms-firstboot generates it per board, but it is gated on the
# placeholder still being in config.env (provision_broker_password, keyed on
# `grep -q '__FPMS_FIRSTBOOT_WILL_REPLACE_THIS__'`), so a config.env whose
# FPMS_MQTT_PASS has already been filled in means firstboot will NOT
# regenerate: one broker password for the whole fleet. Deleting it here would
# only make it unrecoverable while leaving the fleet-wide value in config.env,
# so this reports and does not act - the fix belongs in the overlay.
#
# Absent config.env is NOT a finding: `build.sh --stage 90` runs alone against
# a base image where stage 50 has not installed the overlay. Hence the -f test.
# Capture-and-`case` rather than `grep -q`, per rule 1.
if [ -f /etc/fpms/config.env ]; then
    cfg="$(cat /etc/fpms/config.env 2>/dev/null || true)"
    case "$cfg" in
        *__FPMS_FIRSTBOOT_WILL_REPLACE_THIS__*) : ;;
        *) echo "    NOTE: /etc/fpms/config.env has no FPMS_MQTT_PASS placeholder left." >&2
           echo "    fpms-firstboot only provisions a per-board broker password when that" >&2
           echo "    placeholder is present, so every board from this image would share" >&2
           echo "    one. Check FPMS_MQTT_PASS= in overlay/etc/fpms/config.env." >&2 ;;
    esac
    unset cfg
fi

# --- clean ------------------------------------------------------------------
#
# Rule 2 territory: not one line here may fail the stage. The `|| true`s below
# are not decoration and are not hiding anything that matters - the worst case
# for every one of them is an image that is slightly larger than it could be.
apt-get clean \
    || echo "    NOTE: apt-get clean failed; the image keeps its .deb cache" >&2
rm -rf /var/lib/apt/lists/* || true

# find, not `rm -rf /tmp/*`: the glob misses dotfiles, and half of what
# accumulates in /tmp during a ROS build is dotfiles. -mindepth 1 keeps the
# directories themselves. (Checked build.sh: the chroot bind-mounts /proc,
# /sys, /dev, /dev/pts and a tmpfs on /run - /tmp is NOT bind-mounted from the
# host, so this deletes the image's /tmp and nothing of the build machine's.)
find /tmp /var/tmp -mindepth 1 -delete 2>/dev/null || true
rm -f /root/.bash_history "${FPMS_HOME}/.bash_history" 2>/dev/null || true

# The journal is NOT a log file you can truncate. A .journal truncated to zero
# is a CORRUPT journal file: journald renames it to *.journal~ on first boot
# and complains, once per board, forever. Delete the files and leave the
# directory - the existence of /var/log/journal is what selects persistent
# journaling over volatile, so removing the directory would silently change
# the rover's logging behaviour.
rm -rf /var/log/journal/* /run/log/journal/* 2>/dev/null || true

# `-exec … +` not `\;`: one truncate for a batch of files instead of one exec
# per file, and every exec here is a qemu-user start-up. find exits non-zero if
# any exec did, or if it could not read a directory (a file vanishing between
# the walk and the exec is enough) - neither is a reason to fail this stage.
find /var/log -type f -exec truncate -s 0 {} + 2>/dev/null || true

# The build tree stays: docs/, selftest/ and SPEC.md are useful ON the rover,
# and fpms-doctor and the unit Documentation= lines point into it. Only the
# staged source copy goes - VERIFIED nothing but scripts/30-fpms-payload.sh
# (SRC=/opt/fpms-os/src, which has long since run) reads it, and build.sh's
# stage_all() re-stages it on every invocation, so `--from 90` finds it there
# again and deletes it again.
rm -rf /opt/fpms-os/src || true

# --- reclaim free space so the image compresses ------------------------------
#
# The old version of this was `dd if=/dev/zero of=/EMPTY`. That is a bad trade
# HERE, and the reason is the host, not the image:
#
#   ROOTFS_GROW_MB is 6144, so after the grow there are several GB of free
#   space inside the rootfs. The image is a sparse file on the build host, and
#   writing zeros into every free block ALLOCATES every one of them. `rm
#   /EMPTY` afterwards frees the blocks inside the filesystem but does NOT
#   give them back to the host -- the .img is now fully allocated, permanently,
#   for the rest of the build and for the whole of the xz pass that follows.
#   A build host with room for the image but not for the image plus 6 GB dies
#   at the very last step of a multi-hour run.
#
# fstrim does the same job from the other end and does it better: it issues
# discards for the free blocks, the loop device turns those into hole-punches
# in the backing file, and the file gets SMALLER. The holes read back as zeros,
# so xz compresses exactly as well as it would have after a zero-fill.
#
# fstrim WORKS IN THIS CHROOT even though `df /` does not, and the difference
# is worth writing down because it is not obvious: fstrim(8) opens the path it
# is given and issues FITRIM on that file descriptor. It only consults the
# mount table for -a/-A, which is not used here. `df` and `stat -f` differ -
# see the note in the zero-fill branch.
#
# Nothing inside the chroot can measure the HOST's free space, so if fstrim is
# unavailable the zero-fill stays opt-in rather than defaulting to the risky
# thing. Set FPMS_ZEROFILL=1 in the build environment to allow it.
is_uint() { case "$1" in ''|*[!0-9]*) return 1 ;; *) return 0 ;; esac; }

reclaim_free_space() {
    if command -v fstrim >/dev/null 2>&1; then
        local trim_out="" trim_rc=0
        # Split across two lines deliberately. `local trim_out="$(fstrim …)"`
        # would make the STATUS of the whole thing `local`'s, which is always
        # 0, so trim_rc could never be anything but 0 and a failed fstrim
        # would be reported as a success. Rule 1's cousin.
        trim_out="$(fstrim -v / 2>&1)" || trim_rc=$?
        if [ "$trim_rc" -eq 0 ]; then
            echo "    fstrim: ${trim_out:-/ trimmed}"
            echo "    (free blocks discarded; the .img is punched sparse again)"
            return 0
        fi
        echo "    NOTE: fstrim / failed (exit ${trim_rc}): ${trim_out}" >&2
    fi

    if [ "${FPMS_ZEROFILL:-0}" != 1 ]; then
        echo "    NOTE: fstrim unavailable or unsupported on this loop device." >&2
        echo "    Skipping the zero-fill: it would allocate every free block of the" >&2
        echo "    image on the build host (ROOTFS_GROW_MB of them) and cannot be" >&2
        echo "    undone before the xz pass. The .img.xz will be larger." >&2
        echo "    Set FPMS_ZEROFILL=1 to allow it on a host with the headroom." >&2
        return 0
    fi

    # Opt-in path, and still bounded. Leave 256 MB so the filesystem is never
    # actually driven to zero free -- ext4 at 0 free is a state that turns
    # unrelated later failures into mysteries.
    #
    # statfs, not `df /`. df resolves the mountpoint through /proc/self/mounts,
    # which in this chroot lists the HOST's paths and not "/". `stat -f` calls
    # statfs(2) on the path itself and needs no mount table.
    local blocks="" bsize="" avail=0 write=0 rc=0
    blocks="$(stat -f -c %a / 2>/dev/null || true)"   # free to non-superuser:
    bsize="$(stat -f -c %S / 2>/dev/null || true)"    # the conservative number
    # A failed stat leaves these EMPTY, and `$(( ${empty} * ${empty} ))` is a
    # bash arithmetic SYNTAX ERROR, not a zero - which under `set -e` would
    # kill the final stage of the build inside its own space check.
    if ! is_uint "$blocks" || ! is_uint "$bsize"; then
        echo "    zero-fill skipped: statfs on / gave nothing to work with" >&2
        return 0
    fi
    avail=$(( blocks * bsize / 1048576 ))
    write=$(( avail - 256 ))
    if [ "$write" -le 0 ]; then
        echo "    zero-fill skipped: only ${avail} MB free"
        return 0
    fi

    echo "    zero-filling ${write} MB of free space (FPMS_ZEROFILL=1)"
    # NO `|| true` ON THIS, and no `2>/dev/null` either. dd here writes into a
    # SPARSE file on the build host, so the ENOSPC it is most likely to hit is
    # the HOST's, not the image's - the one failure this whole section exists
    # to avoid. Swallowing it means an image whose free space is now fully
    # allocated on a disk that just proved it has no room, handed straight to
    # a multi-hour xz that needs the .img and the .xz to coexist. Better to
    # stop here, with dd's own message on screen, than 40 minutes into that.
    dd if=/dev/zero of=/EMPTY bs=1M count="$write" status=none || rc=$?
    rm -f /EMPTY            # unconditional: the blocks go back to the image's
    sync                    # filesystem whether dd finished or not
    if [ "$rc" -ne 0 ]; then
        echo "FATAL: the zero-fill failed (dd exit ${rc}) - see dd's message above." >&2
        echo "       If that is ENOSPC it is the BUILD HOST that is full, and the" >&2
        echo "       xz pass that follows needs the .img and the .xz side by side." >&2
        echo "       Free space, then re-run:  sudo ./build.sh --from 90" >&2
        echo "       Or drop FPMS_ZEROFILL entirely: fstrim is the default path" >&2
        echo "       and the only thing lost is some .img.xz size." >&2
        return 1
    fi
    return 0
}
# A function whose last executed command returns non-zero returns non-zero, and
# as a bare command that is a `set -e` exit. Every path above ends in an
# explicit `return`, so this line means what it looks like it means.
reclaim_free_space
sync

echo "--- 90-finalise OK"
