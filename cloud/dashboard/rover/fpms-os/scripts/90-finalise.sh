#!/usr/bin/env bash
# Stage 90 - record what was built, then clean up for compression.
set -euo pipefail
echo "--- 90-finalise"

install -d -m 0755 /etc/fpms

# --- the version manifest ---------------------------------------------------
#
# Nobody ever wrote down what the old Pi was running. bench_npu.py reads the
# NPU driver version at RUNTIME from /sys/kernel/debug/rknpu/version, which is
# the tell: there was no build record, so there was nothing to compare a
# working rover against a broken one.
#
# Written to a temp file and moved into place. A redirect straight onto the
# final path truncates it before python runs, so a python that dies leaves a
# zero-byte manifest that looks like a manifest -- and this stage is the last
# thing anyone would think to re-run.
python3 - <<'PY' > /etc/fpms/.versions.json.new
import json, os, subprocess

def dpkg(p):
    try:
        return subprocess.run(["dpkg-query", "-W", "-f=${Version}", p],
                              capture_output=True, text=True).stdout.strip() or None
    except Exception:
        return None

def pyver(mod):
    try:
        return subprocess.run(
            ["python3", "-c", "import %s;print(%s.__version__)" % (mod, mod)],
            capture_output=True, text=True).stdout.strip() or None
    except Exception:
        return None

def sha(path):
    try:
        return subprocess.run(["sha256sum", path],
                              capture_output=True, text=True).stdout.split()[0]
    except Exception:
        return None

print(json.dumps({
    "fpms_os_version": os.environ.get("FPMS_OS_VERSION"),
    "built": subprocess.run(["date", "-u", "+%Y-%m-%dT%H:%M:%SZ"],
                            capture_output=True, text=True).stdout.strip(),
    "kernel": subprocess.run(["uname", "-r"], capture_output=True,
                             text=True).stdout.strip(),
    "ros_distro": os.environ.get("ROS_DISTRO"),
    "apt": {p: dpkg(p) for p in (
        "ros-humble-ros-base", "ros-humble-navigation2",
        "ros-humble-slam-toolbox", "ros-humble-rosbridge-suite",
        "ros-humble-rmw-fastrtps-cpp", "python3-opencv", "mosquitto")},
    "python": {m: pyver(m) for m in ("numpy", "cv2", "serial", "paho.mqtt")},
    "npu": {
        "librknnrt_sha256": sha("/usr/lib/librknnrt.so"),
        "rknn_toolkit2_tag": os.environ.get("RKNN_TOOLKIT2_TAG"),
        "note": "librknnrt.so must match the kernel rknpu driver. Compare "
                "against /sys/kernel/debug/rknpu/version on the running board.",
    },
}, indent=2))
PY
mv -f /etc/fpms/.versions.json.new /etc/fpms/versions.json
chmod 0644 /etc/fpms/versions.json
chown root:root /etc/fpms/versions.json
echo "    /etc/fpms/versions.json"

# --- os-release -------------------------------------------------------------
#
# Strip any previous FPMS block before appending rather than skipping the whole
# thing when one is present. A `grep -q FPMS` guard makes re-running this stage
# after a version bump silently leave the OLD version in /etc/os-release.
sed -i '/^VARIANT="FPMS-OS"$/d; /^VARIANT_ID="fpms-os"$/d; /^FPMS_OS_VERSION=/d' /etc/os-release
cat >> /etc/os-release <<EOF
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

# Same class of bug, same fix. A random seed shipped in an image is the same
# seed on every board, and SSH host keys are removed in stage 00 -- re-assert
# both here because anything installed since could have put them back.
rm -f /var/lib/systemd/random-seed /var/lib/urandom/random-seed
rm -f /etc/ssh/ssh_host_*

# --- first-boot marker ------------------------------------------------------
# Make sure firstboot actually runs on the flashed image.
rm -f /var/lib/fpms/.firstboot-done /var/lib/fpms/firstboot.json

# --- clean ------------------------------------------------------------------
apt-get clean
rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
rm -f /root/.bash_history "${FPMS_HOME}/.bash_history" 2>/dev/null || true
find /var/log -type f -exec truncate -s 0 {} \; 2>/dev/null || true

# The build tree stays: docs/, selftest/ and SPEC.md are useful ON the rover,
# and fpms-doctor and the unit Documentation= lines point into it. Only the
# staged source copy goes.
rm -rf /opt/fpms-os/src

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
#   at the very last step of a multi-hour run, with ENOSPC swallowed by the
#   `|| true`, which makes it look like something else entirely.
#
# fstrim does the same job from the other end and does it better: it issues
# discards for the free blocks, the loop device turns those into hole-punches
# in the backing file, and the file gets SMALLER. The holes read back as zeros,
# so xz compresses exactly as well as it would have after a zero-fill.
#
# Nothing inside the chroot can measure the HOST's free space, so if fstrim is
# unavailable the zero-fill stays opt-in rather than defaulting to the risky
# thing. Set FPMS_ZEROFILL=1 in the build environment to allow it.
reclaim_free_space() {
    if command -v fstrim >/dev/null 2>&1 && fstrim / >/dev/null 2>&1; then
        echo "    fstrim / - free blocks discarded, image file punched sparse"
        return 0
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
    # statfs, not `df /`. df resolves the mountpoint through /proc/self/mounts,
    # which in this chroot lists the HOST's paths and not "/".
    local avail write
    avail=$(( $(stat -f -c %a /) * $(stat -f -c %S /) / 1048576 ))
    write=$(( avail - 256 ))
    if [ "$write" -le 0 ]; then
        echo "    zero-fill skipped: only ${avail} MB free"
        return 0
    fi
    echo "    zero-filling ${write} MB of free space (FPMS_ZEROFILL=1)"
    dd if=/dev/zero of=/EMPTY bs=1M count="$write" status=none 2>/dev/null || true
    rm -f /EMPTY
}
reclaim_free_space
sync

echo "--- 90-finalise OK"
