#!/usr/bin/env bash
# Stage 90 - record what was built, then clean up for compression.
set -euo pipefail
echo "--- 90-finalise"

# --- the version manifest ---------------------------------------------------
#
# Nobody ever wrote down what the old Pi was running. bench_npu.py reads the
# NPU driver version at RUNTIME from /sys/kernel/debug/rknpu/version, which is
# the tell: there was no build record, so there was nothing to compare a
# working rover against a broken one.
python3 - <<'PY' > /etc/fpms/versions.json
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
chmod 0644 /etc/fpms/versions.json
echo "    /etc/fpms/versions.json"

# --- os-release -------------------------------------------------------------
if ! grep -q FPMS /etc/os-release; then
    cat >> /etc/os-release <<EOF
VARIANT="FPMS-OS"
VARIANT_ID="fpms-os"
FPMS_OS_VERSION="${FPMS_OS_VERSION}"
EOF
fi

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

# --- machine-id -------------------------------------------------------------
#
# MUST be cleared. A machine-id baked into an image means every flashed board
# has the same one, which breaks DHCP leases (identical client identifiers),
# systemd journal identity, and anything else keyed on it. systemd regenerates
# it on first boot from an empty file.
: > /etc/machine-id
rm -f /var/lib/dbus/machine-id
ln -sf /etc/machine-id /var/lib/dbus/machine-id

# --- first-boot marker ------------------------------------------------------
# Make sure firstboot actually runs on the flashed image.
rm -f /var/lib/fpms/.firstboot-done

# --- clean ------------------------------------------------------------------
apt-get clean
rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
rm -f /root/.bash_history "${FPMS_HOME}/.bash_history" 2>/dev/null || true
find /var/log -type f -exec truncate -s 0 {} \; 2>/dev/null || true

# The build tree stays: docs/, selftest/ and SPEC.md are useful ON the rover,
# and fpms-doctor and the unit Documentation= lines point into it. Only the
# staged source copy goes.
rm -rf /opt/fpms-os/src

# --- zero free space so the image compresses ---------------------------------
# Without this the .img.xz is several GB larger for no reason.
dd if=/dev/zero of=/EMPTY bs=1M 2>/dev/null || true
rm -f /EMPTY
sync

echo "--- 90-finalise OK"
