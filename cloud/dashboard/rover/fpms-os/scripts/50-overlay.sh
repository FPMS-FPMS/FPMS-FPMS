#!/usr/bin/env bash
# Stage 50 - apply the rootfs overlay and enforce the modes that matter.
set -euo pipefail
echo "--- 50-overlay"

OVL=/opt/fpms-os/overlay

# The boot partition may not be mounted in the chroot; keep its files aside
# and let stage 90 place them, rather than writing them into the rootfs /boot
# where the operator will never see them from Windows.
cp -a "$OVL/." / 2>/dev/null || true

# --- modes that are load-bearing --------------------------------------------

# config.env holds the broker password. 0640 root:root.
chmod 0640 /etc/fpms/config.env; chown root:root /etc/fpms/config.env

# The sudoers rule. VALIDATE IT -- an invalid file in /etc/sudoers.d does not
# just disable this rule, it can break sudo entirely, and the failure would
# first show up as the last-resort STOP not working.
chmod 0440 /etc/sudoers.d/fpms-cored
chown root:root /etc/sudoers.d/fpms-cored
visudo -cf /etc/sudoers.d/fpms-cored >/dev/null \
    || { echo "FATAL: /etc/sudoers.d/fpms-cored fails visudo -cf" >&2; exit 1; }

# The rule must also MATCH what fpms_cored.py actually runs. sudo compares the
# literal argv, so "fpms-missions" and "fpms-missions.service" are different
# rules, and a mismatch turns the escalation path into a silent password
# prompt on a moving rover. Check the two against each other at build time.
if [ -f "${FPMS_HOME}/fpms_cored.py" ]; then
    if grep -q 'fpms-missions.service' "${FPMS_HOME}/fpms_cored.py" \
       && grep -q 'fpms-missions.service' /etc/sudoers.d/fpms-cored; then
        echo "    sudoers argv matches fpms_cored.py"
    else
        echo "    WARNING: could not confirm the sudoers argv matches fpms_cored.py" >&2
        echo "    Verify by hand: the rule and the subprocess call must agree exactly." >&2
    fi
fi

# mosquitto config carries the bridge password.
chmod 0600 /etc/mosquitto/conf.d/fpms.conf
chown mosquitto:mosquitto /etc/mosquitto/conf.d/fpms.conf 2>/dev/null || true
[ -f /etc/mosquitto/fpms.acl ] && {
    chmod 0644 /etc/mosquitto/fpms.acl
    chown root:root /etc/mosquitto/fpms.acl
}

# Executables.
chmod 0755 /usr/local/bin/* /usr/local/sbin/* 2>/dev/null || true

# The DDS profile and rosbridge whitelist must be world-readable: the services
# run as `ubuntu`, and a profile the process cannot read is silently ignored.
chmod 0644 /etc/fpms/fastdds_udp_only.xml
[ -f /etc/fpms/rosbridge_params.yaml ] && chmod 0644 /etc/fpms/rosbridge_params.yaml

# The model registry. 0644 root:root -- world-readable so the ubuntu-run agent,
# fpms-npud and fpms-model-verify can all read it, root-only writable so
# `fpms-model-verify --register` requires sudo. A model registry an unprivileged
# process could rewrite is not a registry.
[ -f /etc/fpms/models.json ] && { chmod 0644 /etc/fpms/models.json; chown root:root /etc/fpms/models.json; }

# Written by stage 25, not the overlay, but normalise it here with the rest.
[ -f /etc/fpms/npu-versions.json ] && chmod 0644 /etc/fpms/npu-versions.json

# udev.
chmod 0644 /etc/udev/rules.d/99-fpms-*.rules
chown root:root /etc/udev/rules.d/99-fpms-*.rules

# Units.
chmod 0644 /etc/systemd/system/fpms-*.service /etc/systemd/system/*.target \
           /etc/systemd/system/micro-ros-agent.service 2>/dev/null || true

# --- CRLF guard -------------------------------------------------------------
#
# These files are authored on a Windows workstation. A shebang ending in \r
# fails at exec as "/usr/bin/env: bad interpreter: No such file or directory",
# which is spectacularly misleading -- /usr/bin/env plainly exists and the file
# looks perfect in every editor. .gitattributes should prevent this; check
# anyway, because fpms-wait-net is an ExecStartPre with no "-" prefix on two
# units and a CRLF there silently stops the LiDAR and the whole TF tree.
BAD=0
for f in /usr/local/bin/fpms-* /usr/local/sbin/fpms-* \
         /etc/systemd/system/fpms-*.service /etc/fpms/config.env; do
    [ -f "$f" ] || continue
    if head -c 4096 "$f" | grep -q $'\r'; then
        echo "    CRLF in $f -- converting" >&2
        sed -i 's/\r$//' "$f"; BAD=1
    fi
done
[ "$BAD" = 1 ] && echo "    WARNING: CRLF was present. Check .gitattributes." >&2

echo "--- 50-overlay OK"
