#!/usr/bin/env bash
# Stage 50 - apply the rootfs overlay and enforce the modes that matter.
set -euo pipefail
echo "--- 50-overlay"

OVL=/opt/fpms-os/overlay

[ -d "$OVL" ] || { echo "FATAL: $OVL is missing - build.sh did not stage the overlay" >&2; exit 1; }

# --- the copy ---------------------------------------------------------------
#
# NOT `cp -a "$OVL/." /`. That would drop overlay/boot/ into the ROOTFS /boot,
# which is ext4 and which the operator cannot see from Windows. The WiFi
# provisioning file is the entire first-boot user interface: if it is not on
# the FAT partition it may as well not exist. boot/ is handled separately,
# below.
#
# No `|| true` either. A failed overlay copy is not a warning - every chmod
# after this point, and the whole rest of the image, depends on it.
shopt -s dotglob nullglob
for src in "$OVL"/*; do
    [ "$(basename "$src")" = "boot" ] && continue
    cp -a "$src" /
done
shopt -u dotglob

# Prove the copy actually happened. cp exiting 0 having copied nothing (an
# empty staged overlay, a bad path) would otherwise sail straight through and
# fail three stages later as "mosquitto won't start".
for sentinel in /etc/fpms/config.env /etc/systemd/system/fpms-cored.service \
                /usr/local/bin/fpms-selftest /etc/sudoers.d/fpms-cored \
                /etc/udev/rules.d/99-fpms-serial.rules; do
    [ -f "$sentinel" ] || { echo "FATAL: overlay copy did not produce $sentinel" >&2; exit 1; }
done

# --- CRLF guard -------------------------------------------------------------
#
# FIRST, not last. These files are authored on a Windows workstation. A shebang
# ending in \r fails at exec as "/usr/bin/env: bad interpreter: No such file or
# directory", which is spectacularly misleading -- /usr/bin/env plainly exists
# and the file looks perfect in every editor. .gitattributes should prevent it;
# check anyway, because fpms-wait-net is an ExecStartPre with no "-" prefix on
# two units and a CRLF there silently stops the LiDAR and the whole TF tree.
#
# It runs BEFORE visudo and before the chmod/chown block for two reasons:
#   - a \r in /etc/sudoers.d/fpms-cored is a syntax error, so a late guard
#     means visudo rejects a file the guard was about to fix, and the stage
#     dies on a fault it already knew how to repair;
#   - `sed -i` rewrites the file, so any chown done first would be undone.
#
# The list is deliberately wider than "the shell scripts": micro-ros-agent
# .service does not match fpms-*, an /etc/fpms/config.env whose values all end
# in an invisible \r poisons every EnvironmentFile= consumer, and a \r in a
# mosquitto conf or a udev rule fails just as quietly.
BAD=0
for f in /usr/local/bin/fpms-* /usr/local/sbin/fpms-* \
         /etc/systemd/system/*.service /etc/systemd/system/*.target \
         /etc/fpms/* /etc/fpms/nav2/* /etc/fpms/slam/* /etc/sudoers.d/* \
         /etc/mosquitto/conf.d/*.conf /etc/mosquitto/fpms.acl \
         /etc/udev/rules.d/99-fpms-*.rules \
         /opt/fpms-os/firstboot/*; do
    # Never rewrite through a symlink: /etc/systemd/system is full of them and
    # `sed -i` would replace the link with a regular file, silently detaching
    # the unit from its vendor copy.
    [ -L "$f" ] && continue
    [ -f "$f" ] || continue
    # Whole file, not `head -c 4096`. A CRLF in the [Service] section of a long
    # unit is past the first 4 KiB and breaks exactly as hard as one in line 1.
    if LC_ALL=C grep -q $'\r' "$f"; then
        echo "    CRLF in $f -- converting" >&2
        sed -i 's/\r$//' "$f"; BAD=1
    fi
done
shopt -u nullglob
if [ "$BAD" = 1 ]; then
    echo "    WARNING: CRLF was present. Check .gitattributes." >&2
fi

# --- overlay/boot -> the FAT partition --------------------------------------
#
# build.sh mounts the image's FAT partition at /boot/firmware inside the
# chroot. That is the one an operator can see from Windows after flashing, and
# the only place fpms-wifi.conf.example and README-FPMS.txt are any use.
#
# findmnt is not usable here: the chroot shares the host's /proc, so
# /proc/self/mountinfo lists the host-side paths, not /boot/firmware. statfs
# does not care about any of that.
fat_boot_dir() {
    local cand t
    for cand in /boot/firmware /boot/efi /boot; do
        [ -d "$cand" ] || continue
        t="$(stat -f -c %T "$cand" 2>/dev/null || true)"
        case "$t" in
            msdos|vfat|fat|exfat) echo "$cand"; return 0 ;;
        esac
    done
    return 1
}

if BOOTDIR="$(fat_boot_dir)"; then
    # No chmod/chown: vfat has neither, and both would fail the stage.
    cp -f "$OVL"/boot/* "$BOOTDIR"/
    echo "    boot files -> $BOOTDIR (FAT, visible from Windows)"
else
    # Fall back so the files are at least ON the image, and say so loudly.
    # fpms-firstboot looks in /boot when it cannot find a vfat partition, so
    # provisioning still works from a running rover -- but the operator cannot
    # edit an ext4 /boot from Windows, which is the whole point.
    mkdir -p /boot
    cp -f "$OVL"/boot/* /boot/
    echo "    WARNING: no FAT boot partition is mounted in this chroot." >&2
    echo "    README-FPMS.txt and fpms-wifi.conf.example went to the rootfs /boot," >&2
    echo "    where the operator CANNOT see them from Windows. The image will boot" >&2
    echo "    but there is no way to provision WiFi before first boot." >&2
    echo "    Fix in build.sh: the '\$MNT/boot/firmware' mount must succeed, not '|| true'." >&2
fi

# --- modes that are load-bearing --------------------------------------------

# config.env holds the broker password. 0640 root:root.
chmod 0640 /etc/fpms/config.env; chown root:root /etc/fpms/config.env

# The sudoers rule. VALIDATE IT -- an invalid file in /etc/sudoers.d does not
# just disable this rule, it can break sudo entirely, and the failure would
# first show up as the last-resort STOP not working.
#
# `sudo` (which provides visudo) is installed by stage 00. If it somehow is
# not, say so plainly rather than skipping the check: an unvalidated sudoers
# file shipped in an image is the failure this check exists to prevent.
chmod 0440 /etc/sudoers.d/fpms-cored
chown root:root /etc/sudoers.d/fpms-cored
command -v visudo >/dev/null 2>&1 \
    || { echo "FATAL: visudo is missing - install sudo in stage 00 before this runs" >&2; exit 1; }
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

# mosquitto config carries the bridge password. 0600 is right ONLY if the
# broker's own user owns it -- a 0600 root:root config is a config mosquitto
# cannot read, and the broker then refuses to start with the password file
# missing. Do not swallow that with `|| true`.
chmod 0600 /etc/mosquitto/conf.d/fpms.conf
if id -u mosquitto >/dev/null 2>&1; then
    chown mosquitto:mosquitto /etc/mosquitto/conf.d/fpms.conf
else
    echo "    WARNING: no 'mosquitto' user - the broker package is not installed." >&2
    echo "    /etc/mosquitto/conf.d/fpms.conf is left 0600 root:root." >&2
fi
if [ -f /etc/mosquitto/fpms.acl ]; then
    chmod 0644 /etc/mosquitto/fpms.acl
    chown root:root /etc/mosquitto/fpms.acl
fi

# Executables.
shopt -s nullglob
for f in /usr/local/bin/* /usr/local/sbin/*; do
    [ -f "$f" ] && chmod 0755 "$f"
done

# The DDS profile and rosbridge whitelist must be world-readable: the services
# run as `ubuntu`, and a profile the process cannot read is silently ignored.
# Same for the model registry: 0644 root:root -- world-readable so the
# ubuntu-run agent, fpms-npud and fpms-model-verify can all read it, root-only
# writable so `fpms-model-verify --register` requires sudo. A model registry an
# unprivileged process could rewrite is not a registry.
#
# config.env is the one exception and was set to 0640 above; skip it here so
# this loop cannot widen it back out.
for f in /etc/fpms/*; do
    [ -f "$f" ] || continue
    [ "$f" = /etc/fpms/config.env ] && continue
    chmod 0644 "$f"; chown root:root "$f"
done
[ -f /etc/fpms/fastdds_udp_only.xml ] \
    || { echo "FATAL: /etc/fpms/fastdds_udp_only.xml missing - DDS would fall back to shared memory" >&2; exit 1; }

# udev.
for f in /etc/udev/rules.d/99-fpms-*.rules; do
    chmod 0644 "$f"; chown root:root "$f"
done

# Units. Regular files only -- the base image's /etc/systemd/system holds
# symlinks that must stay symlinks.
for f in /etc/systemd/system/*.service /etc/systemd/system/*.target; do
    [ -L "$f" ] && continue
    [ -f "$f" ] || continue
    chmod 0644 "$f"; chown root:root "$f"
done
shopt -u nullglob

echo "--- 50-overlay OK"
