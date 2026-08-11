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

# Unlink before copying, for /etc/systemd/system only.
#
# Stage 60 leaves SYMLINKS there at paths this overlay ships regular files to:
# the /dev/null masks for fpms-ros-tunnel and fpms-rtos-follower sit at exactly
# /etc/systemd/system/<unit>. On a re-run (`--from 50`, which is the documented
# way to resume this build) `cp` FOLLOWS an existing destination symlink and
# writes THROUGH it -- so the unit body goes into /dev/null and the mask looks
# untouched. Worse: build.sh bind-mounts the HOST's /dev into the chroot, so
# `cp -a`'s preserve-attributes step then chmods and chowns the build machine's
# own /dev/null.
#
# Deleting the link is safe and self-healing: stage 60 re-creates the mask from
# scratch a few seconds later, after moving the body to $VENDOR. Scoped to the
# unit directory and to names this overlay actually ships, so a mask the base
# image put there for its own reasons is left alone.
for src in "$OVL"/etc/systemd/system/*; do
    [ -f "$src" ] || continue
    dst="/etc/systemd/system/$(basename "$src")"
    [ -L "$dst" ] || continue
    echo "    unlinking $dst -> $(readlink "$dst") so the overlay copy lands" >&2
    rm -f "$dst"
done

# Snapshot the DIRECTORIES the copy is about to merge into.
#
# `cp -a` implies -p, and -p applies to directories, not just to files: when
# the destination directory already exists, cp still stamps the SOURCE
# directory's mode and ownership onto it. The overlay is staged by build.sh
# with `cp -a "$HERE/overlay" ...` from the checkout, and this checkout lives
# on a Windows filesystem, where every directory reads back as 0777 and owned
# by whichever uid mounted it. So `cp -a "$OVL/etc" /` can quietly re-mode
# /etc, /etc/systemd, /etc/udev, /usr, /usr/local -- and /etc/sudoers.d, which
# sudo REFUSES to read if it is group- or world-writable. That failure appears
# on the rover as the last-resort STOP silently not working: the exact fault
# the visudo check below exists to prevent, arriving by a different door.
#
# Every FILE the overlay ships gets an explicit mode further down, so files are
# already covered. Directories were not. Snapshot what is there now, put it
# back afterwards, and say so -- a policy-free repair that cannot invent a mode
# this image did not already have, and a no-op on a build host where cp leaves
# the directories alone.
DIR_SNAP=()
while IFS= read -r d; do
    [ -n "$d" ] || continue
    [ -d "/$d" ] && [ ! -L "/$d" ] || continue
    s="$(stat -c '%a %u %g' "/$d" 2>/dev/null || true)"
    [ -n "$s" ] || continue
    DIR_SNAP+=("$s /$d")
done < <(cd "$OVL" && find . -mindepth 1 -type d -printf '%P\n' 2>/dev/null)

for src in "$OVL"/*; do
    [ "$(basename "$src")" = "boot" ] && continue
    cp -a "$src" /
done
shopt -u dotglob

# Put the directory modes back. Never fatal: a directory that vanished, or a
# stat that cannot answer, is not a reason to lose fifteen hours of build.
if [ "${#DIR_SNAP[@]}" -gt 0 ]; then
    for e in "${DIR_SNAP[@]}"; do
        mode="${e%% *}"; rest="${e#* }"
        uid="${rest%% *}"; rest="${rest#* }"
        gid="${rest%% *}"; p="${rest#* }"
        now="$(stat -c '%a %u %g' "$p" 2>/dev/null || true)"
        [ -n "$now" ] || continue
        if [ "$now" != "$mode $uid $gid" ]; then
            echo "    restoring $p: cp -a stamped it $now, was $mode $uid $gid" >&2
            chmod "$mode" "$p"
            chown "$uid:$gid" "$p"
        fi
    done
fi

# Prove the copy actually happened. cp exiting 0 having copied nothing (an
# empty staged overlay, a bad path) would otherwise sail straight through and
# fail three stages later as "mosquitto won't start".
#
# fpms.conf is on the list because the chmod/chown block below operates on it
# with no existence guard -- deliberately, since a broker config the daemon
# cannot read is a rover with no telemetry. Better it fails HERE, naming the
# overlay, than forty lines later as a bare "chmod: cannot access".
for sentinel in /etc/fpms/config.env /etc/systemd/system/fpms-cored.service \
                /usr/local/bin/fpms-selftest /etc/sudoers.d/fpms-cored \
                /etc/mosquitto/conf.d/fpms.conf \
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
        # VERIFY THE OUTCOME, do not trust sed's exit code.
        #
        # The detector above matches ANY carriage return. The repair only
        # removes one at END OF LINE. A bare mid-line \r -- a stray character,
        # or a file saved with classic-Mac line endings -- therefore survives
        # `sed`, which still exits 0, and this loop moves on having ANNOUNCED a
        # conversion that did not happen. Measured: a file containing
        # "mid\rCR here\n" is unchanged by `sed -i 's/\r$//'`.
        #
        # That is worse than not checking at all, because the log then says the
        # file is clean. There is no safe automatic repair for a bare CR (only
        # a human knows whether it should have been a newline or nothing), so
        # stop here, while the fix is a one-line edit in the repo rather than a
        # rover that will not start its LiDAR.
        if LC_ALL=C grep -q $'\r' "$f"; then
            echo "FATAL: $f still contains a carriage return after conversion." >&2
            echo "  This is a bare CR that is not at end of line, so the CRLF" >&2
            echo "  repair does not touch it. Fix the file in the repo and check" >&2
            echo "  .gitattributes; do not paste it back from a Windows editor." >&2
            exit 1
        fi
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
    local cand t m
    for cand in /boot/firmware /boot/efi /boot; do
        [ -d "$cand" ] || continue
        t="$(stat -f -c %T "$cand" 2>/dev/null || true)"
        case "$t" in
            msdos|vfat|fat|exfat) echo "$cand"; return 0 ;;
        esac
        # Belt and braces on the single most consequential decision in this
        # stage. When coreutils has no name for a filesystem it prints
        # "UNKNOWN (0x...)" and the string match above misses -- and a miss
        # here is silent, because the fallback branch below still "works".
        # The magic number does not change: 4d44 is MSDOS_SUPER_MAGIC, which
        # is what FAT12/16/32 and vfat all report; 2011bab0 is exfat.
        m="$(stat -f -c %t "$cand" 2>/dev/null || true)"
        case "$m" in
            4d44|2011bab0) echo "$cand"; return 0 ;;
        esac
    done
    return 1
}

[ -d "$OVL/boot" ] || { echo "FATAL: $OVL/boot is missing - the operator-visible WiFi files were never staged" >&2; exit 1; }

# ...and that it is not EMPTY. `nullglob` is off by this point (it is unset
# after the CRLF sweep), so an empty $OVL/boot leaves "$OVL"/boot/* as the
# literal string, `cp` fails on a path containing '*', and the operator gets
# "cannot stat '/opt/fpms-os/overlay/boot/*'" instead of the actual fault. The
# landing check further down would not save us either: it iterates the same
# empty glob and passes vacuously. Name the real problem here.
boot_n=0
for b in "$OVL"/boot/*; do
    [ -f "$b" ] || continue
    boot_n=$((boot_n + 1))
done
[ "$boot_n" -gt 0 ] || {
    echo "FATAL: $OVL/boot is empty - README-FPMS.txt and fpms-wifi.conf.example" >&2
    echo "  were not staged. They are the entire pre-first-boot operator interface." >&2
    exit 1
}

if BOOTDIR="$(fat_boot_dir)"; then
    # No chmod/chown: vfat has neither, and both would fail the stage.
    cp -f "$OVL"/boot/* "$BOOTDIR"/
    echo "    boot files -> $BOOTDIR (FAT, visible from Windows)"
else
    BOOTDIR=/boot
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
    # build.sh no longer tolerates this: prepare_image() mounts $BOOTPART on
    # $MNT/boot/firmware with `|| die`. So reaching this branch means one of
    # two things, and neither is fixed by editing build.sh:
    #   - this stage was run by hand inside a chroot nobody mounted the FAT
    #     partition into (`--stage 50` against a half-set-up tree), or
    #   - the partition IS mounted but statfs does not call it FAT, which after
    #     the magic-number fallback above would be a genuinely novel filesystem.
    echo "    build.sh mounts the FAT partition with '|| die', so this stage was" >&2
    echo "    almost certainly run outside build.sh. Re-run the whole build, or" >&2
    echo "    mount the image's FAT partition on \$MNT/boot/firmware first." >&2
fi

# Prove each file landed, wherever it went. `cp` returning 0 having written
# nothing useful is the same class of non-event the sentinel block above
# guards the rootfs copy against, and this file IS the entire pre-first-boot
# interface -- a FAT partition that is full, or read-only, or an overlay/boot
# that staged empty, must not read as success.
for b in "$OVL"/boot/*; do
    [ -f "$b" ] || continue
    [ -f "$BOOTDIR/$(basename "$b")" ] \
        || { echo "FATAL: $(basename "$b") did not land in $BOOTDIR" >&2; exit 1; }
done

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
#
# `[ -f "$f" ] || continue`, not `[ -f "$f" ] && chmod ...`. Both work HERE --
# bash suppresses errexit for a && list whose left operand fails, and this loop
# is at top level -- but the moment anyone wraps this in a function the same
# line makes the function return 1 on its last iteration and `set -e` kills the
# stage. Measured: `f(){ for x in a b; do [ -f "/nope/$x" ] && echo hi; done; }`
# under `set -euo pipefail` exits 1 at the call site, while the identical loop
# at top level does not. Do not leave that landmine in a file that only ever
# runs at hour fifteen.
shopt -s nullglob
for f in /usr/local/bin/* /usr/local/sbin/*; do
    [ -f "$f" ] || continue
    chmod 0755 "$f"
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
