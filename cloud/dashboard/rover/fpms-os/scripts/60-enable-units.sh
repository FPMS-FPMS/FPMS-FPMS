#!/usr/bin/env bash
# Stage 60 - enable, mask, and deliberately-not-enable.
#
# EVERY BOOT UNIT IS ENABLED HERE. fpms-missions was once found DISABLED on the
# running Pi, so the rover came back from a power cycle with no mission
# executor and nothing said so. An image is the right place to make that
# impossible.
#
# THIS STAGE DOES NOT TRUST systemctl.
# ====================================
# There is no running systemd in this chroot, and there cannot be. `systemctl
# enable` is *supposed* to notice that (sd_booted() fails because build.sh
# mounts a fresh tmpfs on /run, so /run/systemd/system does not exist) and drop
# into offline mode, where it just writes .wants symlinks. Usually it does.
# But "usually" is not a property you want between a six-hour build and a rover
# that boots into nothing, and under qemu-user emulation the chroot detection
# has more ways to go wrong than it has ways to go right.
#
# So: run systemctl, ignore its exit code, and then VERIFY THE SYMLINK. If the
# symlink is not there, create it by hand -- which is all `enable` ever did.
# The only thing that fails this stage is a missing unit or a missing symlink,
# never systemctl being unhappy about its environment.
set -euo pipefail
echo "--- 60-enable-units"

# Belt and braces on the above: this is systemd's own documented "pretend there
# is no init and just edit files" switch.
export SYSTEMD_OFFLINE=1

ETC=/etc/systemd/system
VENDOR=/usr/lib/systemd/system

# Expected to fail -- there is no bus to reload. Kept because on a native
# aarch64 host that happens to be running systemd it is harmless and correct.
systemctl daemon-reload >/dev/null 2>&1 || true

BOOT_UNITS=(
    fpms-firstboot.service          # must run before anything reads config.env
    fpms-cored.service              # stop authority, before anything that moves
    fpms-rover-agent.service
    fpms-lidar-ros.service
    fpms-tf.service
    fpms-map-anchor.service
    fpms-odom-tf.service
    fpms-teleop.service
    fpms-missions.service
    fpms-uros-supervisor.service
    fpms-wifi-powersave-hold.service
    micro-ros-agent.service
    # The NPU layer. fpms-npud owns the RKNN runtime so that a slow or wedged
    # inference cannot stall the camera pump that also feeds the fire detector.
    # fpms-npu-tune runs in REPORT mode -- it changes nothing unless an operator
    # passes --apply -- and exists so thermal throttling is visible rather than
    # silently eating the latency budget.
    fpms-npud.service
    fpms-npu-tune.service
    fpms-ros-publishers.target
    fpms-rosbridge.service
    fpms-console.service
    fpms-selftest.service
)

# --- helpers ----------------------------------------------------------------

# Where does this unit actually live? /etc wins over /usr/lib, same as systemd.
unit_path() {
    local d
    for d in "$ETC" "$VENDOR" /lib/systemd/system; do
        if [ -f "$d/$1" ] && [ ! -L "$d/$1" ]; then echo "$d/$1"; return 0; fi
    done
    return 1
}

# Print "wants <target>" / "requires <target>" for every install target in the
# unit's [Install] section. This is the whole of what `systemctl enable` does
# for these units -- none of them use Alias= or Also=, which would need more.
install_targets() {
    awk '
        /^[[:space:]]*\[/ { ininstall = ($0 ~ /^[[:space:]]*\[Install\][[:space:]]*$/); next }
        !ininstall { next }
        /^[[:space:]]*WantedBy[[:space:]]*=/   { kind = "wants" }
        /^[[:space:]]*RequiredBy[[:space:]]*=/ { kind = "requires" }
        kind != "" {
            line = $0; sub(/^[^=]*=/, "", line)
            n = split(line, a, /[ \t]+/)
            for (i = 1; i <= n; i++) if (a[i] != "") print kind, a[i]
            kind = ""
        }
    ' "$1"
}

# --- enable -----------------------------------------------------------------

fail=0
for u in "${BOOT_UNITS[@]}"; do
    if ! path="$(unit_path "$u")"; then
        echo "FATAL: no unit file for $u in $ETC or $VENDOR" >&2
        echo "  A typo in BOOT_UNITS, or a unit missing from the overlay. Either way" >&2
        echo "  the image would boot without it and nothing at runtime would say so." >&2
        fail=1; continue
    fi

    # A stale mask symlink would make `enable` a no-op that reports success.
    if [ -L "$ETC/$u" ] && [ "$(readlink "$ETC/$u")" = /dev/null ]; then
        echo "FATAL: $u is masked but listed in BOOT_UNITS" >&2
        fail=1; continue
    fi

    systemctl enable "$u" >/dev/null 2>&1 || true

    n=0; missing=""
    while read -r kind tgt; do
        [ -n "${tgt:-}" ] || continue
        n=$((n + 1))
        link="$ETC/${tgt}.${kind}/$u"
        if [ ! -L "$link" ]; then
            mkdir -p "$ETC/${tgt}.${kind}"
            ln -sf "$path" "$link"
        fi
        [ -L "$link" ] || missing="$missing $link"
    done <<EOF
$(install_targets "$path")
EOF

    if [ "$n" = 0 ]; then
        # `systemctl enable` on a unit with no [Install] prints "The unit files
        # have no installation config" and changes nothing. It would sit in
        # this list looking enabled forever.
        echo "FATAL: $u has no [Install] section - it can never be enabled" >&2
        fail=1; continue
    fi
    if [ -n "$missing" ]; then
        echo "FATAL: could not create .wants symlink(s) for $u:$missing" >&2
        fail=1; continue
    fi
    echo "    enabled  $u  ($n link(s))"
done
[ "$fail" = 0 ] || { echo "FATAL: one or more boot units could not be enabled" >&2; exit 1; }

# --- masked, permanently ----------------------------------------------------
#
# Both can write /cmd_vel. Two writers is not a race the mission executor can
# win, or even reliably detect in time.
#
# MASKED, NOT DISABLED. `disable` only removes the WantedBy symlink; anything
# that pulls the unit in by name still starts it. Masking points it at
# /dev/null so no path can start it at all.
#
# They are SHIPPED so they can be masked: masking a unit that does not exist
# is not the same guarantee, because a restore from backup or a copy from the
# old Pi would put an unmasked writer back on the robot.
#
# THE UNIT FILE HAS TO MOVE FIRST.
# ================================
# A mask IS a symlink at /etc/systemd/system/<unit> -> /dev/null. The overlay
# ships these two units as regular files at exactly that path, so `systemctl
# mask` refuses with "File /etc/systemd/system/... already exists", the old
# `|| true` swallowed it, and the is-enabled check below then found "disabled"
# and killed the stage -- at the very end of a multi-hour build.
#
# The fix is to put the real unit body in the vendor directory, /usr/lib/
# systemd/system, which is where a shipped-but-overridden unit belongs, and
# leave /etc free for the mask. The unit is still present on the image, so the
# "shipped so it can be masked" guarantee is intact and stronger: /etc now
# wins over it by systemd's own precedence rules rather than by luck.
for u in fpms-ros-tunnel.service fpms-rtos-follower.service; do
    if [ -f "$ETC/$u" ] && [ ! -L "$ETC/$u" ]; then
        mkdir -p "$VENDOR"
        mv -f "$ETC/$u" "$VENDOR/$u"
        chmod 0644 "$VENDOR/$u"; chown root:root "$VENDOR/$u"
        echo "    moved    $u -> $VENDOR (so /etc can hold the mask)"
    fi
    [ -f "$VENDOR/$u" ] || { echo "FATAL: $u is not shipped anywhere - nothing to mask" >&2; exit 1; }

    # Drop any .wants/.requires symlink first. `mask` does not remove those,
    # and a dangling one in multi-user.target.wants is noise in every boot log.
    find "$ETC" "$VENDOR" /lib/systemd/system -mindepth 2 -name "$u" -type l -delete 2>/dev/null || true

    systemctl disable "$u" >/dev/null 2>&1 || true
    systemctl mask    "$u" >/dev/null 2>&1 || true

    # Verify the symlink, not the exit code, and not `systemctl is-enabled` --
    # which needs the same environment we already decided not to trust.
    if [ ! -L "$ETC/$u" ] || [ "$(readlink "$ETC/$u")" != /dev/null ]; then
        ln -sf /dev/null "$ETC/$u"
    fi
    [ -L "$ETC/$u" ] && [ "$(readlink "$ETC/$u")" = /dev/null ] \
        || { echo "FATAL: could not mask $u ($ETC/$u is not a symlink to /dev/null)" >&2; exit 1; }

    # If systemctl IS working here, its opinion is a free second check.
    state="$(systemctl is-enabled "$u" 2>/dev/null || true)"
    case "$state" in
        masked|masked-runtime|"") ;;
        *) echo "FATAL: $u masked on disk but systemctl reports '$state'" >&2; exit 1 ;;
    esac
    echo "    masked   $u"
done

# --- installed but deliberately NOT enabled ---------------------------------
#
# fpms-ros-settle: 75 seconds of sleep mitigating the SHM bug that
#   /etc/fpms/fastdds_udp_only.xml now removes at the cause. Kept as a
#   fallback, because removing a mitigation for a bug not yet confirmed fixed
#   on hardware is a mistake this project has made before.
#
# fpms-nav2, fpms-slam-*: three preconditions are unmet -- the LiDAR mount
#   transform is unmeasured, LIDAR_ROTATION_SIGN is unverified, and a saved
#   map cannot ship in an image. Their [Install] sections are absent so they
#   cannot drift into the boot path by accident.
#
# fpms-ros-settle DOES have an [Install], so "not enabled" is a state that has
# to be actively checked rather than assumed. Delete the symlink directly for
# the same reason as everything else in this file.
systemctl disable fpms-ros-settle.service >/dev/null 2>&1 || true
find "$ETC" /lib/systemd/system "$VENDOR" -mindepth 2 -name fpms-ros-settle.service -type l -delete 2>/dev/null || true
# Capture, not `find | grep -q`: pipefail turns grep -q's early exit into
# find's SIGPIPE (141) and the condition reads backwards.
if [ -n "$(find "$ETC" -mindepth 2 -name fpms-ros-settle.service -type l 2>/dev/null)" ]; then
    echo "FATAL: fpms-ros-settle.service is still enabled" >&2; exit 1
fi
echo "    installed-not-enabled: fpms-ros-settle, fpms-nav2, fpms-slam-mapping, fpms-slam-localization"

# --- verify -----------------------------------------------------------------
#
# On-disk state, which is the state the flashed image will actually boot with.
# `systemctl is-enabled` is printed alongside it only as commentary; a "?" here
# means systemctl could not answer in the chroot, NOT that the unit is broken.
echo "--- enable state (on disk | systemctl):"
for u in "${BOOT_UNITS[@]}"; do
    links="$(find "$ETC" -mindepth 2 -name "$u" -type l 2>/dev/null | wc -l | tr -d ' ')"
    printf '    %-36s %s link(s) | %s\n' \
        "$u" "$links" "$(systemctl is-enabled "$u" 2>/dev/null || echo '?')"
    [ "$links" -ge 1 ] || { echo "FATAL: $u ended with no .wants symlink" >&2; exit 1; }
done

echo "--- 60-enable-units OK"
