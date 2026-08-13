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
    # THE DASHBOARD THE ROVER SERVES ITSELF, on :8090.
    #
    # fpms-console.service is deliberately NOT here any more. It
    # ExecStarted /home/ubuntu/fpms_console/fpms_console.py, which is in
    # no repository and exists only on the old Pi -- so it was enabled and
    # FAILED ON EVERY BOOT of every image ever built (verify_image.sh
    # reports it). An enabled unit that cannot start is worse than an
    # absent one: it fills the journal and makes `systemctl --failed`
    # useless as a signal, which teaches an operator to ignore red.
    fpms-dashboard.service
    # ROS-level hardware verification: is the LiDAR producing data, is the
    # camera, is the ESP32 link actually carrying /odom_raw, /imu and /battery.
    # Deeper than fpms-selftest, which checks presence; this checks PRODUCTION.
    # Shipped-but-not-enabled is the exact failure this project already has on
    # record for fpms-missions, so it goes in the boot set, not beside it.
    fpms-hwcheck.service
    # Boot-time detection-model provisioning. Installs a .rknn the operator
    # dropped on the FAT boot partition from Windows -- the only realistic
    # delivery path for a file that cannot live in git -- verifies its output
    # shape, and FAILS THE UNIT if there is no usable model. That failure is
    # the point: without it the agent logs "NPU unavailable; streaming without
    # detection" once and runs blind forever with every unit reporting active.
    fpms-model-provision.service
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
# (Re-checked against overlay/etc/systemd/system: no Alias=, no Also=, no
# RequiredBy=, no DefaultInstance=, and no templates.)
#
# Handles all three shapes systemd accepts, because a target this misses is a
# unit that prints as enabled below and never starts on the rover:
#   - repeated WantedBy= lines (six units here do this);
#   - several targets on one line, space separated;
#   - a line CONTINUED with a trailing backslash. That last one used to be
#     actively dangerous rather than merely unsupported: "WantedBy=a.target \"
#     split the literal backslash out as its own target, so the loop below
#     created /etc/systemd/system/\.wants/ and then dropped the real target on
#     the continuation line entirely -- silently, in both directions.
#
# No backslash inside an awk regex constant. gawk's lexer collapses "\\" before
# the regex compiler sees it, so /\\[ \t]*$/ does NOT match a line ending in a
# backslash (measured on gawk 5.4 against a real unit line). substr() is
# unambiguous and needs no quoting archaeology.
install_targets() {
    awk '
        /^[[:space:]]*\[/ {
            ininstall = ($0 ~ /^[[:space:]]*\[Install\][[:space:]]*$/); cont = 0; next
        }
        !ininstall { next }
        {
            line = $0
            if (cont) { kind = contkind }
            else {
                kind = ""
                if (line ~ /^[[:space:]]*WantedBy[[:space:]]*=/)   kind = "wants"
                if (line ~ /^[[:space:]]*RequiredBy[[:space:]]*=/) kind = "requires"
                if (kind == "") next
                sub(/^[^=]*=/, "", line)
            }
            cont = 0
            sub(/[ \t]+$/, "", line)
            if (length(line) > 0 && substr(line, length(line)) == "\\") {
                line = substr(line, 1, length(line) - 1)
                cont = 1; contkind = kind
            }
            n = split(line, a, /[ \t]+/)
            for (i = 1; i <= n; i++) if (a[i] != "") print kind, a[i]
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

    n=0; missing=""; bogus=""
    while read -r kind tgt; do
        [ -n "${tgt:-}" ] || continue
        # $tgt is about to become a DIRECTORY NAME under /etc/systemd/system.
        # Check it looks like a unit before letting it near mkdir -p: a parse
        # slip that yields "\", "..", or a path fragment would otherwise create
        # a junk directory on the image and count as a successful enable. The
        # parser above is careful, but this is the cheap check that turns any
        # future parser bug into a build failure instead of a rover that boots
        # without its mission executor.
        case "$tgt" in
            *[!A-Za-z0-9@:._-]*|*/*)
                bogus="$bogus '$tgt'"; continue ;;
            *.target|*.service|*.socket|*.timer|*.path|*.mount|*.slice) ;;
            *) bogus="$bogus '$tgt'"; continue ;;
        esac
        n=$((n + 1))
        link="$ETC/${tgt}.${kind}/$u"
        # Three ways this link can be wrong, and only the first is obvious:
        #   - not a symlink at all: nothing enabled the unit;
        #   - a symlink that DANGLES: systemd skips it in silence at boot, so
        #     the unit is listed here as "enabled" and never starts. `systemctl
        #     enable` can leave one if it resolved the unit body to a path we
        #     did not, which is exactly the chroot behaviour we do not trust;
        #   - a symlink to some OTHER path: stale, from a build where the body
        #     lived elsewhere -- e.g. before the mask section moved a unit into
        #     $VENDOR.
        # One `ln -sfn` repairs all three and is idempotent.
        if [ ! -L "$link" ] || [ ! -e "$link" ] \
           || [ "$(readlink "$link")" != "$path" ]; then
            mkdir -p "$ETC/${tgt}.${kind}"
            # -n so that if $link is somehow a symlink TO A DIRECTORY, ln
            # replaces it instead of quietly dropping the new link INSIDE it
            # (where systemd would never look). `|| true` because the check on
            # the next line is the verdict -- ln's stderr still reaches the log,
            # and an ln that failed shows up as a missing link with a name
            # attached, which is a better report than a bare exit at hour 15.
            ln -sfn "$path" "$link" || true
        fi
        # -e follows the link: "exists AND resolves". A dangling link is not a
        # working enable and must not be counted as one.
        [ -L "$link" ] && [ -e "$link" ] || missing="$missing $link"
    done <<EOF
$(install_targets "$path")
EOF

    if [ -n "$bogus" ]; then
        echo "FATAL: $u [Install] yielded target name(s) that are not units:$bogus" >&2
        echo "  Either the unit file is malformed or install_targets() mis-parsed it." >&2
        fail=1; continue
    fi
    if [ "$n" = 0 ]; then
        # `systemctl enable` on a unit with no [Install] prints "The unit files
        # have no installation config" and changes nothing. It would sit in
        # this list looking enabled forever.
        echo "FATAL: $u has no [Install] section - it can never be enabled" >&2
        fail=1; continue
    fi
    if [ -n "$missing" ]; then
        echo "FATAL: missing or dangling .wants symlink(s) for $u:$missing" >&2
        fail=1; continue
    fi
    echo "    enabled  $u  ($n link(s))"
done

# NOT `[ "$fail" = 0 ] || exit 1` here.
#
# This is the last substantive stage of a build that takes the better part of a
# day, and every early exit costs a whole run to learn about the next problem.
# Nothing after this point depends on the enable loop having succeeded -- the
# mask section touches different units, and the verify section only reads --
# so keep going, collect everything, and fail ONCE at the bottom with the full
# list. One run, one complete answer.

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
    [ -f "$VENDOR/$u" ] || {
        echo "FATAL: $u is not shipped anywhere - nothing to mask" >&2
        fail=1; continue
    }

    # Drop any .wants/.requires symlink first. `mask` does not remove those,
    # and a dangling one in multi-user.target.wants is noise in every boot log.
    find "$ETC" "$VENDOR" /lib/systemd/system -mindepth 2 -name "$u" -type l -delete 2>/dev/null || true

    systemctl disable "$u" >/dev/null 2>&1 || true
    systemctl mask    "$u" >/dev/null 2>&1 || true

    # Verify the symlink, not the exit code, and not `systemctl is-enabled` --
    # which needs the same environment we already decided not to trust.
    if [ ! -L "$ETC/$u" ] || [ "$(readlink "$ETC/$u")" != /dev/null ]; then
        ln -sfn /dev/null "$ETC/$u" || true
    fi
    [ -L "$ETC/$u" ] && [ "$(readlink "$ETC/$u")" = /dev/null ] \
        || { echo "FATAL: could not mask $u ($ETC/$u is not a symlink to /dev/null)" >&2
             fail=1; continue; }

    # If systemctl IS working here, its opinion is a free second check -- but
    # only a check, never a verdict. This line used to `exit 1` on any answer
    # it did not like, which contradicts the doctrine at the top of this file
    # and hands systemctl exactly the power to kill a sixteen-hour build that
    # the rest of the stage is written to deny it. A systemctl that cannot
    # resolve the unit under qemu answers "not-found" and exits 4; the mask
    # symlink verified two lines above is still on the disk the rover boots.
    state="$(systemctl is-enabled "$u" 2>/dev/null || true)"
    case "$state" in
        masked|masked-runtime|"") ;;
        *) echo "    WARNING: $u is masked on disk but systemctl says '$state'." >&2
           echo "    The on-disk symlink is what the flashed image boots with;" >&2
           echo "    treating this as commentary, not as a failure." >&2 ;;
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
# "not enabled" is a state that has to be actively CHECKED rather than assumed,
# and that goes for all four, not just fpms-ros-settle.
#
# fpms-ros-settle has an [Install] and so is one edit away from being enabled.
# The other three have none TODAY -- but "their [Install] sections are absent"
# is a claim about files this stage does not own, and the whole doctrine of
# this file is that a claim printed to the log is worth nothing next to a check
# against the disk. Adding a WantedBy= to fpms-nav2.service is exactly the sort
# of well-meant edit that would put an unmeasured LiDAR transform into the boot
# path, and until now this stage would have printed
# "installed-not-enabled: ... fpms-nav2 ..." over the top of it.
#
# `disable` is called on all four; it is a no-op on a unit with no [Install]
# and is only ever advisory here anyway. The find/delete is what actually does
# the work, for the same reason as everything else in this file.
# fpms-console is listed rather than deleted: an operator who has copied
# fpms_console across from the old Pi can `systemctl enable --now fpms-console`
# and have it back. It is superseded by fpms-dashboard.service, not forbidden.
for u in fpms-ros-settle.service fpms-nav2.service \
         fpms-slam-mapping.service fpms-slam-localization.service \
         fpms-console.service; do
    systemctl disable "$u" >/dev/null 2>&1 || true
    find "$ETC" /lib/systemd/system "$VENDOR" -mindepth 2 -name "$u" -type l -delete 2>/dev/null || true
    # Capture, not `find | grep -q`: pipefail turns grep -q's early exit into
    # find's SIGPIPE (141) and the condition reads backwards. Inside `[ -n ... ]`
    # the substitution's own exit status is discarded, so there is no pipeline and
    # nothing for pipefail to act on.
    #
    # Search the same three roots the delete above swept, not just $ETC: a .wants
    # link under the vendor tree enables the unit exactly as well as one under
    # /etc, so checking only /etc would report success over a live symlink.
    if [ -n "$(find "$ETC" "$VENDOR" /lib/systemd/system -mindepth 2 \
                  -name "$u" -type l 2>/dev/null)" ]; then
        echo "FATAL: $u is still enabled" >&2
        fail=1; continue
    fi
    echo "    installed-not-enabled: $u"
done

# --- verify -----------------------------------------------------------------
#
# On-disk state, which is the state the flashed image will actually boot with.
# `systemctl is-enabled` is printed alongside it only as commentary; a "?" here
# means systemctl could not answer in the chroot, NOT that the unit is broken.
echo "--- enable state (on disk | systemctl):"
#
# Counted with a GLOB, not with `find | wc -l`. That pipeline was the last
# instance in this file of the bug class that has killed this build twice:
# under `set -o pipefail` a find that returns non-zero for any reason at all
# (an unreadable subdirectory is enough) makes the whole `links="$(...)"`
# assignment non-zero, `set -e` takes the stage down, and 2>/dev/null has
# already thrown away the only clue -- on the very last screenful of a
# sixteen-hour build. Measured: `n="$(find /nosuchdir | wc -l)"` under
# `set -euo pipefail` exits 1 and prints nothing.
#
# An unmatched glob here expands to the literal path containing '*', which is
# not a symlink, so it counts as zero. No nullglob needed, no subshell, no
# pipeline, nothing for pipefail to act on.
for u in "${BOOT_UNITS[@]}"; do
    links=0; dangling=""
    for l in "$ETC"/*.wants/"$u" "$ETC"/*.requires/"$u"; do
        [ -L "$l" ] || continue
        # -e follows the link. A .wants entry pointing at a unit file that is
        # not there is skipped in silence by systemd at boot: the unit would
        # print as enabled here and never start on the rover.
        if [ -e "$l" ]; then links=$((links + 1)); else dangling="$dangling $l"; fi
    done
    printf '    %-36s %s link(s) | %s\n' \
        "$u" "$links" "$(systemctl is-enabled "$u" 2>/dev/null || echo '?')"
    # Collected, not exited on. This loop walks every boot unit; exiting at the
    # first bad one meant learning about exactly one broken unit per build, and
    # a build is fifteen hours. Print the whole table, flag every fault in it,
    # and decide once at the bottom.
    [ -z "$dangling" ] \
        || { echo "FATAL: $u has dangling .wants symlink(s):$dangling" >&2; fail=1; }
    [ "$links" -ge 1 ] || { echo "FATAL: $u ended with no .wants symlink" >&2; fail=1; }
done

# The one and only exit. Everything above records into $fail and keeps going,
# so a single run reports every fault it can see rather than the first one.
[ "$fail" = 0 ] || {
    echo "FATAL: 60-enable-units found faults above. The image is NOT bootable as" >&2
    echo "  intended - at least one unit would be missing at boot with nothing at" >&2
    echo "  runtime to say so. Fix them all, then resume with: sudo ./build.sh --from 50" >&2
    exit 1
}

echo "--- 60-enable-units OK"
