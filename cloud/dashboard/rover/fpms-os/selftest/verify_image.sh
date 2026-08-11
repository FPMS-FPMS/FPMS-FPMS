#!/usr/bin/env bash
#
# verify_image.sh - did the image actually come out right?
#
#   sudo selftest/verify_image.sh fpms-os-1.0.0-20260811.img.xz
#   sudo selftest/verify_image.sh --json fpms-os-1.0.0-20260811.img
#
#   --json          machine-readable, nothing else on stdout
#   --quiet         exit code only
#   --tmpdir DIR    where to decompress a .xz (default: the image's directory)
#   --keep          keep the decompressed .img instead of deleting it
#   --no-checksum   skip verifying the .sha256 sidecar
#
# Exit: 0 all pass, 1 one or more WARN, 2 one or more FAIL.
#
#
# WHY THIS EXISTS
# ===============
# The build takes fifteen hours; stage 10 alone has been measured at 14h20m.
# When it finally emits fpms-os-<version>-<date>.img.xz, the only checks that
# exist are structural ones over the SOURCE TREE - selftest/test_image_offline.py
# reads overlay/ and scripts/ and never looks at the artefact. Nothing has ever
# inspected what actually landed in the image.
#
# And the artefact cannot be booted here. It is aarch64, the Orange Pi 5B has
# not arrived, and no stage of the build runs a kernel or runs systemd. Mounting
# the produced image and reading it is therefore the ONLY verification available
# between "the build said OK" and "somebody flashed a card and drove a rover
# into a wall".
#
# Every check below is here because the corresponding thing has gone wrong:
#
#   - fpms-missions was found active-but-DISABLED on the old Pi, which is fine
#     until the next reboot, at which point it is silently absent. So this
#     checks for the .wants SYMLINK ON DISK, not for the unit file.
#   - `disable` is not `mask`. Disable only drops the WantedBy symlink; anything
#     that pulls a unit in by name still starts it, and two writers on /cmd_vel
#     is not a race the mission executor can win.
#   - fpms-wait-net was named by three units as an ExecStartPre with no "-"
#     prefix and existed in no repository. Two units could not start at all and
#     the only symptom was a missing LiDAR and a missing TF tree.
#   - a shebang ending "\r" fails at exec with "/usr/bin/env: bad interpreter:
#     No such file or directory", which is a spectacularly misleading message
#     because /usr/bin/env plainly exists and the file looks perfect in every
#     editor. On fpms-wait-net that takes down the drive link.
#   - micro-ros-agent.service sources ~/uros_ws/install/setup.bash under `set -e`.
#     If stage 10's colcon build silently produced nothing, the wrapper dies
#     instantly, with no useful message, and the rover cannot move.
#   - the micro-ROS agent is a CROSS BUILD. A silently-x86 binary in an aarch64
#     image is the nightmare case: it builds, it installs, it is the right size,
#     and it will never execute on the board. Hence the ELF machine check.
#   - /etc/fpms/nav2/ and /etc/fpms/slam/ were once created EMPTY. Three units
#     name files in them by absolute path.
#   - README-FPMS.txt and fpms-wifi.conf.example must be on the FAT partition.
#     On the ext4 rootfs Windows cannot see them, and every WiFi instruction in
#     docs/FLASHING.md then refers to a file the operator cannot reach - which
#     means the rover cannot be provisioned at all.
#   - a machine-id baked into an image is the SAME machine-id on every board
#     ever flashed from it.
#
#
# WHAT THIS CANNOT ESTABLISH
# ==========================
# Say this out loud before quoting a PASS at anybody.
#
#   - IT DOES NOT BOOT THE IMAGE. Not on the board, not in a VM, not under
#     qemu-system. No kernel runs, no systemd runs, no unit is ever observed to
#     start. "Enabled" here means a symlink exists on disk. That is a different
#     claim from "starts".
#   - IT DOES NOT EXECUTE ANYTHING FROM INSIDE THE IMAGE. The binaries are
#     aarch64 and this host is not. Every check is file existence, file mode,
#     file content, or ELF header inspection. An import that would fail at
#     runtime passes here as long as the file is on disk.
#   - NO NPU. There is no NPU device to test against, and no .rknn model in the
#     repository. Whether librknnrt matches the kernel rknpu driver cannot be
#     known until the board boots.
#   - NO SERIAL DEVICES. Neither CP2102, the udev topology rules, the ESP32
#     reset behaviour, nor the 90-225 s micro-ROS re-link are testable here.
#   - NO DDS. Nothing publishes or subscribes. This checks that the profile
#     parses and that useBuiltinTransports is false. Whether data crosses
#     process eras is fpms-selftest's job, on hardware.
#   - NO NETWORK, NO MQTT, NO WIFI. The broker password does not exist until
#     first boot; this checks the placeholder is still THERE.
#   - IT DOES NOT TELL YOU WHETHER THE ROVER DRIVES. Nothing short of the
#     operator's eyes does that, and the odometry has reported clean travel
#     while the rover span in place.
#
# A clean run here means "the image contains the things it is supposed to
# contain, arranged the way they are supposed to be arranged". It is worth a
# lot. It is not the same thing as working.
#
#
# IT NEVER WRITES TO THE IMAGE
# ============================
# The loop device is attached read-only (losetup -r) and both filesystems are
# mounted -o ro. The trap unmounts and detaches on every exit path, including
# INT and TERM, because a leaked loop device makes `losetup --find` hand out a
# second device for the same backing file and the next build then has two
# mounts fighting over one ext4.
#
# Companion tools:
#   selftest/test_image_offline.py   the source tree, before the build
#   selftest/verify_image.sh         this file, the artefact, after the build
#   fpms-selftest                    the rover, on hardware, after the flash
#

set -euo pipefail

PASS="PASS"; WARN="WARN"; FAIL="FAIL"; SKIP="SKIP"

# --------------------------------------------------------------------------
# What the image is supposed to contain.
#
# BOOT_UNITS is stage 60's list, not fpms-selftest's. fpms-selftest asks "is it
# running now"; this asks "did stage 60 write the symlink", and stage 60 is the
# authority on which units it was asked to enable.
# --------------------------------------------------------------------------
BOOT_UNITS=(
    fpms-firstboot.service
    fpms-cored.service
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
    fpms-npud.service
    fpms-npu-tune.service
    fpms-ros-publishers.target
    fpms-rosbridge.service
    fpms-console.service
    fpms-selftest.service
)

# Both can write /cmd_vel. They are SHIPPED so they can be masked - masking a
# unit that does not exist is not the same guarantee, because a restore from
# backup or a copy from the old Pi would put an unmasked writer back on the
# robot. The real body lives in /usr/lib/systemd/system so that /etc is free to
# hold the mask symlink.
MUST_BE_MASKED=(
    fpms-ros-tunnel.service
    fpms-rtos-follower.service
)

# Installed by stage 30 into /home/ubuntu.
PAYLOAD_PY=(
    fpms_missions.py
    fpms_lidar_ros.py
    fpms_teleop.py
    fpms_odom_tf.py
    fpms_duty_driver.py
    fpms_cloud_uplink.py
    fpms_cored.py
    fpms_charact.py
    yolo/fpms_yolo26_npu.py
)

# Installed with `|| true` by stage 30, and their units are masked anyway.
PAYLOAD_PY_OPTIONAL=(
    fpms_ros_tunnel.py
    fpms_rtos_follower.py
)

# The nav2/slam trees. fpms-tf.service referenced ~/nav2/fpms_tf.launch.py for
# months and NOTHING EVER DEPLOYED IT.
PAYLOAD_TREES=(
    nav2/fpms_tf.launch.py
    nav2/fpms_nav2.launch.py
    nav2/nav2_params.yaml
    nav2/nav2_params_slam.yaml
    nav2/make_arena_map.py
    nav2/arena_map.yaml
    slam/fpms_slam_localization.launch.py
    slam/fpms_slam_mapping.launch.py
    slam/mapper_params_localization.yaml
    slam/mapper_params_mapping.yaml
)

# Named without a directory: each is looked for in BOTH /usr/local/bin and
# /usr/local/sbin, so that moving one between them does not silently turn this
# check into a false FAIL - or, worse, a false PASS.
PAYLOAD_EXE=(
    fpms-rover-agent
    fpms-uros-supervisor
    fpms-wait-net
    fpms-wifi-ps-hold
    fpms-uros-release-reset
    fpms-uros-agent-run
    fpms-selftest
    fpms-doctor
    fpms-npud
    fpms-npu-tune
    fpms-npu-selftest
    fpms-model-verify
    fpms-firstboot
    fpms-wifi-provision
)

# The LiDAR mount calibration tool. Two files that are useless apart: the
# library is pure functions with no ROS and no I/O, and the node is the thing
# that subscribes to /scan_lidar and /imu. Both ship in the overlay and are
# placed by stage 50's wholesale copy of overlay/usr/local/**, so neither
# appears in PAYLOAD_EXE - that list is stage 30's, and stage 30 only VERIFIES
# these.
#
# They are checked at all because the mount transform has never been measured:
# every base_link -> laser_frame offset in fpms_tf.launch.py is a MEASURE ME
# placeholder, and fpms-nav2 and both SLAM units refuse to start until
# FPMS_TF_OFFSETS_MEASURED=1. This tool is the only thing in the image that can
# produce those numbers, so an image that shipped without it is an image that
# can never navigate - and nothing else in this file would notice.
CALIB_BIN=fpms-calibrate-lidar
CALIB_LIB=usr/local/lib/fpms/fpms_scanmatch.py

# Named one at a time. A rename inside the library is silent: the module still
# imports and the tool dies on an AttributeError with the operator's hands on
# the rover.
CALIB_FUNCS=(
    scan_to_xy
    wrap_pi
    estimate_rotation
    estimate_translation
    mirror_verdict
    yaw_from_straight_push
    lever_arm_from_rotation
    plane_level_diagnostic
)

# Named by fpms-nav2.service, fpms-slam-localization.service and
# fpms-slam-mapping.service by absolute path, and hard-asserted by stage 40.
ETC_FPMS_CONFIGS=(
    nav2/nav2_params.yaml
    nav2/nav2_params_slam.yaml
    slam/mapper_params_localization.yaml
    slam/mapper_params_mapping.yaml
)

# Build scaffolding that must NOT survive into the artefact. Each of these is
# invisible on a running board until the day it matters.
BUILD_LEFTOVERS=(
    usr/sbin/policy-rc.d
    usr/bin/qemu-aarch64-static
    etc/fpms/.versions.json.new
    home/ubuntu/uros_ws/build
    home/ubuntu/uros_ws/log
    var/lib/systemd/credential.secret
    var/lib/fpms/broker-password
)

# Documented as living only on the old Pi. Their absence is CORRECT on a fresh
# image and must not be reported as a defect - but it must be reported, because
# the units that need them are enabled and will fail.
KNOWN_ABSENT=(
    home/ubuntu/yolo/yolo26n-rk3588.rknn
    home/ubuntu/fpms_console/fpms_console.py
)

PLACEHOLDER='__FPMS_FIRSTBOOT_WILL_REPLACE_THIS__'

# --------------------------------------------------------------------------
# State. Every one of these is read by cleanup(), which runs from the EXIT trap
# and must therefore never touch an unset variable under `set -u`.
# --------------------------------------------------------------------------
IMG=""            # what the user handed us
WORKIMG=""        # the plain .img we actually attach
WORKIMG_OWNED=0   # 1 if we created it and must delete it
LOOPDEV=""
PART_PREFIX=""
KPARTX_USED=0
ROOTPART=""
BOOTPART=""
MNTBASE=""
R=""              # the mounted rootfs
B=""              # the mounted FAT boot partition, "" if there is none

OPT_JSON=0
OPT_QUIET=0
OPT_KEEP=0
OPT_NOSUM=0
OPT_TMPDIR=""

R_NAME=()
R_STATUS=()
R_DETAIL=()
R_REMEDY=()

note() {
    if [ "$OPT_JSON" = 0 ] && [ "$OPT_QUIET" = 0 ]; then
        printf '    %s\n' "$1" >&2
    fi
    return 0
}

die() {
    printf 'verify_image.sh: %s\n' "$1" >&2
    exit 2
}

add() {   # add <name> <status> <detail> [remedy]
    R_NAME+=( "$1" )
    R_STATUS+=( "$2" )
    R_DETAIL+=( "$3" )
    R_REMEDY+=( "${4:-}" )
    return 0
}

# --------------------------------------------------------------------------
# Teardown. This is the part that must never be skipped.
# --------------------------------------------------------------------------
umount_quietly() {   # umount_quietly <dir>
    local d="${1:-}" i
    if [ -z "$d" ]; then return 0; fi
    if ! mountpoint -q "$d" 2>/dev/null; then return 0; fi
    for i in 1 2 3; do
        if umount "$d" 2>/dev/null; then return 0; fi
        sleep 1
    done
    umount -lf "$d" 2>/dev/null || true
    return 0
}

cleanup() {
    set +e
    sync 2>/dev/null

    umount_quietly "$B"
    umount_quietly "$R"
    sync 2>/dev/null

    if [ -n "$LOOPDEV" ]; then
        if [ "$KPARTX_USED" = 1 ]; then
            kpartx -d "$LOOPDEV" >/dev/null 2>&1
        fi
        partx -d "$LOOPDEV" >/dev/null 2>&1
        local i
        for i in 1 2 3 4 5; do
            losetup -d "$LOOPDEV" 2>/dev/null && break
            sleep 1
        done
        # Capture, then test. `losetup -j "$f" | grep -q .` under pipefail
        # reports losetup's SIGPIPE (141), not grep's verdict, and the test
        # then fires exactly backwards. build.sh carries the same comment on
        # the same call for the same reason.
        local still
        still="$(losetup -j "$WORKIMG" 2>/dev/null || true)"
        if [ -n "$still" ]; then
            printf '\033[1;31mWARNING: %s is still attached to %s.\n' \
                   "$WORKIMG" "$LOOPDEV" >&2
            printf '  Run: losetup -d %s   before the next build.\033[0m\n' \
                   "$LOOPDEV" >&2
        fi
        LOOPDEV=""; PART_PREFIX=""; KPARTX_USED=0
        ROOTPART=""; BOOTPART=""
    fi

    if [ -n "$MNTBASE" ]; then
        rmdir "$MNTBASE/boot" "$MNTBASE/root" "$MNTBASE" 2>/dev/null
    fi

    if [ "$WORKIMG_OWNED" = 1 ] && [ "$OPT_KEEP" = 0 ] && [ -n "$WORKIMG" ]; then
        rm -f "$WORKIMG"
    fi

    set -e
    return 0
}
trap cleanup EXIT
# Without an explicit exit the shell RESUMES where it was interrupted, against
# a loop device the handler has just detached.
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# --------------------------------------------------------------------------
# Small helpers. Each one is written so that a missing or unreadable file
# returns a value the caller can name, never a non-zero status that `set -e`
# turns into a silent abort halfway through the report.
# --------------------------------------------------------------------------

# The mounted image's symlinks are absolute against the IMAGE's root, not ours.
# `[ -e /etc/systemd/system/fpms-cored.service ]` on a .wants link resolves
# against the BUILD HOST and answers a question nobody asked. Rebase first.
resolve_in_image() {   # resolve_in_image <linkpath> -> prints host path of target
    local link="$1" tgt=""
    tgt="$(readlink "$link" 2>/dev/null || true)"
    if [ -z "$tgt" ]; then printf '%s' "$link"; return 0; fi
    case "$tgt" in
        /*) printf '%s%s' "$R" "$tgt" ;;
        *)  printf '%s/%s' "$(dirname "$link")" "$tgt" ;;
    esac
    return 0
}

file_mode() {   # file_mode <path> -> "640" or "-"
    local m=""
    m="$(stat -c '%a' "$1" 2>/dev/null || true)"
    if [ -z "$m" ]; then printf '-'; else printf '%s' "$m"; fi
    return 0
}

file_size() {   # file_size <path> -> bytes, or "-"
    local s=""
    s="$(stat -c '%s' "$1" 2>/dev/null || true)"
    if [ -z "$s" ]; then printf '-'; else printf '%s' "$s"; fi
    return 0
}

dir_is_empty() {   # dir_is_empty <dir> -> 0 if empty or absent
    local ents=""
    if [ ! -d "$1" ]; then return 0; fi
    ents="$(find "$1" -mindepth 1 -maxdepth 1 -print 2>/dev/null || true)"
    if [ -z "$ents" ]; then return 0; fi
    return 1
}

read_text() {   # read_text <path> -> file contents, empty if unreadable
    if [ ! -f "$1" ]; then return 0; fi
    cat "$1" 2>/dev/null || true
    return 0
}

# Comments in these files legitimately DISCUSS the strings we are looking for.
# /etc/sudoers.d/fpms-cored spends thirty lines explaining why the argv must be
# "fpms-missions.service", and rosbridge_params.yaml explains why /cmd_vel is
# absent. Matching raw text turns both explanations into a false verdict.
strip_hash_comments() {   # strip_hash_comments <path>
    if [ ! -f "$1" ]; then return 0; fi
    sed -e 's/#.*$//' "$1" 2>/dev/null || true
    return 0
}

# THE CARRIAGE RETURN IS INSPECTED AS BYTES, DELIBERATELY.
#
# The obvious implementation is `line="$(sed -n 1p "$f")"` and then a glob for
# a trailing \r. It was written that way first and it was WRONG: several seds -
# MSYS2's among them - open files in text mode and silently strip the \r on the
# way past, so the check reported "no carriage returns" on a file that plainly
# had one. That is a permanent, silent PASS on the single most load-bearing
# check in this file, which is the exact defect class every other check here
# exists to catch. od -tx1 cannot be talked out of telling the truth.
#
# The pipeline ends in `tr`, which drains to EOF. `head -1` here would SIGPIPE
# its producer, pipefail would report 141, and a plain assignment would adopt
# it - the bug class that has killed this build twice.
hexdump_head() {   # hexdump_head <path> <nbytes> -> " 23 21 2f ... "
    if [ ! -f "$1" ]; then return 0; fi
    od -An -v -tx1 -N "$2" "$1" 2>/dev/null | tr '\n' ' ' | tr -s ' ' || true
    return 0
}

has_shebang() {   # 0 = the file starts "#!"
    local hex=""
    hex="$(hexdump_head "$1" 8)"
    case "$hex" in ' 23 21 '*) return 0 ;; esac
    return 1
}

shebang_is_crlf() {   # 0 = the FIRST line ends CR LF
    local hex="" prefix=""
    hex="$(hexdump_head "$1" 512)"
    case "$hex" in
        *' 0a '*) prefix="${hex%%' 0a '*}" ;;
        *)        return 1 ;;
    esac
    case "$prefix" in *' 0d') return 0 ;; esac
    return 1
}

file_has_cr() {   # 0 = a CR appears anywhere in the first 256 KiB
    local hex=""
    hex="$(hexdump_head "$1" 262144)"
    case "$hex" in *' 0d '*) return 0 ;; esac
    return 1
}

# ELF identification without executing anything. We are on x86_64 and the image
# is aarch64: running `file` is fine, running the binary is not. od is used
# rather than readelf/file so the check works on a build host with nothing but
# coreutils installed.
elf_machine() {   # elf_machine <path> -> aarch64 | x86-64 | arm32 | not-elf | ...
    local f="$1" hdr=""
    local -a b=()
    if [ ! -f "$f" ]; then printf 'missing'; return 0; fi
    hdr="$(od -An -v -tx1 -N 20 "$f" 2>/dev/null || true)"
    if [ -z "$hdr" ]; then printf 'unreadable'; return 0; fi
    hdr="${hdr//$'\n'/ }"
    read -r -a b <<<"$hdr" || true
    # Magic BEFORE length: a colcon-generated shell wrapper is shorter than an
    # ELF header, and calling that "too-short" instead of "not an ELF" sends
    # the reader looking for a truncated binary that does not exist.
    if [ "${#b[@]}" -lt 4 ]; then printf 'too-short'; return 0; fi
    if [ "${b[0]}" != "7f" ] || [ "${b[1]}" != "45" ] \
    || [ "${b[2]}" != "4c" ] || [ "${b[3]}" != "46" ]; then
        printf 'not-elf'; return 0
    fi
    if [ "${#b[@]}" -lt 20 ]; then printf 'elf-truncated'; return 0; fi
    # e_machine is a 16-bit little-endian field at offset 0x12.
    case "${b[18]}${b[19]}" in
        b700) printf 'aarch64' ;;
        3e00) printf 'x86-64' ;;
        2800) printf 'arm32' ;;
        f300) printf 'riscv64' ;;
        *)    printf 'elf-machine-0x%s%s' "${b[19]}" "${b[18]}" ;;
    esac
    return 0
}

HAVE_PY=0
json_parses() {   # json_parses <path> -> 0 ok
    if [ "$HAVE_PY" = 0 ]; then return 2; fi
    python3 -c 'import json,sys; json.load(open(sys.argv[1]))' "$1" >/dev/null 2>&1
    return $?
}

# --------------------------------------------------------------------------
# Argument handling
# --------------------------------------------------------------------------
usage() {
    sed -n '2,16p' "$0"
    return 0
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --json)        OPT_JSON=1 ;;
        --quiet)       OPT_QUIET=1 ;;
        --keep)        OPT_KEEP=1 ;;
        --no-checksum) OPT_NOSUM=1 ;;
        --tmpdir)      shift; OPT_TMPDIR="${1:-}" ;;
        --tmpdir=*)    OPT_TMPDIR="${1#--tmpdir=}" ;;
        -h|--help)     usage; exit 0 ;;
        -*)            die "unknown option: $1" ;;
        *)             IMG="$1" ;;
    esac
    shift
done

if [ -z "$IMG" ]; then
    usage
    die "no image given"
fi
if [ ! -f "$IMG" ]; then
    die "no such file: $IMG"
fi
IMG="$(cd "$(dirname "$IMG")" && pwd)/$(basename "$IMG")"

if [ "$(id -u)" != "0" ]; then
    die "must run as root - losetup and mount both need it.
Run: sudo $0 $IMG"
fi

for t in losetup mount umount blkid blockdev partx od sed find stat; do
    if ! command -v "$t" >/dev/null 2>&1; then
        die "required tool not found: $t"
    fi
done
if command -v python3 >/dev/null 2>&1; then HAVE_PY=1; fi

# --------------------------------------------------------------------------
# 0. The artefact itself, before we open it
# --------------------------------------------------------------------------
check_artefact() {
    local sz="" szmb=0 sums="" sumout=""

    sz="$(file_size "$IMG")"
    if [ "$sz" = "-" ]; then
        add "artefact readable" "$FAIL" "cannot stat $IMG"
        return 0
    fi
    szmb=$(( sz / 1048576 ))
    add "artefact readable" "$PASS" "$(basename "$IMG") (${szmb} MB)"

    case "$IMG" in
        *.img.xz)
            # docs/BUILDING.md: 2-3 GB is the expected shape. A much smaller
            # one is a SIGNAL, not a win - it usually means the rootfs never
            # grew (sgdisk absent, the call is `|| true`) or a stage silently
            # installed nothing.
            if [ "$szmb" -lt 1200 ]; then
                add "artefact size plausible" "$WARN" "${szmb} MB compressed" \
                    "A finished FPMS-OS image compresses to roughly 2-3 GB. A much
smaller one usually means the rootfs never actually grew, or a stage installed
nothing and said so only in a line nobody read. Check parted -s <img> print free
and the stage 10 / stage 40 output before trusting anything below."
            else
                add "artefact size plausible" "$PASS" "${szmb} MB compressed"
            fi
            ;;
        *)
            add "artefact size plausible" "$PASS" "${szmb} MB uncompressed"
            ;;
    esac

    if [ "$OPT_NOSUM" = 1 ]; then
        add "sha256 sidecar" "$SKIP" "--no-checksum"
        return 0
    fi
    if [ ! -f "${IMG}.sha256" ]; then
        add "sha256 sidecar" "$WARN" "no ${IMG##*/}.sha256 beside the image" \
            "finalise() writes one. Its absence means either an interrupted
build or a hand-finished image, and a hand-finished image is not a clean build.
Say so wherever it goes."
        return 0
    fi
    if ! command -v sha256sum >/dev/null 2>&1; then
        add "sha256 sidecar" "$SKIP" "sha256sum not installed"
        return 0
    fi
    sumout="$(cd "$(dirname "$IMG")" && sha256sum -c "${IMG}.sha256" 2>&1 || true)"
    case "$sumout" in
        *": OK"*)
            add "sha256 sidecar" "$PASS" "matches" ;;
        *)
            add "sha256 sidecar" "$FAIL" "sha256sum -c did not report OK" \
                "The artefact does not match its own sidecar. Either the copy is
truncated or the file was written twice. Do not flash it and do not spend
fifteen hours investigating what is inside it. Output was: ${sumout}"
            ;;
    esac
    return 0
}

# --------------------------------------------------------------------------
# 1. Get a plain .img we can attach
# --------------------------------------------------------------------------
prepare_image() {
    local dir="" avail_kb=0 need_kb=0

    case "$IMG" in
        *.xz)
            if ! command -v xz >/dev/null 2>&1; then
                die "the image is .xz and xz is not installed"
            fi
            dir="${OPT_TMPDIR:-$(dirname "$IMG")}"
            if [ ! -d "$dir" ]; then
                die "--tmpdir $dir does not exist"
            fi
            WORKIMG="$dir/$(basename "${IMG%.xz}")"
            if [ -f "$WORKIMG" ]; then
                note "reusing the already-decompressed $WORKIMG"
                WORKIMG_OWNED=0
            else
                # The decompressed image is 10-11 GB. Running out of space
                # halfway leaves a truncated .img that mounts, reads short, and
                # produces a page of confident FAILs about missing files that
                # are actually present in the artefact.
                avail_kb="$(df -Pk "$dir" 2>/dev/null | awk 'NR==2 {print $4}' || true)"
                need_kb=12000000
                if [ -n "$avail_kb" ] && [ "$avail_kb" -lt "$need_kb" ]; then
                    die "only $(( avail_kb / 1024 )) MB free in $dir.
The decompressed image is 10-11 GB. Use --tmpdir on a bigger filesystem; a
short write here produces a truncated image and a page of FAILs about files
that are really there."
                fi
                note "decompressing to $WORKIMG (10-11 GB, a few minutes)"
                if ! xz -dc "$IMG" > "$WORKIMG"; then
                    rm -f "$WORKIMG"
                    die "xz could not decompress $IMG - the container itself is bad"
                fi
                WORKIMG_OWNED=1
            fi
            ;;
        *)
            WORKIMG="$IMG"
            WORKIMG_OWNED=0
            ;;
    esac
    return 0
}

part_nodes() {
    local n
    for n in "${PART_PREFIX}"*; do
        if [ -b "$n" ]; then printf '%s\n' "$n"; fi
    done
    # Explicit: "no partitions yet" is the NORMAL case on the first probe and
    # must not look like an error to `set -e`.
    return 0
}

settle_partitions() {
    local i n
    PART_PREFIX="${LOOPDEV}p"
    for i in $(seq 1 20); do
        udevadm settle --timeout=2 >/dev/null 2>&1 || true
        n="$(part_nodes 2>/dev/null | wc -l || true)"
        if [ -n "$n" ] && [ "$n" -ge 1 ]; then
            note "partition nodes: $n (after $i probe(s))"
            return 0
        fi
        if [ "$i" = 4 ]; then
            note "no partition nodes yet - forcing with partx -a"
            partx -a "$LOOPDEV" >/dev/null 2>&1 || true
        fi
        if [ "$i" = 8 ] && command -v kpartx >/dev/null 2>&1; then
            note "still nothing - falling back to kpartx"
            kpartx -as "$LOOPDEV" >/dev/null 2>&1 || true
            PART_PREFIX="/dev/mapper/$(basename "$LOOPDEV")p"
            KPARTX_USED=1
        fi
        sleep 1
    done
    die "no partition device nodes appeared for $LOOPDEV.
On WSL2 this usually means the kernel has no loop partition support, or udev is
not running. Check: losetup -l ; ls -l /dev/loop* ; lsblk"
}

# Detect the rootfs by what it IS, not by its number. On a Rockchip table the
# loader/uboot/trust entries appear AHEAD of the filesystems, so "p2" is a
# guess, and guessing wrong here means mounting the FAT boot partition as the
# rootfs and reporting that the entire payload is missing.
detect_partitions() {
    local node type label size best=0 label_locked=0

    note "partition table: $(blkid -o value -s PTTYPE "$LOOPDEV" 2>/dev/null || echo '?')"
    while read -r node; do
        if [ -z "$node" ]; then continue; fi
        type="$(blkid -o value -s TYPE  "$node" 2>/dev/null || true)"
        label="$(blkid -o value -s LABEL "$node" 2>/dev/null || true)"
        size="$(blockdev --getsize64 "$node" 2>/dev/null || echo 0)"
        note "$(printf '%-24s %-8s %-16s %6s MB' \
                "$node" "${type:--}" "${label:--}" "$(( size / 1048576 ))")"
        case "$type" in
            ext2|ext3|ext4)
                case "$label" in
                    writable|rootfs|cloudimg-rootfs|ROOTFS)
                        ROOTPART="$node"; label_locked=1 ;;
                    *)
                        if [ "$label_locked" = 0 ] && [ "$size" -gt "$best" ]; then
                            ROOTPART="$node"; best="$size"
                        fi ;;
                esac
                ;;
            vfat|msdos|fat16|fat32)
                if [ -z "$BOOTPART" ]; then BOOTPART="$node"; fi
                ;;
        esac
    done < <(part_nodes)
    return 0
}

attach_and_mount() {
    # -r: the kernel refuses writes to this loop device at the block layer, so
    # nothing below - not a stray mount option, not a journal replay - can
    # modify the artefact.
    LOOPDEV="$(losetup --find --show --partscan --read-only "$WORKIMG")" \
        || die "losetup failed on $WORKIMG"
    note "loop device: $LOOPDEV (read-only)"

    settle_partitions
    detect_partitions

    if [ -z "$ROOTPART" ]; then
        die "could not find an ext2/3/4 rootfs on $LOOPDEV.
The partitions found are listed above. If they are all empty or all vfat then
this is not an FPMS-OS image, or the partition scan handed back stale nodes
from a previous build. Try: losetup -D ; ls /dev/loop*"
    fi

    MNTBASE="$(mktemp -d -t fpms-verify.XXXXXX)"
    mkdir -p "$MNTBASE/root" "$MNTBASE/boot"

    # `noload` is the fallback: an image whose journal is not clean cannot be
    # mounted ro from a read-only device without it, and replaying the journal
    # is exactly the write we have promised not to make.
    if mount -o ro "$ROOTPART" "$MNTBASE/root" 2>/dev/null; then
        R="$MNTBASE/root"
    elif mount -o ro,noload "$ROOTPART" "$MNTBASE/root" 2>/dev/null; then
        R="$MNTBASE/root"
        note "mounted with noload - the journal is not clean"
    else
        die "cannot mount $ROOTPART read-only"
    fi

    if [ ! -d "$R/etc" ] || [ ! -d "$R/usr/bin" ]; then
        die "$ROOTPART mounted but has no /etc or /usr/bin - this is the wrong
partition, or the image is truncated. Re-check: blkid ${PART_PREFIX}*"
    fi

    if [ -n "$BOOTPART" ]; then
        if mount -o ro "$BOOTPART" "$MNTBASE/boot" 2>/dev/null; then
            B="$MNTBASE/boot"
        else
            note "WARNING: FAT partition $BOOTPART would not mount"
        fi
    fi
    return 0
}

# --------------------------------------------------------------------------
# 2. Units. Not "the file exists" - the SYMLINK.
# --------------------------------------------------------------------------
check_units() {
    local etc="$R/etc/systemd/system"
    local vendor="$R/usr/lib/systemd/system"
    local lib="$R/lib/systemd/system"
    local u p found links target real
    local missing_unit="" missing_link="" dangling=""

    for u in "${BOOT_UNITS[@]}"; do
        found=""
        for p in "$etc/$u" "$vendor/$u" "$lib/$u"; do
            if [ -f "$p" ] && [ ! -L "$p" ]; then found="$p"; break; fi
        done
        if [ -z "$found" ]; then
            missing_unit="$missing_unit $u"
            continue
        fi

        # `systemctl enable` is nothing but these symlinks. On an image nobody
        # has booted, they are the entire evidence that the unit will start.
        links=0
        for target in "$etc"/*.wants/"$u" "$etc"/*.requires/"$u" \
                      "$vendor"/*.wants/"$u" "$lib"/*.wants/"$u"; do
            if [ -L "$target" ]; then
                links=$(( links + 1 ))
                real="$(resolve_in_image "$target")"
                if [ ! -e "$real" ]; then
                    dangling="$dangling $u"
                fi
            fi
        done
        if [ "$links" -lt 1 ]; then
            missing_link="$missing_link $u"
        fi
    done

    if [ -n "$missing_unit" ]; then
        add "boot units shipped" "$FAIL" "not in the image:$missing_unit" \
            "A unit named in stage 60's BOOT_UNITS is not on the image at all.
Either the overlay copy failed or the name is a typo, and the build reported
success either way."
    else
        add "boot units shipped" "$PASS" "${#BOOT_UNITS[@]}/${#BOOT_UNITS[@]} present"
    fi

    if [ -n "$missing_link" ]; then
        add "boot units enabled" "$FAIL" "no .wants symlink:$missing_link" \
            "THIS IS THE ONE THAT LOOKS FINE. A unit with no .wants symlink is
not enabled, and an image that is not enabled comes up with the unit silently
absent on the very first boot - there is no earlier boot for it to have been
running from. fpms-missions was found in exactly this state on the old Pi.
The remedy is in the build, not on the card: stage 60 asserts links>=1 per unit
and should have failed. Rerun it, or re-enter the image and run
systemctl enable <unit> under SYSTEMD_OFFLINE=1."
    else
        add "boot units enabled" "$PASS" "all ${#BOOT_UNITS[@]} have a .wants symlink"
    fi

    if [ -n "$dangling" ]; then
        add "no dangling .wants links" "$FAIL" "dangling:$dangling" \
            "The symlink exists but points at nothing IN THE IMAGE. systemd
logs it at every boot and starts nothing. Note this was resolved against the
image root, not this host's - a check that forgets to do that reports whatever
happens to be installed on the build machine."
    else
        add "no dangling .wants links" "$PASS" "every link resolves inside the image"
    fi
    return 0
}

check_masked() {
    local etc="$R/etc/systemd/system"
    local vendor="$R/usr/lib/systemd/system"
    local u tgt bad="" unshipped="" stray=""

    for u in "${MUST_BE_MASKED[@]}"; do
        tgt=""
        if [ -L "$etc/$u" ]; then
            tgt="$(readlink "$etc/$u" 2>/dev/null || true)"
        fi
        if [ "$tgt" != "/dev/null" ]; then
            if [ -f "$etc/$u" ]; then
                bad="$bad $u=regular-file"
            elif [ -n "$tgt" ]; then
                bad="$bad $u=link-to-$tgt"
            else
                bad="$bad $u=absent"
            fi
        fi
        # Shipped so it CAN be masked. A mask over a unit that does not exist
        # is a weaker guarantee: a restore from backup or a copy from the old
        # Pi puts an unmasked second writer back on the robot.
        if [ ! -f "$vendor/$u" ]; then
            unshipped="$unshipped $u"
        fi
        # `mask` does not remove .wants entries, and a leftover one is both
        # noise in every boot log and evidence that the unit was once enabled.
        local w
        for w in "$etc"/*.wants/"$u" "$etc"/*.requires/"$u"; do
            if [ -L "$w" ]; then stray="$stray $u"; fi
        done
    done

    if [ -n "$bad" ]; then
        add "second /cmd_vel writers masked" "$FAIL" "$bad" \
            "'disabled' is NOT 'masked'. Disable only drops the WantedBy
symlink; anything that pulls the unit in by name still starts it, and two
writers on /cmd_vel is not a race the mission executor can win or even reliably
detect in time. A mask IS a symlink at /etc/systemd/system/<unit> -> /dev/null,
and that is what this checks for - byte for byte, not 'systemctl says
disabled'."
    else
        add "second /cmd_vel writers masked" "$PASS" \
            "${#MUST_BE_MASKED[@]} symlinks to /dev/null in /etc/systemd/system"
    fi

    if [ -n "$unshipped" ]; then
        add "masked units still shipped" "$WARN" "no vendor copy:$unshipped" \
            "The mask holds, but the unit body is gone from
/usr/lib/systemd/system. Masking a unit that does not exist is not the same
guarantee - restoring one from a backup would put an unmasked /cmd_vel writer
back on the robot with nothing standing in its way."
    else
        add "masked units still shipped" "$PASS" "vendor copies present"
    fi

    if [ -n "$stray" ]; then
        add "no .wants on masked units" "$WARN" "leftover links:$stray"
    else
        add "no .wants on masked units" "$PASS" "none"
    fi
    return 0
}

# --------------------------------------------------------------------------
# 3. Payload, and the CRLF that takes down the drive link
# --------------------------------------------------------------------------
find_exe() {   # find_exe <name> -> path inside the image, or ""
    local n="$1" d
    for d in usr/local/bin usr/local/sbin usr/bin usr/sbin; do
        if [ -f "$R/$d/$n" ]; then printf '%s' "$R/$d/$n"; return 0; fi
    done
    return 0
}

check_payload() {
    local f p missing="" notexec="" crlf="" noshebang=""
    local home="$R/home/ubuntu"

    if [ ! -d "$home" ]; then
        add "rover payload present" "$FAIL" "/home/ubuntu does not exist" \
            "Stage 30 never ran, or the user was never created. Nothing else
below about the payload means anything."
        return 0
    fi

    for f in "${PAYLOAD_PY[@]}"; do
        if [ ! -f "$home/$f" ]; then
            missing="$missing $f"
        elif [ ! -x "$home/$f" ]; then
            notexec="$notexec $f"
        fi
    done
    if [ -n "$missing" ]; then
        add "rover payload present" "$FAIL" "missing from /home/ubuntu:$missing" \
            "Stage 30 installs these by name and reports MISSING to stderr, in
the middle of a forty-minute log. The units ExecStart them by absolute path."
    else
        add "rover payload present" "$PASS" \
            "${#PAYLOAD_PY[@]} rover .py files in /home/ubuntu"
    fi
    if [ -n "$notexec" ]; then
        add "rover payload executable" "$FAIL" "not +x:$notexec" \
            "Stage 30 installs these 0755."
    else
        add "rover payload executable" "$PASS" "all 0755"
    fi

    local opt_missing=""
    for f in "${PAYLOAD_PY_OPTIONAL[@]}"; do
        if [ ! -f "$home/$f" ]; then opt_missing="$opt_missing $f"; fi
    done
    if [ -n "$opt_missing" ]; then
        add "optional payload" "$WARN" "absent:$opt_missing" \
            "Installed with || true by stage 30, and their units are masked, so
absence is survivable. Noted because their absence is also how you find out a
staging directory was incomplete."
    else
        add "optional payload" "$PASS" "present"
    fi

    missing=""; notexec=""
    for f in "${PAYLOAD_EXE[@]}"; do
        p="$(find_exe "$f")"
        if [ -z "$p" ]; then
            missing="$missing $f"
        elif [ ! -x "$p" ]; then
            notexec="$notexec $f"
        fi
    done
    if [ -n "$missing" ]; then
        add "installed scripts present" "$FAIL" "not found:$missing" \
            "fpms-wait-net, fpms-wifi-ps-hold and fpms-uros-release-reset
existed in NO REPOSITORY for months while three units named fpms-wait-net as an
ExecStartPre with no '-' prefix. Two units could not start at all, and the only
symptom was a missing LiDAR and a missing TF tree. Searched
/usr/local/bin, /usr/local/sbin, /usr/bin and /usr/sbin."
    else
        add "installed scripts present" "$PASS" \
            "${#PAYLOAD_EXE[@]} scripts in /usr/local/{bin,sbin}"
    fi
    if [ -n "$notexec" ]; then
        add "installed scripts executable" "$FAIL" "not +x:$notexec" \
            "ExecStart= on a non-executable file is a start failure, and
Restart=always turns it into a crash loop that fills the journal."
    else
        add "installed scripts executable" "$PASS" "all +x"
    fi

    # THE CRLF CHECK.
    #
    # A shebang ending "\r" fails at exec with
    #     /usr/bin/env: bad interpreter: No such file or directory
    # which is spectacularly misleading: /usr/bin/env plainly exists, and the
    # file looks perfect in every editor. .gitattributes forces LF for exactly
    # this reason, but .gitattributes only governs what git checks out - it
    # cannot govern a file staged by hand, copied off the old Pi, or written by
    # a stage that used a Windows-authored heredoc.
    for f in "${PAYLOAD_EXE[@]}"; do
        p="$(find_exe "$f")"
        if [ -z "$p" ]; then continue; fi
        if ! has_shebang "$p"; then
            noshebang="$noshebang $f"
            continue
        fi
        if shebang_is_crlf "$p"; then crlf="$crlf $f"; fi
    done
    for f in "${PAYLOAD_PY[@]}"; do
        if [ ! -f "$home/$f" ]; then continue; fi
        if shebang_is_crlf "$home/$f"; then crlf="$crlf $f"; fi
    done

    if [ -n "$crlf" ]; then
        add "shebangs are LF, not CRLF" "$FAIL" "carriage return in:$crlf" \
            "A \\r on the shebang line fails at exec with '/usr/bin/env: bad
interpreter: No such file or directory'. /usr/bin/env exists; the interpreter
name it was handed is 'bash\\r'. On fpms-wait-net this takes down fpms-lidar-ros
and fpms-tf - the LiDAR and the whole TF tree - and on fpms-uros-supervisor it
takes down the drive link. Rebuild after confirming .gitattributes applied:
git add --renormalize . and check the staged blobs, not the worktree."
    else
        add "shebangs are LF, not CRLF" "$PASS" "no carriage returns"
    fi
    if [ -n "$noshebang" ]; then
        add "scripts have a shebang" "$WARN" "no #! on:$noshebang" \
            "May be a compiled binary, which is fine. Worth a look."
    else
        add "scripts have a shebang" "$PASS" "all start with #!"
    fi

    missing=""
    for f in "${PAYLOAD_TREES[@]}"; do
        if [ ! -f "$home/$f" ]; then missing="$missing $f"; fi
    done
    if [ -n "$missing" ]; then
        add "nav2 and slam payload" "$FAIL" "missing:$missing" \
            "fpms-tf.service has referenced ~/nav2/fpms_tf.launch.py from the
beginning and for months NOTHING EVER DEPLOYED IT. A missing nav2/ means no TF
tree on every boot; a partial copy is invisible to a directory check, which is
why these are named one at a time."
    else
        add "nav2 and slam payload" "$PASS" "${#PAYLOAD_TREES[@]} files under ~/nav2 and ~/slam"
    fi
    return 0
}

# --------------------------------------------------------------------------
# 4. micro-ROS. The one that decides whether the rover can move at all.
# --------------------------------------------------------------------------
check_uros() {
    local ws="$R/home/ubuntu/uros_ws"
    local setup="$ws/install/setup.bash"
    local exe="" mach="" sz=""

    if [ ! -f "$setup" ]; then
        add "uros_ws setup.bash" "$FAIL" "$ws/install/setup.bash is missing" \
            "micro-ros-agent.service sources this under set -e. Without it the
wrapper dies instantly, with no useful message, and the drive link never comes
up - so the rover cannot move, while systemd reports the unit as having simply
failed to start. Stage 10 asserts this at line 99; it did not hold here."
        add "micro_ros_agent executable" "$FAIL" "no workspace to look in"
        add "micro_ros_agent is aarch64" "$SKIP" "no binary"
        return 0
    fi
    sz="$(file_size "$setup")"
    if [ "$sz" = "-" ] || [ "$sz" -lt 16 ]; then
        add "uros_ws setup.bash" "$FAIL" "present but ${sz} bytes" \
            "An empty setup.bash sources cleanly and defines nothing, so
'ros2 run micro_ros_agent' then fails with a package-not-found that looks like
a completely different bug."
    else
        add "uros_ws setup.bash" "$PASS" "$sz bytes"
    fi

    exe="$(find "$ws/install" -type f -name micro_ros_agent -perm -u+x -print 2>/dev/null || true)"
    exe="${exe%%$'\n'*}"
    if [ -z "$exe" ]; then
        add "micro_ros_agent executable" "$FAIL" \
            "nothing named micro_ros_agent under $ws/install" \
            "fpms-uros-agent-run execs 'ros2 run micro_ros_agent
micro_ros_agent'. apt ships micro-ros-msgs and micro-ros-diagnostic-* but NOT
the agent, so if stage 10's multi-hour colcon build produced nothing, there is
no agent anywhere on this image and no amount of restarting will find one."
        add "micro_ros_agent is aarch64" "$SKIP" "no binary"
        return 0
    fi
    add "micro_ros_agent executable" "$PASS" "${exe#"$R"} ($(file_size "$exe") bytes)"

    # THE CROSS-BUILD CHECK.
    #
    # This is a cross build, and a silently-x86 binary is the nightmare case:
    # it compiles, it installs, it is the right size, it passes every existence
    # check ever written, and on the board it is "cannot execute binary file".
    # Fifteen hours to find out.
    mach="$(elf_machine "$exe")"
    case "$mach" in
        aarch64)
            add "micro_ros_agent is aarch64" "$PASS" "ELF aarch64" ;;
        not-elf)
            add "micro_ros_agent is aarch64" "$WARN" "not an ELF file" \
                "Probably a shell wrapper colcon generated. Look for the real
binary under install/micro_ros_agent/lib/." ;;
        *)
            add "micro_ros_agent is aarch64" "$FAIL" "ELF machine is $mach" \
                "THE NIGHTMARE CASE. A cross build that silently produced a
host-architecture binary. It is the right name, the right size, in the right
place, and on the Orange Pi it is 'cannot execute binary file: Exec format
error'. The whole drive path depends on it. Check that stage 10 ran under
qemu-aarch64-static and that binfmt_misc was registered on the build host -
without it the chroot runs the HOST's compiler and nothing complains." ;;
    esac

    if [ -f "$R/opt/ros/humble/setup.bash" ]; then
        add "ROS Humble installed" "$PASS" "/opt/ros/humble"
    else
        add "ROS Humble installed" "$FAIL" "/opt/ros/humble/setup.bash missing" \
            "Every ROS unit begins 'source /opt/ros/humble/setup.bash'. Under
bash -lc that is a hard failure and every one of them crash-loops."
    fi
    return 0
}

# --------------------------------------------------------------------------
# 5. Configuration
# --------------------------------------------------------------------------
check_config() {
    local cfg="$R/etc/fpms/config.env"
    local mode="" body="" dds="$R/etc/fpms/fastdds_udp_only.xml"
    local out="" rb="$R/etc/fpms/rosbridge_params.yaml"
    local sud="$R/etc/sudoers.d/fpms-cored"
    local d f missing=""

    if [ ! -f "$cfg" ]; then
        add "config.env present" "$FAIL" "/etc/fpms/config.env missing" \
            "Every service swallows this and falls back to a broker at
192.168.137.1 with no credentials, then retries forever. Nothing crashes and
the dashboard is silently empty."
    else
        mode="$(file_mode "$cfg")"
        if [ "$mode" = "640" ]; then
            add "config.env present" "$PASS" "mode 0640"
        else
            add "config.env present" "$FAIL" "mode 0$mode, expected 0640" \
                "It holds the broker password after first boot. 0644 makes that
password world-readable on every board flashed from this image."
        fi

        body="$(read_text "$cfg")"
        # The same placeholder is also in the mosquitto bridge config's
        # remote_password. Both are replaced by fpms-firstboot, on the board.
        local secrets="$body"
        secrets="$secrets$(read_text "$R/etc/mosquitto/conf.d/fpms.conf")"
        case "$secrets" in
            *"$PLACEHOLDER"*)
                add "firstboot placeholder intact" "$PASS" \
                    "FPMS_MQTT_PASS is still the placeholder" ;;
            *)
                add "firstboot placeholder intact" "$FAIL" \
                    "FPMS_MQTT_PASS has been replaced" \
                    "A SHIPPED IMAGE MUST CARRY THE PLACEHOLDER. If it has been
replaced then either a real credential is baked into an artefact that gets
copied around and flashed onto every board, or firstboot ran during the build.
Either way this image must not be distributed. The placeholder is
${PLACEHOLDER} and fpms-firstboot is what replaces it, on the board, once." ;;
        esac

        if file_has_cr "$cfg"; then
            add "config.env has no CR" "$FAIL" "carriage returns in config.env" \
                "EnvironmentFile= keeps the \\r. Every value then ends in an
invisible character: FPMS_LIDAR_PORT points at a device that does not exist,
FPMS_TF_OFFSETS_MEASURED=1\\r is not '1' so fpms-tf refuses to start, and the
comparison that rejects it prints two strings that look identical."
        else
            add "config.env has no CR" "$PASS" "LF only"
        fi
    fi

    # --- DDS ---------------------------------------------------------------
    if [ ! -f "$dds" ]; then
        add "DDS profile present" "$FAIL" "/etc/fpms/fastdds_udp_only.xml missing" \
            "Without it Fast DDS prefers shared memory, whose segments do not
survive across process eras: discovery succeeds and NO DATA FLOWS, with no
error anywhere. See docs/DDS.md."
        add "DDS useBuiltinTransports=false" "$SKIP" "no profile"
    elif [ "$HAVE_PY" = 0 ]; then
        add "DDS profile present" "$WARN" "present; python3 absent so not parsed"
        add "DDS useBuiltinTransports=false" "$SKIP" "python3 not installed"
    else
        # Parse it rather than grep it: the file's own comments discuss the
        # element at length, and Fast DDS SILENTLY IGNORES a malformed profile
        # and falls back to defaults - reintroducing the exact bug the file
        # exists to fix, with no message.
        out="$(python3 -c 'import sys,xml.etree.ElementTree as ET
t = ET.parse(sys.argv[1])
v = [ (e.text or "").strip().lower() for e in t.iter()
      if e.tag.split("}")[-1] == "useBuiltinTransports" ]
print(",".join(v) if v else "-")' "$dds" 2>&1 || true)"
        case "$out" in
            false|false,*|*,false)
                add "DDS profile present" "$PASS" "parses"
                add "DDS useBuiltinTransports=false" "$PASS" "false" ;;
            -)
                add "DDS profile present" "$PASS" "parses"
                add "DDS useBuiltinTransports=false" "$FAIL" \
                    "the element is not in the profile" \
                    "This one line is what actually stops Fast DDS using shared
memory. Without it the profile parses, loads, and changes nothing." ;;
            *Error*|*error*|*Traceback*)
                add "DDS profile present" "$FAIL" "does not parse: $out" \
                    "Fast DDS silently ignores a malformed profile and falls
back to defaults. There is no warning at any log level."
                add "DDS useBuiltinTransports=false" "$SKIP" "profile does not parse" ;;
            *)
                add "DDS profile present" "$PASS" "parses"
                add "DDS useBuiltinTransports=false" "$FAIL" "value is '$out'" ;;
        esac
    fi

    # --- rosbridge ---------------------------------------------------------
    if [ ! -f "$rb" ]; then
        add "rosbridge whitelist" "$FAIL" "/etc/fpms/rosbridge_params.yaml missing" \
            "rosbridge then starts with NO topic whitelist and a browser can
reach the drive topics."
    else
        body="$(strip_hash_comments "$rb")"
        case "$body" in
            *cmd_vel*|*cmd_duty*)
                add "rosbridge whitelist" "$FAIL" "a drive topic is in the globs" \
                    "/cmd_vel and /cmd_duty must be ABSENT so that a stray click
or a stale browser tab cannot turn a wheel. Checked with comments stripped -
the file legitimately explains in prose why they are not there." ;;
            *)
                add "rosbridge whitelist" "$PASS" "no drive topics exposed" ;;
        esac
    fi

    # --- sudoers -----------------------------------------------------------
    if [ ! -f "$sud" ]; then
        add "STOP escalation sudoers rule" "$FAIL" "/etc/sudoers.d/fpms-cored missing" \
            "The last-resort stop becomes a password prompt on a moving rover."
    else
        mode="$(file_mode "$sud")"
        body="$(strip_hash_comments "$sud")"
        case "$body" in
            *"kill -s SIGTERM fpms-missions.service"*)
                if [ "$mode" = "440" ]; then
                    add "STOP escalation sudoers rule" "$PASS" \
                        "mode 0440, argv names fpms-missions.service"
                else
                    add "STOP escalation sudoers rule" "$FAIL" \
                        "argv correct but mode is 0$mode, expected 0440" \
                        "sudo IGNORES a file in /etc/sudoers.d whose mode is
group- or world-writable. It does not complain; the rule simply is not there,
and the failure surfaces as a password prompt on a rover that is still moving."
                fi ;;
            *fpms-missions*)
                add "STOP escalation sudoers rule" "$FAIL" \
                    "names fpms-missions but not the exact argv" \
                    "sudo matches on the LITERAL argument vector.
'fpms-missions' and 'fpms-missions.service' are not the same rule. fpms_cored.py
runs: sudo -n /bin/systemctl kill -s SIGTERM fpms-missions.service. A mismatch
does not error in any visible way - it turns the last-resort stop into a silent
password prompt that nobody is there to answer." ;;
            *)
                add "STOP escalation sudoers rule" "$FAIL" \
                    "the file does not mention fpms-missions at all" ;;
        esac
    fi

    # --- nav2 / slam config, NOT EMPTY -------------------------------------
    #
    # A previous bug created both directories empty. Three units name files in
    # them by absolute path, and `install -d` succeeding is not evidence that
    # anything was put inside.
    for d in nav2 slam; do
        if [ ! -d "$R/etc/fpms/$d" ]; then
            add "/etc/fpms/$d populated" "$FAIL" "the directory does not exist" \
                "Named by absolute path in the fpms-$d unit(s)."
        elif dir_is_empty "$R/etc/fpms/$d"; then
            add "/etc/fpms/$d populated" "$FAIL" "the directory is EMPTY" \
                "This has happened. install -d succeeds, the build reports
success, the directory is there, and it contains nothing. The units name files
inside it by absolute path and ros2 launch dies on the missing params_file."
        else
            add "/etc/fpms/$d populated" "$PASS" "not empty"
        fi
    done

    missing=""
    for f in "${ETC_FPMS_CONFIGS[@]}"; do
        if [ ! -f "$R/etc/fpms/$f" ]; then missing="$missing $f"; fi
    done
    if [ -n "$missing" ]; then
        add "unit-referenced config files" "$FAIL" "missing:$missing" \
            "Each is named by an ExecStart= as params_file:=/etc/fpms/... . A
non-empty directory is not enough; these exact names are what the units ask
for."
    else
        add "unit-referenced config files" "$PASS" \
            "${#ETC_FPMS_CONFIGS[@]} params files present"
    fi
    return 0
}

# --------------------------------------------------------------------------
# 6. The build record
# --------------------------------------------------------------------------
check_versions() {
    local f rc
    for f in versions.json npu-versions.json; do
        if [ ! -f "$R/etc/fpms/$f" ]; then
            add "$f" "$FAIL" "/etc/fpms/$f missing" \
                "This is the only record of what went into the image. Without
it nobody can state the provenance of a card that is already in a rover."
            continue
        fi
        rc=0
        json_parses "$R/etc/fpms/$f" || rc=$?
        if [ "$rc" = 0 ]; then
            add "$f" "$PASS" "parses"
        elif [ "$rc" = 2 ]; then
            add "$f" "$SKIP" "present; python3 absent so not parsed"
        else
            add "$f" "$FAIL" "present but is not valid JSON" \
                "A truncated manifest means the stage that wrote it was killed
part-way, which means whatever it was recording is also incomplete."
        fi
    done

    if [ -f "$R/etc/fpms/ros-versions.txt" ]; then
        add "ros-versions.txt" "$PASS" "present"
    else
        add "ros-versions.txt" "$WARN" "/etc/fpms/ros-versions.txt missing" \
            "Stage 40 writes it. Its absence suggests stage 40 did not finish."
    fi
    return 0
}

# --------------------------------------------------------------------------
# 7. Per-board identity. Everything here must be ABSENT or EMPTY.
# --------------------------------------------------------------------------
check_identity() {
    local mid="$R/etc/machine-id"
    local link="" keys="" seeds="" markers="" f

    if [ ! -f "$mid" ]; then
        add "machine-id cleared" "$FAIL" "/etc/machine-id is not a regular file" \
            "systemd needs it PRESENT and EMPTY. Absent is not the same marker:
an absent file makes systemd fall back to a transient id in /run, and the board
gets a new identity on every boot instead of one stable identity generated
once."
    elif [ -s "$mid" ]; then
        add "machine-id cleared" "$FAIL" "/etc/machine-id is not empty ($(file_size "$mid") bytes)" \
            "Every board flashed from this image would share one machine-id.
That collides journald, DHCP client identifiers and anything keyed on the id,
and the symptom on the second rover is 'the dashboard shows one rover twice'."
    else
        add "machine-id cleared" "$PASS" "present and empty, mode 0$(file_mode "$mid")"
    fi

    link="$(readlink "$R/var/lib/dbus/machine-id" 2>/dev/null || true)"
    if [ "$link" = "/etc/machine-id" ]; then
        add "dbus machine-id symlinked" "$PASS" "-> /etc/machine-id"
    else
        add "dbus machine-id symlinked" "$WARN" "is '${link:-not a symlink}'" \
            "If it is a regular file it holds a baked id of its own, and dbus
and systemd then disagree about who this board is."
    fi

    keys="$(find "$R/etc/ssh" -maxdepth 1 -name 'ssh_host_*' -print 2>/dev/null || true)"
    if [ -n "$keys" ]; then
        add "no SSH host keys" "$FAIL" "host keys are baked into the image" \
            "Every board flashed from this image would present the SAME host
key, so any one of them can impersonate any other and the operator's
known_hosts cannot tell them apart. They must be regenerated on first boot."
    else
        add "no SSH host keys" "$PASS" "none"
    fi

    seeds=""
    for f in var/lib/systemd/random-seed var/lib/urandom/random-seed; do
        if [ -e "$R/$f" ]; then seeds="$seeds /$f"; fi
    done
    if [ -n "$seeds" ]; then
        add "no random seed" "$FAIL" "shipped:$seeds" \
            "A seed shipped in an image is the SAME seed on every board, which
is the opposite of what a seed is for."
    else
        add "no random seed" "$PASS" "none"
    fi

    markers=""
    for f in var/lib/fpms/.firstboot-done var/lib/fpms/firstboot.json; do
        if [ -e "$R/$f" ]; then markers="$markers /$f"; fi
    done
    if [ -n "$markers" ]; then
        add "firstboot not yet run" "$FAIL" "marker present:$markers" \
            "fpms-firstboot skips everything when the marker is there. The
board would boot with the placeholder broker password never replaced, the
rootfs never grown, and no error - just an empty dashboard."
    else
        add "firstboot not yet run" "$PASS" "no marker"
    fi
    return 0
}

# --------------------------------------------------------------------------
# 8. Build hygiene - what must NOT be in the artefact, and what is known absent
# --------------------------------------------------------------------------
check_hygiene() {
    local f left="" absent=""

    for f in "${BUILD_LEFTOVERS[@]}"; do
        if [ -e "$R/$f" ]; then left="$left /$f"; fi
    done
    if [ -n "$left" ]; then
        add "no build scaffolding left" "$FAIL" "still in the image:$left" \
            "policy-rc.d returns 101 for EVERYTHING, so on the board every
apt install silently declines to start the service it just installed, forever,
with no error. qemu-aarch64-static is a foreign binary shipped to customers.
uros_ws/build and uros_ws/log are gigabytes of object files. A leftover
.versions.json.new means stage 90 was killed mid-write. finalise() removes all
of these; if they are here, it did not finish."
    else
        add "no build scaffolding left" "$PASS" "policy-rc.d, qemu, colcon build trees all gone"
    fi

    if [ -e "$R/etc/fpms/ALLOW_NPU_MISSING" ]; then
        add "no NPU escape hatch shipped" "$FAIL" "/etc/fpms/ALLOW_NPU_MISSING present" \
            "That marker tells stage 25 to accept an image with no working NPU
runtime, and stage 50 removes nothing from /etc/fpms - so it SHIPS. Whoever
flashes this gets a rover that streams video and detects nothing, and the one
file that would have said so was disarmed at build time."
    else
        add "no NPU escape hatch shipped" "$PASS" "absent"
    fi

    # The overlay directory of the build itself is deliberately KEPT, because
    # fpms-doctor and the DDS notes are referenced by absolute path from unit
    # comments and from fpms-selftest's remedies.
    if [ -d "$R/opt/fpms-os/docs" ]; then
        add "in-image documentation" "$PASS" "/opt/fpms-os/docs"
    else
        add "in-image documentation" "$WARN" "/opt/fpms-os/docs missing" \
            "fpms-selftest's remedies point operators at /opt/fpms-os/docs/DDS.md
by absolute path. Without it every remedy is a dead reference on a rover that
may have no network."
    fi

    for f in "${KNOWN_ABSENT[@]}"; do
        if [ ! -e "$R/$f" ]; then absent="$absent /$f"; fi
    done
    if [ -n "$absent" ]; then
        add "known-absent payload" "$WARN" "not in the image:$absent" \
            "THIS IS EXPECTED AND IT IS STILL A GAP. The .rknn model and the
operator console exist only on the old Pi and are in no repository, so no build
can produce them. fpms-console.service is nonetheless ENABLED and will fail on
every boot, and the agent will log 'NPU unavailable; streaming without
detection' and carry on. Copy them across before relying on detection or the
console: scp -r ubuntu@<old-pi>:~/yolo ubuntu@<board>:~/"
    else
        add "known-absent payload" "$PASS" "the old-Pi-only files are present"
    fi

    # config.env is 0640 because it will hold the broker password. The mosquitto
    # bridge config holds the same secret and is 0600.
    if [ -f "$R/etc/mosquitto/conf.d/fpms.conf" ]; then
        local m
        m="$(file_mode "$R/etc/mosquitto/conf.d/fpms.conf")"
        if [ "$m" = "600" ]; then
            add "mosquitto bridge config mode" "$PASS" "0600"
        else
            add "mosquitto bridge config mode" "$WARN" "mode 0$m, expected 0600" \
                "It carries the same broker credential config.env does."
        fi
    else
        add "mosquitto bridge config mode" "$WARN" "/etc/mosquitto/conf.d/fpms.conf missing"
    fi

    if [ -f "$R/etc/sudoers.d/90-fpms-user" ]; then
        local m2
        m2="$(file_mode "$R/etc/sudoers.d/90-fpms-user")"
        if [ "$m2" = "440" ]; then
            add "deploy sudoers mode" "$PASS" "0440"
        else
            add "deploy sudoers mode" "$FAIL" "mode 0$m2, expected 0440" \
                "sudo IGNORES a group- or world-writable file in
/etc/sudoers.d, silently. The deploy tooling then prompts for a password
nobody is there to type."
        fi
    else
        add "deploy sudoers mode" "$WARN" "/etc/sudoers.d/90-fpms-user missing" \
            "The deploy tooling requires passwordless sudo for ubuntu."
    fi
    return 0
}

# --------------------------------------------------------------------------
# 9. The boot partition - the only part of the image Windows can read
# --------------------------------------------------------------------------
check_bootpart() {
    local f missing="" onext4=""

    if [ -z "$B" ]; then
        add "FAT boot partition" "$FAIL" "no mountable FAT partition in the image" \
            "The operator provisions WiFi by editing a file on the partition
that appears when the card is plugged into a Windows machine. Without a FAT
partition there is no such file, and every WiFi instruction in docs/FLASHING.md
refers to something that cannot be reached. A rover that cannot join a network
cannot be reached at all."
        add "operator files on FAT" "$SKIP" "no FAT partition"
        return 0
    fi
    add "FAT boot partition" "$PASS" "$BOOTPART mounted read-only"

    for f in README-FPMS.txt fpms-wifi.conf.example; do
        if [ ! -f "$B/$f" ]; then
            missing="$missing $f"
            if [ -f "$R/boot/$f" ]; then onext4="$onext4 $f"; fi
        fi
    done

    if [ -z "$missing" ]; then
        add "operator files on FAT" "$PASS" "README-FPMS.txt, fpms-wifi.conf.example"
    elif [ -n "$onext4" ]; then
        add "operator files on FAT" "$FAIL" "on the ext4 /boot instead:$onext4" \
            "They exist, and they are in the one place the operator cannot
reach. Windows cannot read ext4, so the card shows only the FAT partition and
these files are invisible. build.sh falls back to the ext4 /boot with a warning
when it cannot mount the FAT partition - that warning was the whole event."
    else
        add "operator files on FAT" "$FAIL" "missing from the FAT partition:$missing" \
            "stage 50 copies overlay/boot/* onto the mounted FAT partition.
Without them the operator has no WiFi template and no instructions, on the only
partition their machine can see."
    fi
    return 0
}

# --------------------------------------------------------------------------
# 10. Python. File inspection only - we cannot run aarch64 here.
# --------------------------------------------------------------------------
find_pkg() {   # find_pkg <relative path under a site/dist-packages dir>
    local hit=""
    hit="$(find "$R/usr/lib/python3" "$R/usr/lib/python3.10" \
                "$R/usr/local/lib" "$R/usr/lib/python3/dist-packages" \
                -maxdepth 6 -path "*/$1" -print 2>/dev/null || true)"
    printf '%s' "${hit%%$'\n'*}"
    return 0
}

check_python() {
    local p="" body="" dist=""

    # paho >= 2.0. Five files import CallbackAPIVersion unguarded and one of
    # them is fpms_cored.py, the STOP authority. Ubuntu 22.04's apt package is
    # 1.6.1 and does not have it, so a rover built against apt has no emergency
    # stop and no telemetry - and nothing anywhere says so.
    #
    # We cannot import it here (aarch64), so we read the module and look for
    # the symbol. That is weaker than an import and it is the strongest thing
    # available without a board.
    p="$(find_pkg "paho/mqtt/client.py")"
    if [ -z "$p" ]; then
        add "paho.mqtt.client present" "$FAIL" "paho/mqtt/client.py not found" \
            "fpms_cored.py, the STOP authority, imports it at module scope.
Without it the unit crash-loops at import and the rover has no emergency stop
and no telemetry."
        add "paho is 2.x" "$SKIP" "module not found"
    else
        add "paho.mqtt.client present" "$PASS" "${p#"$R"}"
        body="$(read_text "$p")"
        case "$body" in
            *CallbackAPIVersion*)
                add "paho is 2.x" "$PASS" "CallbackAPIVersion is defined" ;;
            *)
                add "paho is 2.x" "$FAIL" "no CallbackAPIVersion in client.py" \
                    "This is apt's 1.6.1. Five files import CallbackAPIVersion
unguarded, including the STOP authority. The pip install in stage 20 did not
take - check whether --break-system-packages was needed and swallowed." ;;
        esac
    fi

    # A user-local numpy SHADOWS the system one for every service. This exact
    # thing happened on the old Pi and surfaced far from its cause.
    if [ -d "$R/home/ubuntu/.local/lib/python3.10/site-packages/numpy" ]; then
        add "no shadowing user-local numpy" "$FAIL" \
            "/home/ubuntu/.local/.../site-packages/numpy exists" \
            "A user-local NumPy overrides the system one for every service run
as ubuntu. If it is 2.x it breaks ROS Humble's C extensions with
'np.maximum_sctype was removed', reported from tf_transformations, nowhere near
the cause."
    else
        add "no shadowing user-local numpy" "$PASS" "none"
    fi

    # Read the version off the dist-info directory NAME rather than out of
    # numpy/version.py: the assignment in that file has changed shape between
    # releases ("version = " vs "version: str = "), and a pattern that stops
    # matching turns this check into a silent PASS - which is the failure mode
    # every check in this repository exists to avoid.
    p="$(find_pkg "numpy/version.py")"
    dist="$(find "$R/usr/lib/python3" "$R/usr/lib/python3.10" "$R/usr/local/lib" \
                 -maxdepth 5 -type d -name 'numpy-*.dist-info' -print 2>/dev/null || true)"
    dist="${dist##*/}"
    if [ -z "$p" ] && [ -z "$dist" ]; then
        add "numpy present" "$WARN" "neither numpy/version.py nor a dist-info found" \
            "It may be installed somewhere this search does not reach. rclpy
will not import without it."
    else
        case "$dist" in
            numpy-2.*)
                add "numpy present" "$FAIL" "$dist" \
                    "ROS Humble's C extensions are built against the NumPy 1.x
ABI. 2.x breaks tf_transformations with 'np.maximum_sctype was removed',
reported from a module nowhere near the cause. Pin numpy<2." ;;
            numpy-*)
                add "numpy present" "$PASS" "$dist" ;;
            *)
                add "numpy present" "$WARN" "found at ${p#"$R"}, version unreadable" \
                    "No numpy-*.dist-info directory, so the version could not be
established. 2.x would break every ROS C extension in the image." ;;
        esac
    fi

    p="$(find_pkg "rknnlite/api/__init__.py")"
    if [ -z "$p" ]; then
        add "rknnlite present" "$WARN" "rknnlite not found by this search" \
            "The agent degrades SILENTLY - it logs 'NPU unavailable; streaming
without detection' and carries on, so the rover streams video and detects
nothing while systemd reports every unit active. /etc/fpms/npu-versions.json is
the fuller record; check its status field."
    else
        add "rknnlite present" "$PASS" "${p#"$R"}"
    fi
    return 0
}

# --------------------------------------------------------------------------
# 11. The LiDAR mount calibration tool.
#
# The mount transform is the last unmeasured thing standing between this image
# and a rover that can navigate: every base_link -> laser_frame offset in
# fpms_tf.launch.py is a MEASURE ME placeholder, the unit launches it with none
# of them supplied, and by that file's own arithmetic 1 degree of mount yaw is
# ~10.5 mm of position error inherited IN THE SAME DIRECTION by both the
# obstacle cone guard and the occupancy grid - so it accumulates rather than
# averaging out. fpms-nav2 and both SLAM units refuse to start until
# FPMS_TF_OFFSETS_MEASURED=1, and this tool is the only thing in the image that
# can honestly produce that 1.
#
# An image that shipped without it boots perfectly, reports every unit fine,
# and can never be made to navigate. Nothing else in this file looks at either
# file, so without this section that is a silent pass.
#
# ONE THING HERE IS STRONGER THAN THE REST OF THIS SCRIPT, and it is worth
# being precise about, because the header above says we execute nothing from
# inside the image. We still do not. fpms_scanmatch.py is TEXT, not an aarch64
# binary, so the HOST's python3 can PARSE it - ast.parse builds a tree and runs
# none of it, imports nothing, and writes nothing. That is a genuinely stronger
# claim than the paho and numpy checks above, which can only grep a module for
# a symbol. It is still not an import: numpy is not resolved here, and a
# NameError inside a function body survives this untouched.
# --------------------------------------------------------------------------
CALIB_AST="$(cat <<'PYEOF'
import ast, sys
path = sys.argv[1]
required = sys.argv[2].split()
try:
    src = open(path, "rb").read().decode("utf-8")
except (OSError, UnicodeDecodeError) as e:
    sys.stderr.write("cannot read as UTF-8: %s" % e)
    raise SystemExit(1)
try:
    mod = ast.parse(src, path)
except SyntaxError as e:
    sys.stderr.write("SyntaxError at line %s: %s" % (e.lineno, e.msg))
    raise SystemExit(1)
have = set()
for n in mod.body:
    if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
        have.add(n.name)
gone = [n for n in required if n not in have]
if gone:
    sys.stderr.write("parses, but these top-level functions are gone: %s"
                     % " ".join(gone))
    raise SystemExit(1)
sys.stdout.write("parses, %d top-level functions, all %d required present"
                 % (len(have), len(required)))
PYEOF
)"

check_calibration() {
    local p="" lib="$R/$CALIB_LIB" libdir="$R/usr/local/lib/fpms"
    local out="" mode="" sz="" pth="" hits="" f=""

    # --- the node -----------------------------------------------------------
    p="$(find_exe "$CALIB_BIN")"
    if [ -z "$p" ]; then
        add "calibration tool present" "$FAIL" "$CALIB_BIN not found" \
            "It ships at overlay/usr/local/bin/$CALIB_BIN and stage 50 copies
overlay/usr/local/** into /. Absent means the overlay copy did not run or the
file was never committed. Without it the mount transform cannot be measured,
FPMS_TF_OFFSETS_MEASURED stays 0, and fpms-nav2 and both SLAM units refuse to
start on every boot. Searched /usr/local/bin, /usr/local/sbin, /usr/bin and
/usr/sbin. See docs/CALIBRATION.md."
        add "calibration tool executable" "$SKIP" "tool not found"
        add "calibration tool shebang is LF" "$SKIP" "tool not found"
    else
        add "calibration tool present" "$PASS" "${p#"$R"}"

        if [ ! -x "$p" ]; then
            add "calibration tool executable" "$FAIL" "not +x: ${p#"$R"}" \
                "Stage 50 chmods /usr/local/bin/* to 0755. A tool the operator
cannot run is the same as a tool that is not there, except that it looks fine."
        else
            add "calibration tool executable" "$PASS" "mode $(file_mode "$p")"
        fi

        # THE CARRIAGE RETURN. Same failure as fpms-wait-net and
        # fpms-uros-supervisor, checked the same way, for the same reason: a \r
        # on the shebang line fails at exec as "/usr/bin/env: bad interpreter:
        # No such file or directory", which sends the reader looking for a
        # missing /usr/bin/env that plainly exists. This one is worse than most
        # to diagnose because the operator meets it crouched next to a rover
        # rather than in front of a journal.
        if ! has_shebang "$p"; then
            add "calibration tool shebang is LF" "$FAIL" "no #! on $CALIB_BIN" \
                "It is installed 0755 and run by hand as
/usr/local/bin/$CALIB_BIN. With no interpreter line the shell runs it as a
shell script and it dies on the first python statement."
        elif shebang_is_crlf "$p"; then
            add "calibration tool shebang is LF" "$FAIL" \
                "carriage return on the shebang of $CALIB_BIN" \
                "Fails at exec with '/usr/bin/env: bad interpreter: No such file
or directory'. /usr/bin/env exists; the interpreter name it was handed is
'python3\\r'. Stage 50's CRLF sweep covers /usr/local/bin/fpms-* and should have
caught this, so reaching here means the sweep did not run or the file arrived
after it. Rebuild after git add --renormalize . and check the STAGED blob."
        else
            add "calibration tool shebang is LF" "$PASS" "no carriage return"
        fi
    fi

    # --- the library --------------------------------------------------------
    #
    # Checked for SIZE as well as existence. A zero-byte fpms_scanmatch.py
    # copies, chmods and mounts perfectly; it fails only at import, on the
    # rover.
    if [ ! -f "$lib" ]; then
        add "scanmatch library present" "$FAIL" "/$CALIB_LIB not found" \
            "The node imports fpms_scanmatch at start-up. It ships at
overlay/usr/local/lib/fpms/fpms_scanmatch.py and stage 50 places it. The tool
is two files and it needs both: this one holds every estimator, the node holds
none of them."
        add "scanmatch library parses" "$SKIP" "library not found"
    else
        sz="$(file_size "$lib")"
        if [ "$sz" = "-" ] || [ "$sz" = "0" ]; then
            add "scanmatch library present" "$FAIL" "/$CALIB_LIB is $sz bytes" \
                "A zero-byte module imports without error and defines nothing,
so the node dies on an AttributeError rather than an ImportError - which sends
the reader to the node instead of to the copy that truncated."
            add "scanmatch library parses" "$SKIP" "library is empty"
        else
            add "scanmatch library present" "$PASS" "/$CALIB_LIB ($sz bytes)"

            if [ "$HAVE_PY" = 0 ]; then
                add "scanmatch library parses" "$SKIP" \
                    "no python3 on this host to parse it with"
            elif out="$(python3 -c "$CALIB_AST" "$lib" "${CALIB_FUNCS[*]}" 2>&1)"; then
                add "scanmatch library parses" "$PASS" "$out"
            else
                add "scanmatch library parses" "$FAIL" "$out" \
                    "Either the file is not valid Python, or a function the node
calls has been renamed or removed. Both fail on the rover - the first at
import, the second at the moment the operator asks for a measurement. Stage 30
makes the same check against the STAGED overlay and would have failed the
build, so a failure here means the two trees disagree."
            fi
        fi
    fi

    # --- is the library actually importable? --------------------------------
    #
    # Present is not importable. /usr/local/lib/fpms is a directory no Python
    # has ever heard of, so without a .pth in a site directory the node's
    # `import fpms_scanmatch` raises ImportError no matter how correct both
    # files are. Stage 30 writes that .pth and asserts sys.path in the chroot;
    # this confirms the file survived into the artefact.
    #
    # The search is a find, captured whole. `find ... | grep -q` here would
    # SIGPIPE find, pipefail would report 141, and the verdict would come out
    # backwards - the bug class this repository has been bitten by repeatedly.
    if [ ! -d "$libdir" ]; then
        add "scanmatch library importable" "$FAIL" "/usr/local/lib/fpms is not a directory" \
            "Nothing to put on sys.path. See stage 30."
    else
        hits="$(find "$R/usr/local/lib" "$R/usr/lib/python3" \
                     "$R/usr/lib/python3.10" "$R/usr/lib/python3/dist-packages" \
                     -maxdepth 4 -name '*.pth' -print 2>/dev/null || true)"
        pth=""
        while IFS= read -r f; do
            [ -n "$f" ] || continue
            case "$(read_text "$f")" in
                */usr/local/lib/fpms*) pth="$f"; break ;;
            esac
        done <<<"$hits"

        if [ -n "$pth" ]; then
            add "scanmatch library importable" "$PASS" \
                "${pth#"$R"} puts /usr/local/lib/fpms on sys.path"
        else
            add "scanmatch library importable" "$FAIL" \
                "no .pth names /usr/local/lib/fpms" \
                "The module is on disk and no interpreter can find it.
fpms-calibrate-lidar dies at import with ModuleNotFoundError: fpms_scanmatch.
Stage 30 writes this .pth into python3's own site directory and asserts
sys.path afterwards, so its absence here means stage 30 did not run or the site
directory moved. Workaround on the rover:
PYTHONPATH=/usr/local/lib/fpms /usr/local/bin/fpms-calibrate-lidar"
        fi

        # A world-writable directory on EVERY interpreter's sys.path is a place
        # any process on the rover can drop a module that every other process
        # then imports. The overlay is staged from a Windows filesystem where
        # every directory reads back 0777, so this is a live possibility rather
        # than a theoretical one - stage 30 creates the directory 0755 first and
        # stage 50's DIR_SNAP restores it, and this checks that both worked.
        mode="$(file_mode "$libdir")"
        case "$mode" in
            *[2367])
                add "scanmatch library directory not writable by all" "$FAIL" \
                    "/usr/local/lib/fpms is mode $mode" \
                    "It is on sys.path for every python3 on the rover. Anything
that can write here can inject a module into every ROS node. cp -a stamps the
Windows source directory's 0777 onto the destination; stage 30 pre-creates it
0755 and stage 50's DIR_SNAP block restores the mode - one of those did not
happen." ;;
            -)
                add "scanmatch library directory not writable by all" "$WARN" \
                    "could not stat /usr/local/lib/fpms" ;;
            *)
                add "scanmatch library directory not writable by all" "$PASS" \
                    "mode $mode" ;;
        esac
    fi

    # The tool is useless if nobody knows it exists. docs/ is staged into the
    # image by build.sh's stage_all(), so the procedure travels with the rover
    # rather than living only in a repository the operator does not have at the
    # arena.
    if [ -f "$R/opt/fpms-os/docs/CALIBRATION.md" ]; then
        add "calibration procedure documented" "$PASS" \
            "/opt/fpms-os/docs/CALIBRATION.md"
    else
        add "calibration procedure documented" "$WARN" \
            "/opt/fpms-os/docs/CALIBRATION.md is not in the image" \
            "The operator runs this crouched next to a rover, and the order of
the steps is load-bearing: the mirror check has to come before yaw, because a
mirror is not a rigid transform and yaw measured against a mirrored world is
confidently wrong. Read it from the repository instead."
    fi
    return 0
}

# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------
json_escape() {
    local s="${1:-}"
    s="${s//\\/\\\\}"
    s="${s//\"/\\\"}"
    s="${s//$'\n'/\\n}"
    s="${s//$'\r'/\\r}"
    s="${s//$'\t'/\\t}"
    printf '%s' "$s"
    return 0
}

emit_json() {   # emit_json <nfail> <nwarn>
    local i n first=1 ok="false"
    n="${#R_NAME[@]}"
    if [ "$1" = 0 ]; then ok="true"; fi
    printf '{\n'
    printf '  "tool": "verify_image.sh",\n'
    printf '  "image": "%s",\n' "$(json_escape "$IMG")"
    printf '  "ts": %s,\n' "$(date +%s)"
    printf '  "ok": %s,\n' "$ok"
    printf '  "fail": %s, "warn": %s, "total": %s,\n' "$1" "$2" "$n"
    printf '  "checks": [\n'
    for (( i=0; i<n; i++ )); do
        if [ "$first" = 0 ]; then printf ',\n'; fi
        first=0
        printf '    {"check": "%s", "status": "%s", "detail": "%s", "remedy": "%s"}' \
            "$(json_escape "${R_NAME[$i]}")" \
            "$(json_escape "${R_STATUS[$i]}")" \
            "$(json_escape "${R_DETAIL[$i]}")" \
            "$(json_escape "${R_REMEDY[$i]}")"
    done
    printf '\n  ]\n}\n'
    return 0
}

emit_text() {   # emit_text <nfail> <nwarn>
    local i n chunk k
    n="${#R_NAME[@]}"
    printf '========================================================================\n'
    printf ' FPMS-OS image verification    %s\n' "$(date '+%Y-%m-%d %H:%M:%S')"
    printf ' %s\n' "$(basename "$IMG")"
    printf '========================================================================\n'
    for (( i=0; i<n; i++ )); do
        printf '  [%-4s] %-32s %s\n' \
               "${R_STATUS[$i]}" "${R_NAME[$i]}" "${R_DETAIL[$i]}"
        if [ -n "${R_REMEDY[$i]}" ] \
        && { [ "${R_STATUS[$i]}" = "$FAIL" ] || [ "${R_STATUS[$i]}" = "$WARN" ]; }; then
            k=0
            # `|| [ -n "$chunk" ]` is not decoration. tr has turned the final
            # newline into a space, so fold's last line has no terminator,
            # `read` returns 1 on it, and a plain `while read` DROPS IT - which
            # silently truncated the last line of every remedy in this file.
            while IFS= read -r chunk || [ -n "$chunk" ]; do
                if [ "$k" = 0 ]; then
                    printf '         -> %s\n' "$chunk"
                else
                    printf '            %s\n' "$chunk"
                fi
                k=$(( k + 1 ))
            done < <(printf '%s\n' "${R_REMEDY[$i]}" | tr '\n' ' ' | fold -s -w 60)
        fi
    done
    printf -- '------------------------------------------------------------------------\n'
    if [ "$1" != 0 ]; then
        printf '  %s FAILED, %s warnings. DO NOT FLASH THIS IMAGE.\n' "$1" "$2"
    elif [ "$2" != 0 ]; then
        printf '  %s warnings, nothing failed. Read them before you flash.\n' "$2"
    else
        printf '  All %s checks passed.\n' "$n"
    fi
    printf '  This did not boot the image. It tested no NPU, no serial device,\n'
    printf '  no DDS traffic, and nothing about whether the rover drives.\n'
    printf '========================================================================\n'
    return 0
}

# --------------------------------------------------------------------------
main() {
    check_artefact
    prepare_image
    attach_and_mount

    check_units
    check_masked
    check_payload
    check_uros
    check_config
    check_versions
    check_identity
    check_hygiene
    check_bootpart
    check_python
    check_calibration

    local i n nfail=0 nwarn=0
    n="${#R_NAME[@]}"
    for (( i=0; i<n; i++ )); do
        case "${R_STATUS[$i]}" in
            "$FAIL") nfail=$(( nfail + 1 )) ;;
            "$WARN") nwarn=$(( nwarn + 1 )) ;;
        esac
    done

    if [ "$OPT_JSON" = 1 ]; then
        emit_json "$nfail" "$nwarn"
    elif [ "$OPT_QUIET" = 0 ]; then
        emit_text "$nfail" "$nwarn"
    fi

    if [ "$nfail" != 0 ]; then return 2; fi
    if [ "$nwarn" != 0 ]; then return 1; fi
    return 0
}

# `main` bare under `set -e` would exit on its own non-zero return before any
# explicit exit ran. Capture it so the intent is on the page.
RC=0
main || RC=$?
exit "$RC"
