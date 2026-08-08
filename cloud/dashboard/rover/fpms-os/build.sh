#!/usr/bin/env bash
# build.sh - build the FPMS-OS image for the Orange Pi 5B.
#
#   sudo ./build.sh                 full build
#   sudo ./build.sh --stage 20      run one stage against the existing chroot
#   sudo ./build.sh --shell         drop into the chroot to poke at it
#   sudo ./build.sh --no-download   reuse the cached base image
#   sudo ./build.sh --dry-run       print what would happen, touch nothing
#
# WHAT THIS PRODUCES
# ==================
# fpms-os-<version>-<date>.img.xz plus a .sha256, flashable to eMMC or SD.
# On first boot it grows itself, provisions WiFi from a file on the FAT
# partition, generates its own broker password and SSH host keys, brings up the
# whole FPMS stack, and self-tests. There is no setup step.
#
# WHAT IT REPLACES
# ================
# deploy_stack.sh, which does much of this at DEPLOY time against a Pi somebody
# had already configured by hand. The problem with that model is recorded in
# PI_FILE_INVENTORY.md: three scripts the entire stack depends on existed only
# at /usr/local/bin on that one Pi, in no repository, and reimaging the card
# would have taken the rover with it. Everything is in the image now.
#
# REQUIREMENTS
# ============
#   - Linux (or WSL2) with root
#   - aarch64 host, OR x86_64 with qemu-user-static + binfmt_misc registered
#   - ~25 GB free
#   apt install qemu-user-static binfmt-support xz-utils parted e2fsprogs \
#               dosfstools kpartx wget

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/config/fpms-os.conf"

WORK="$HERE/.build"
CACHE="$HERE/.cache"
MNT="$WORK/mnt"
STAMP="$(date +%Y%m%d)"
OUT="$HERE/fpms-os-${FPMS_OS_VERSION}-${STAMP}.img"

DRY=0; NO_DOWNLOAD=0; ONE_STAGE=""; SHELL_MODE=0
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)     DRY=1 ;;
        --no-download) NO_DOWNLOAD=1 ;;
        --stage)       ONE_STAGE="${2:?--stage needs a number}"; shift ;;
        --shell)       SHELL_MODE=1 ;;
        -h|--help)     sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31mFATAL: %s\033[0m\n' "$*" >&2; exit 1; }
run()  { if [ "$DRY" = 1 ]; then echo "[dry-run] $*"; else eval "$@"; fi; }

[ "$(id -u)" -eq 0 ] || die "must run as root (loop devices and chroot)"

# ---------------------------------------------------------------------------
# CLEANUP TRAP - this is not optional.
#
# A build that dies with a loop device attached and a chroot still bind-mounted
# leaves the host in a state where the next attempt fails in confusing ways,
# and where an `rm -rf` on the work directory can walk into /dev, /proc or
# /sys through the bind mounts. Always unmount, always detach, in reverse
# order, even on failure.
# ---------------------------------------------------------------------------
LOOPDEV=""
cleanup() {
    set +e
    for m in dev/pts dev proc sys run boot/firmware; do
        mountpoint -q "$MNT/$m" 2>/dev/null && umount -lf "$MNT/$m" 2>/dev/null
    done
    mountpoint -q "$MNT" 2>/dev/null && umount -lf "$MNT" 2>/dev/null
    [ -n "$LOOPDEV" ] && losetup -d "$LOOPDEV" 2>/dev/null
    set -e
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# Cross-architecture check
# ---------------------------------------------------------------------------
check_arch() {
    local host; host="$(uname -m)"
    if [ "$host" = "aarch64" ]; then
        log "native aarch64 build"
        return
    fi
    log "cross build on $host - checking qemu-aarch64 binfmt"
    [ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ] \
        || die "qemu-aarch64 is not registered with binfmt_misc.
  apt install qemu-user-static binfmt-support
  (on some hosts:  docker run --rm --privileged multiarch/qemu-user-static --reset -p yes)
Without it, every command inside the chroot fails with 'Exec format error'."
    [ -x /usr/bin/qemu-aarch64-static ] \
        || die "/usr/bin/qemu-aarch64-static missing (apt install qemu-user-static)"
}

# ---------------------------------------------------------------------------
# Base image
# ---------------------------------------------------------------------------
fetch_base() {
    mkdir -p "$CACHE"
    local xz="$CACHE/${BASE_IMAGE_NAME}.img.xz"
    local img="$CACHE/${BASE_IMAGE_NAME}.img"

    if [ -f "$img" ] && [ "$NO_DOWNLOAD" = 1 ]; then
        log "reusing cached base image"; echo "$img"; return
    fi

    if [ ! -f "$xz" ]; then
        log "downloading base image"
        run "wget -O '$xz.part' '$BASE_IMAGE_URL'" && mv "$xz.part" "$xz"
    fi

    # A build on an unverified base is not reproducible, and upstream re-cuts
    # releases under the same filename.
    if [ "${BASE_IMAGE_REQUIRE_SHA:-1}" = 1 ]; then
        [ "$BASE_IMAGE_SHA256" = "__SET_ME__" ] && die \
"BASE_IMAGE_SHA256 is unset in config/fpms-os.conf.

Download the base image, run:
    sha256sum '$xz'
check it against the checksum published with the upstream release, and put it
in the config. Refusing to build on an unverified base."
        log "verifying checksum"
        echo "${BASE_IMAGE_SHA256}  ${xz}" | sha256sum -c - \
            || die "base image checksum MISMATCH - refusing to build"
    fi

    if [ ! -f "$img" ]; then
        log "decompressing base image"
        run "xz -dkc '$xz' > '$img.part'" && mv "$img.part" "$img"
    fi
    echo "$img"
}

# ---------------------------------------------------------------------------
# Mount / grow
# ---------------------------------------------------------------------------
prepare_image() {
    local base="$1"
    mkdir -p "$WORK" "$MNT"
    log "copying base -> $OUT"
    run "cp --sparse=always '$base' '$OUT'"

    log "growing rootfs by ${ROOTFS_GROW_MB} MB"
    run "truncate -s +${ROOTFS_GROW_MB}M '$OUT'"

    LOOPDEV="$(losetup --find --show --partscan "$OUT")"
    log "loop device: $LOOPDEV"

    # Rockchip images put the rootfs last, so growing the file and then the
    # last partition is safe. parted needs the "Fix" answer non-interactively
    # because the backup GPT header has moved.
    run "parted -s '$LOOPDEV' print free >/dev/null 2>&1 || true"
    run "sgdisk -e '$LOOPDEV' >/dev/null 2>&1 || true"
    local rootpart="${LOOPDEV}p2"
    [ -e "$rootpart" ] || rootpart="${LOOPDEV}p1"
    run "parted -s '$LOOPDEV' resizepart ${rootpart##*p} 100%"
    run "e2fsck -fy '$rootpart' >/dev/null 2>&1 || true"
    run "resize2fs '$rootpart'"

    log "mounting"
    run "mount '$rootpart' '$MNT'"
    if [ -e "${LOOPDEV}p1" ] && [ "$rootpart" != "${LOOPDEV}p1" ]; then
        run "mkdir -p '$MNT/boot/firmware'"
        run "mount '${LOOPDEV}p1' '$MNT/boot/firmware' || true"
    fi
}

enter_chroot_mounts() {
    for m in proc sys dev dev/pts run; do
        run "mkdir -p '$MNT/$m'"
    done
    run "mount -t proc  proc  '$MNT/proc'"
    run "mount -t sysfs sys   '$MNT/sys'"
    run "mount --bind /dev    '$MNT/dev'"
    run "mount --bind /dev/pts '$MNT/dev/pts'"
    run "mount -t tmpfs tmpfs '$MNT/run'"

    if [ "$(uname -m)" != "aarch64" ]; then
        run "cp /usr/bin/qemu-aarch64-static '$MNT/usr/bin/'"
    fi

    # The chroot has no working DNS otherwise, and every stage needs apt.
    run "cp /etc/resolv.conf '$MNT/etc/resolv.conf'"

    # Keep services from starting inside the chroot. Without this, installing
    # mosquitto or ssh tries to start them against the host's init and either
    # fails the build or, worse, half-succeeds.
    run "printf '#!/bin/sh\\nexit 101\\n' > '$MNT/usr/sbin/policy-rc.d'"
    run "chmod +x '$MNT/usr/sbin/policy-rc.d'"
}

in_chroot() {
    if [ "$DRY" = 1 ]; then echo "[dry-run] chroot: $*"; return; fi
    chroot "$MNT" /usr/bin/env -i \
        HOME=/root PATH=/usr/sbin:/usr/bin:/sbin:/bin TERM=xterm \
        DEBIAN_FRONTEND=noninteractive LC_ALL=C \
        FPMS_OS_VERSION="$FPMS_OS_VERSION" \
        ROS_DISTRO="$ROS_DISTRO" ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
        FPMS_USER="$FPMS_USER" FPMS_HOME="$FPMS_HOME" \
        FPMS_HOSTNAME="$FPMS_HOSTNAME" \
        PIP_PAHO="$PIP_PAHO" PIP_NUMPY="$PIP_NUMPY" \
        PIP_WEBSOCKET="$PIP_WEBSOCKET" \
        RKNN_LITE_WHEEL_URL="$RKNN_LITE_WHEEL_URL" \
        LIBRKNNRT_URL="$LIBRKNNRT_URL" \
        RKNN_TOOLKIT2_TAG="$RKNN_TOOLKIT2_TAG" \
        /bin/bash -c "$*"
}

stage_all() {
    # Payload the stages need, staged where the chroot can reach it.
    run "rm -rf '$MNT/opt/fpms-os'"
    run "mkdir -p '$MNT/opt/fpms-os'"
    run "cp -a '$HERE/scripts' '$HERE/overlay' '$HERE/selftest' '$HERE/docs' \
             '$HERE/firstboot' '$HERE/SPEC.md' '$MNT/opt/fpms-os/'"
    # The rover source tree, two levels up from here.
    run "mkdir -p '$MNT/opt/fpms-os/src'"
    run "cp -a '$HERE/..'/*.py '$HERE/../stack' '$HERE/../nav2' '$HERE/../slam' \
             '$HERE/../STACK.md' '$MNT/opt/fpms-os/src/' 2>/dev/null || true"
    run "chmod +x '$MNT/opt/fpms-os/scripts/'*.sh"

    for s in "$MNT/opt/fpms-os/scripts/"[0-9]*.sh; do
        local base; base="$(basename "$s")"
        if [ -n "$ONE_STAGE" ]; then
            case "$base" in "$ONE_STAGE"*) ;; *) continue ;; esac
        fi
        log "stage: $base"
        in_chroot "/opt/fpms-os/scripts/$base" \
            || die "stage $base FAILED - the image is not usable. Fix and re-run with --stage ${base%%-*}"
    done
}

finalise() {
    log "finalising"
    run "rm -f '$MNT/usr/sbin/policy-rc.d'"
    run "rm -f '$MNT/usr/bin/qemu-aarch64-static'"
    cleanup
    trap - EXIT INT TERM

    log "compressing (this takes a while)"
    run "xz -T0 -6 -f '$OUT'"
    run "sha256sum '$OUT.xz' > '$OUT.xz.sha256'"

    printf '\n\033[1;32m'
    echo "======================================================================"
    echo " FPMS-OS ${FPMS_OS_VERSION} built"
    echo "   $OUT.xz"
    echo "   $(cat "$OUT.xz.sha256" 2>/dev/null || echo '(checksum pending)')"
    echo ""
    echo " Next: docs/FLASHING.md"
    echo " Before first boot, put your WiFi details in fpms-wifi.conf on the"
    echo " FAT boot partition. See README-FPMS.txt there."
    echo "======================================================================"
    printf '\033[0m\n'
}

# ---------------------------------------------------------------------------
main() {
    check_arch
    local base; base="$(fetch_base)"
    prepare_image "$base"
    enter_chroot_mounts

    if [ "$SHELL_MODE" = 1 ]; then
        log "chroot shell - exit when done (image will NOT be finalised)"
        chroot "$MNT" /bin/bash || true
        exit 0
    fi

    stage_all

    if [ -n "$ONE_STAGE" ]; then
        log "single stage complete; not finalising"
        exit 0
    fi
    finalise
}

main "$@"
