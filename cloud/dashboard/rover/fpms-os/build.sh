#!/usr/bin/env bash
# build.sh - build the FPMS-OS image for the Orange Pi 5B.
#
#   sudo ./build.sh                 full build
#   sudo ./build.sh --stage 20      run ONE stage against the existing image
#   sudo ./build.sh --from 20       run stage 20 and everything after it
#   sudo ./build.sh --shell         drop into the chroot to poke at it
#   sudo ./build.sh --no-download   reuse the cached base image
#   sudo ./build.sh --fresh         with --stage/--from: start from the base
#                                   image again instead of resuming
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
#   - ~14 GB free on the filesystem holding this directory. The pre-flight in
#     prepare_image() checks for the WORST case - the whole image allocated at
#     its apparent size, plus the .xz alongside it - because a sparse .img is
#     only sparse until something writes to the blocks, and ROS Humble plus
#     Nav2 plus the micro-ROS workspace writes to a great many of them.
#   apt install qemu-user-static binfmt-support xz-utils parted e2fsprogs \
#               dosfstools kpartx gdisk util-linux udev wget
#
#     gdisk is NOT optional: growing the file moves the end of the disk, and
#     without `sgdisk -e` the backup GPT header stays where it was and parted
#     refuses to resize the last partition. An earlier revision hid that
#     behind `|| true` and the resize silently did nothing.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/config/fpms-os.conf"

WORK="$HERE/.build"
CACHE="$HERE/.cache"
MNT="$WORK/mnt"
STAMP="$(date +%Y%m%d)"
OUT="$HERE/fpms-os-${FPMS_OS_VERSION}-${STAMP}.img"

# Abort with a number rather than let the environment's watchdog (which kills
# below 1.5 GB) take us down mid-write, because a kill mid-write leaves a loop
# device attached and the next attempt fails in a way that looks unrelated.
MIN_FREE_MB=2048
# Headroom for the .xz, which exists alongside the .img until xz finishes.
COMPRESS_HEADROOM_MB=2048

DRY=0; NO_DOWNLOAD=0; ONE_STAGE=""; FROM_STAGE=""; SHELL_MODE=0; FRESH=0
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)     DRY=1 ;;
        --no-download) NO_DOWNLOAD=1 ;;
        --stage)       ONE_STAGE="${2:?--stage needs a number}"; shift ;;
        --from)        FROM_STAGE="${2:?--from needs a number}"; shift ;;
        --fresh)       FRESH=1 ;;
        --shell)       SHELL_MODE=1 ;;
        -h|--help)     sed -n '2,43p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

# log() writes to STDERR, and that is load-bearing, not style.
#
# fetch_base() returns the path to the base image on stdout via
# `base="$(fetch_base)"`. When log() printed to stdout every banner it emitted
# - and the "img.xz: OK" line from sha256sum -c - was captured into $base too,
# so the very next command became
#     cp --sparse=always '<banner><newline>/root/.../base.img' out.img
# which fails on the first full build with a cp error naming a path that looks
# almost right. Anything a function may return through stdout must be the ONLY
# thing that function writes to stdout.
log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*" >&2; }
note() { printf '    %s\n' "$*" >&2; }
die()  { printf '\033[1;31mFATAL: %s\033[0m\n' "$*" >&2; exit 1; }
run()  { if [ "$DRY" = 1 ]; then echo "[dry-run] $*" >&2; else eval "$@"; fi; }

[ "$(id -u)" -eq 0 ] || die "must run as root (loop devices and chroot)"

# ---------------------------------------------------------------------------
# DISK SPACE
#
# Disk exhaustion is the single most likely way this build dies, and it dies
# badly: the failure surfaces as an unrelated apt or colcon error deep inside
# a stage, hours in. Report free space at every seam so a post-mortem can
# attribute it, and refuse up front when the number cannot possibly work.
# ---------------------------------------------------------------------------
# The trailing `|| true` is not decoration: `pipefail` is on, so a df that
# fails would make the whole `have="$(free_mb ...)"` assignment non-zero and
# `set -e` would kill the build inside its own disk-space check.
free_mb() { df -Pm "$1" 2>/dev/null | awk 'NR==2 {print $4}' || true; }

report_space() {   # report_space <label>
    local f; f="$(free_mb "$HERE")"
    note "free space (${1}): ${f:-?} MB"
}

require_space() {  # require_space <needed MB> <what for>
    local need="$1" what="$2" have
    have="$(free_mb "$HERE")"
    [ -n "$have" ] || return 0
    if [ "$have" -lt "$need" ]; then
        die "not enough disk space for $what.
  free:     ${have} MB   on $(df -P "$HERE" | awk 'NR==2 {print $6}')
  required: ${need} MB
  short by: $(( need - have )) MB

Reclaim space and re-run. The largest reclaimable things are usually:
  $CACHE/${BASE_IMAGE_NAME}.img      (decompressed base, not needed - this
                                      script streams from the .xz)
  $HERE/fpms-os-*.img                (previous unfinished builds)
  and inside WSL:  apt-get clean; docker system prune"
    fi
}

space_floor() {    # space_floor <where we are>
    local have; have="$(free_mb "$HERE")"
    [ -n "$have" ] || return 0
    [ "$have" -ge "$MIN_FREE_MB" ] || die \
"disk space fell to ${have} MB during: $1
Minimum is ${MIN_FREE_MB} MB. Stopping here deliberately, while the loop
device and mounts can still be released cleanly. A build killed by the
environment's own low-space watchdog leaks the loop device instead."
}

# ---------------------------------------------------------------------------
# CLEANUP TRAP - this is not optional.
#
# A build that dies with a loop device attached and a chroot still bind-mounted
# leaves the host in a state where the next attempt fails in confusing ways,
# and where an `rm -rf` on the work directory can walk into /dev, /proc or
# /sys through the bind mounts. Always unmount, always detach, in reverse
# order, even on failure.
#
# Two things the previous version got wrong, both of which leak a loop device
# on WSL2 and wedge the next attempt:
#
#   1. It used `umount -lf` (LAZY) unconditionally. A lazy unmount detaches the
#      name, not the filesystem - the loop device is still busy when
#      `losetup -d` runs a microsecond later, that fails, the failure is
#      swallowed by 2>/dev/null, and the device stays attached. Worse, writes
#      may not have reached the backing file yet, so the image can be corrupt.
#      Unmount properly first; lazy only as a last resort, and sync either way.
#   2. It never cleared LOOPDEV, so cleanup was not idempotent - it runs on the
#      normal path (from finalise) AND from the EXIT trap.
# ---------------------------------------------------------------------------
LOOPDEV=""
PART_PREFIX=""
KPARTX_USED=0
ROOTPART=""
ROOTPARTNUM=""
BOOTPART=""

umount_quietly() {  # umount_quietly <dir>
    local d="$1" i
    mountpoint -q "$d" 2>/dev/null || return 0
    for i in 1 2 3; do
        umount "$d" 2>/dev/null && return 0
        sleep 1
    done
    # Something still holds it open. Lazy-detach so we can at least get the
    # loop device back, but say so - a lazy unmount here means the image may
    # not be fully written.
    note "WARNING: $d would not unmount cleanly; forcing (image may be stale)"
    umount -lf "$d" 2>/dev/null || true
}

cleanup() {
    set +e
    sync 2>/dev/null

    for m in dev/pts dev proc sys run boot/firmware; do
        umount_quietly "$MNT/$m"
    done
    umount_quietly "$MNT"
    sync 2>/dev/null

    if [ -n "$LOOPDEV" ]; then
        [ "$KPARTX_USED" = 1 ] && kpartx -d "$LOOPDEV" >/dev/null 2>&1
        partx -d "$LOOPDEV" >/dev/null 2>&1
        local i
        for i in 1 2 3 4 5; do
            losetup -d "$LOOPDEV" 2>/dev/null && break
            sleep 1
        done
        if losetup -j "${OUT}" 2>/dev/null | grep -q .; then
            printf '\033[1;31mWARNING: %s is still attached to %s.\n' \
                   "$OUT" "$LOOPDEV" >&2
            printf '  Run: losetup -d %s   before the next build.\033[0m\n' \
                   "$LOOPDEV" >&2
        fi
        LOOPDEV=""; PART_PREFIX=""; KPARTX_USED=0
        ROOTPART=""; ROOTPARTNUM=""; BOOTPART=""
    fi
    set -e
}
trap cleanup EXIT
# Without an explicit exit here the shell RESUMES the interrupted stage after
# the handler returns - against a chroot we have just unmounted.
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# Anything left over from a previous run that died badly. On WSL2 a leaked
# loop device on the same backing file makes `losetup --find` hand out a
# second device for the same image, and then two mounts fight over one ext4.
release_stale() {
    [ "$DRY" = 1 ] && return 0
    local m stale
    for m in dev/pts dev proc sys run boot/firmware ''; do
        mountpoint -q "$MNT/$m" 2>/dev/null && {
            note "releasing stale mount: $MNT/$m"
            umount_quietly "$MNT/$m"
        }
    done
    for stale in $(losetup -j "$OUT" 2>/dev/null | cut -d: -f1); do
        note "detaching stale loop device on $(basename "$OUT"): $stale"
        partx -d "$stale" >/dev/null 2>&1 || true
        losetup -d "$stale" 2>/dev/null || true
    done
}

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

    local need
    for need in sgdisk parted e2fsck resize2fs blkid partx blockdev losetup \
                mountpoint xz wget sha256sum; do
        command -v "$need" >/dev/null 2>&1 || die \
"$need is not installed, and the build needs it.
  apt install qemu-user-static binfmt-support xz-utils parted e2fsprogs \\
              dosfstools kpartx gdisk util-linux udev wget"
    done
}

# ---------------------------------------------------------------------------
# Base image
#
# Returns ONE line on stdout: the path to use as the source. That may be the
# .xz itself - prepare_image() decompresses straight into the output file
# rather than materialising a second full-size copy in the cache. On a host
# with 17 GB free, a 4-5 GB cached .img is the difference between a build that
# finishes and one that dies in stage 90.
# ---------------------------------------------------------------------------
fetch_base() {
    mkdir -p "$CACHE"
    local xz="$CACHE/${BASE_IMAGE_NAME}.img.xz"
    local img="$CACHE/${BASE_IMAGE_NAME}.img"

    if [ ! -f "$xz" ]; then
        [ "$NO_DOWNLOAD" = 1 ] && die \
"--no-download was given but there is no cached base image at
    $xz
and no decompressed copy at
    $img"
        log "downloading base image"
        run "wget -O '$xz.part' '$BASE_IMAGE_URL'"
        [ "$DRY" = 1 ] || mv "$xz.part" "$xz"
    fi

    # A build on an unverified base is not reproducible, and upstream re-cuts
    # releases under the same filename.
    if [ "${BASE_IMAGE_REQUIRE_SHA:-1}" = 1 ] && [ "$DRY" != 1 ]; then
        [ "$BASE_IMAGE_SHA256" = "__SET_ME__" ] && die \
"BASE_IMAGE_SHA256 is unset in config/fpms-os.conf.

Download the base image, run:
    sha256sum '$xz'
check it against the checksum published with the upstream release, and put it
in the config. Refusing to build on an unverified base."
        log "verifying checksum"
        echo "${BASE_IMAGE_SHA256}  ${xz}" | sha256sum -c - >/dev/null \
            || die "base image checksum MISMATCH - refusing to build"
        note "checksum OK"
    fi

    # Only reuse a decompressed copy if somebody already made one. Never
    # create one: it costs the same disk as the output image and buys nothing
    # except a slightly faster second build.
    if [ -f "$img" ]; then
        note "using the decompressed cache copy ($(du -m "$img" | cut -f1) MB)"
        note "you can delete it - the .xz alone is enough - and get that back"
        echo "$img"
    else
        echo "$xz"
    fi
}

# ---------------------------------------------------------------------------
# Partitions
#
# WHY NOT ${LOOPDEV}p2
# ====================
# The previous version guessed p2 and fell back to p1. ubuntu-rockchip is GPT
# and the partition set is not fixed across board variants or releases -
# loader/uboot/trust entries can and do appear ahead of the filesystems. Guess
# wrong and you mount the FAT boot partition as the rootfs; every stage then
# fails with something that looks like a corrupt base image rather than "you
# mounted the wrong partition". Detect the rootfs by what it IS - the largest
# ext[234] filesystem - and prefer an explicit label when upstream sets one.
# ---------------------------------------------------------------------------
part_nodes() {
    local n
    for n in "${PART_PREFIX}"*; do
        [ -b "$n" ] && echo "$n"
    done
    # Explicit, because "no partitions yet" is the NORMAL case on the first
    # probe and must not look like an error: `part_nodes | wc -l` under
    # `pipefail` would otherwise return non-zero and `set -e` would kill the
    # build in the middle of waiting for the partition scan.
    return 0
}

# losetup --partscan asks the kernel to scan; it does not promise the device
# nodes exist by the time losetup returns, and on WSL2 (where udev is often
# not running at all) they may never appear on their own. Wait, then force it.
settle_partitions() {
    local i n
    PART_PREFIX="${LOOPDEV}p"
    for i in $(seq 1 20); do
        udevadm settle --timeout=2 >/dev/null 2>&1 || true
        n="$(part_nodes | wc -l)"
        [ "$n" -ge 1 ] && { note "partition nodes: $n (after $i probe(s))"; return 0; }

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

losetup --partscan ran, partx -a ran, kpartx ran, and none of
    ${LOOPDEV}p*   /dev/mapper/$(basename "$LOOPDEV")p*
exists. On WSL2 this usually means the kernel has no loop partition support
(CONFIG_BLK_DEV_LOOP_MIN_COUNT / max_part) or udev is not running.
Check:  losetup -l ; ls -l /dev/loop* ; lsblk"
}

detect_partitions() {
    local node type label size best=0 label_locked=0
    ROOTPART=""; ROOTPARTNUM=""; BOOTPART=""

    note "partition table: $(blkid -o value -s PTTYPE "$LOOPDEV" 2>/dev/null || echo '?')"
    while read -r node; do
        [ -n "$node" ] || continue
        type="$(blkid -o value -s TYPE  "$node" 2>/dev/null || true)"
        label="$(blkid -o value -s LABEL "$node" 2>/dev/null || true)"
        size="$(blockdev --getsize64 "$node" 2>/dev/null || echo 0)"
        note "$(printf '%-24s %-8s %-12s %6s MB' \
                "$node" "${type:--}" "${label:--}" "$(( size / 1048576 ))")"

        case "$type" in
            ext2|ext3|ext4)
                # An explicit label wins outright. ubuntu-rockchip labels the
                # rootfs `writable`; other Ubuntu arm64 images use `cloudimg-rootfs`.
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
                [ -n "$BOOTPART" ] || BOOTPART="$node" ;;
        esac
    done < <(part_nodes)

    [ -n "$ROOTPART" ] || die \
"could not find an ext2/3/4 rootfs on $LOOPDEV.

The partitions found are listed above. If they are all empty or all vfat, the
base image is not what config/fpms-os.conf says it is, or the partition scan
handed back stale nodes from a previous build. Try:
    losetup -D ; ls /dev/loop*"

    # The number is what parted resizepart wants, and it must come from the
    # detection above. `${rootpart##*p}` on a guessed node is how you end up
    # resizing partition 1 (the bootloader) on a five-partition Rockchip image.
    ROOTPARTNUM="${ROOTPART##*p}"
    case "$ROOTPARTNUM" in
        ''|*[!0-9]*) die "cannot derive a partition number from $ROOTPART" ;;
    esac

    note "rootfs -> $ROOTPART (partition $ROOTPARTNUM)"
    [ -n "$BOOTPART" ] && note "boot   -> $BOOTPART (FAT, what Windows shows the operator)"
    return 0
}

# Re-read the table after sgdisk/parted have changed it. partx -u refreshes the
# kernel's idea of ${LOOPDEV}p*; if we had to fall back to kpartx the real nodes
# live under /dev/mapper and need their own refresh, or resize2fs is handed a
# device-mapper target that is still the OLD size and grows the filesystem to
# the wrong length.
refresh_partitions() {
    partx -u "$LOOPDEV" >/dev/null 2>&1 || true
    [ "$KPARTX_USED" = 1 ] && { kpartx -u "$LOOPDEV" >/dev/null 2>&1 || true; }
    udevadm settle --timeout=5 >/dev/null 2>&1 || true
    return 0
}

attach_loop() {
    LOOPDEV="$(losetup --find --show --partscan "$OUT")" \
        || die "losetup failed on $OUT"
    log "loop device: $LOOPDEV"
    settle_partitions
    detect_partitions
}

# ---------------------------------------------------------------------------
# Mount / grow
# ---------------------------------------------------------------------------
prepare_image() {
    local src="$1"
    mkdir -p "$WORK" "$MNT"

    if [ "$DRY" = 1 ]; then
        log "[dry-run] would build $OUT from $src, grow +${ROOTFS_GROW_MB} MB,"
        note "[dry-run] detect the rootfs partition, resize it, and mount it."
        note "[dry-run] loop devices and partition scans are not simulated."
        return 0
    fi

    release_stale

    # --- space pre-flight ---------------------------------------------------
    #
    # Do this BEFORE decompressing, not after. The number that matters is the
    # image's APPARENT size, not its sparse size: the stages fill most of the
    # rootfs for real, and if fstrim turns out to be unsupported on this loop
    # device the free blocks stay allocated too. Budget for the worst case and
    # refuse with a number now, rather than run out four hours in where it
    # surfaces as an apt or colcon error that names something else entirely.
    local base_mb=0
    case "$src" in
        *.xz)
            base_mb="$(xz --robot -l "$src" 2>/dev/null \
                       | awk '/^totals/ {print int($5/1048576)}')" ;;
        *)  base_mb="$(( $(stat -c %s "$src") / 1048576 ))" ;;
    esac
    [ -n "$base_mb" ] && [ "$base_mb" -gt 0 ] || base_mb=5120   # be pessimistic
    local need=$(( base_mb + ROOTFS_GROW_MB + COMPRESS_HEADROOM_MB + MIN_FREE_MB ))
    note "base image is ${base_mb} MB decompressed; +${ROOTFS_GROW_MB} MB growth"
    require_space "$need" "the image, its growth, and the .xz alongside it"
    report_space "before unpacking the base"

    # --- materialise the output image ---------------------------------------
    rm -f "$OUT" "$OUT.part"
    case "$src" in
        *.xz)
            # Straight from the .xz into the output. The old path decompressed
            # into .cache and then `cp --sparse=always` to the output, which
            # holds two full-size copies on a disk that does not have room for
            # two. xz's own output is dense, so pipe it through `cp --sparse`
            # semantics by letting the filesystem punch holes later (fstrim,
            # below) rather than paying for a second copy now.
            log "decompressing base -> $OUT"
            xz -dc "$src" > "$OUT.part" || die "decompressing $src failed"
            mv "$OUT.part" "$OUT" ;;
        *)
            log "copying base -> $OUT"
            cp --sparse=always "$src" "$OUT" || die "copying $src failed" ;;
    esac
    report_space "after unpacking the base"

    log "growing image by ${ROOTFS_GROW_MB} MB"
    truncate -s "+${ROOTFS_GROW_MB}M" "$OUT"

    attach_loop

    # --- move the backup GPT header ------------------------------------------
    #
    # The file just got longer, so the secondary GPT header is now stranded in
    # the middle of the disk. parted will not extend the last partition past a
    # header it still believes is the end. `sgdisk -e` moves it. The previous
    # revision ran this as `sgdisk -e ... || true`, so on a host without gdisk
    # the resize silently did nothing and the rootfs stayed at its stock size -
    # which surfaces two hours later as apt running out of space in stage 10.
    if [ "$(blkid -o value -s PTTYPE "$LOOPDEV" 2>/dev/null)" = "gpt" ]; then
        log "moving the backup GPT header to the new end of disk"
        sgdisk -e "$LOOPDEV" >/dev/null 2>&1 \
            || die "sgdisk -e failed on $LOOPDEV.
Without it the last partition cannot be grown. Install gdisk:
    apt install gdisk
and check the table by hand with:  sgdisk -v '$LOOPDEV'"
        refresh_partitions
    else
        note "not a GPT disk; skipping the sgdisk backup-header move"
    fi

    local was_mb; was_mb=$(( $(blockdev --getsize64 "$ROOTPART") / 1048576 ))
    log "resizing partition $ROOTPARTNUM (currently ${was_mb} MB) to fill the image"
    parted -s "$LOOPDEV" resizepart "$ROOTPARTNUM" 100% \
        || die "parted resizepart $ROOTPARTNUM failed on $LOOPDEV"
    refresh_partitions
    [ -b "$ROOTPART" ] || die "$ROOTPART vanished after resizepart"

    # PROVE the resize landed, right here. The old `sgdisk -e ... || true`
    # could leave the partition at its stock size with nothing said, and a
    # rootfs that is still 4 GB does not fail until apt runs out of room two
    # hours later inside stage 10, where it looks like a mirror problem.
    local now_mb; now_mb=$(( $(blockdev --getsize64 "$ROOTPART") / 1048576 ))
    note "rootfs partition: ${was_mb} MB -> ${now_mb} MB"
    [ "$now_mb" -ge $(( was_mb + ROOTFS_GROW_MB - 64 )) ] || die \
"the rootfs partition did not grow.
  before: ${was_mb} MB
  after:  ${now_mb} MB
  wanted: at least $(( was_mb + ROOTFS_GROW_MB )) MB

Almost always the backup GPT header: parted will not extend the last partition
past where it thinks the disk ends. Check by hand:
    sgdisk -v '$LOOPDEV'
    parted -s '$LOOPDEV' print free"

    # e2fsck's exit status is a bitmask: 1 = errors corrected, 2 = corrected and
    # a reboot is advised. Anything >= 4 is a filesystem we must not build on,
    # and the old `|| true` would have carried straight on into it.
    log "checking and growing the filesystem"
    local rc=0
    e2fsck -fy "$ROOTPART" >/dev/null 2>&1 || rc=$?
    [ "$rc" -le 2 ] || die "e2fsck on $ROOTPART returned $rc - the base rootfs
is damaged. Delete $CACHE and re-download the base image."
    resize2fs "$ROOTPART" || die "resize2fs failed on $ROOTPART"

    mount_image
}

# Mounting is separate from preparing so that --stage/--from can re-attach an
# image that already has hours of work in it. See resume_image().
mount_image() {
    log "mounting"
    # -o discard so that when stage 90 removes its zero-fill file the blocks
    # are punched back out of the sparse backing file immediately, instead of
    # staying allocated until the fstrim in finalise().
    mount -o discard "$ROOTPART" "$MNT" \
        || mount "$ROOTPART" "$MNT" \
        || die "cannot mount $ROOTPART on $MNT"

    # Sanity: if partition detection had gone wrong we would be looking at a
    # FAT boot partition right now, and every stage would fail confusingly.
    [ -d "$MNT/etc" ] && [ -d "$MNT/usr/bin" ] || die \
"$ROOTPART mounted, but it does not look like a root filesystem
(no /etc or /usr/bin). This is the wrong partition. Re-check the table:
    parted -s '$LOOPDEV' print
    blkid ${PART_PREFIX}*"

    # NOT `|| true`. scripts/50-overlay.sh copies the operator-visible files
    # (README-FPMS.txt, fpms-wifi.conf.example) onto whatever FAT partition it
    # finds mounted, and falls back to the ext4 /boot with a loud warning if
    # there is none - where Windows cannot see them, which is the whole point
    # of them. If we detected a FAT partition and then failed to mount it, that
    # is a defect here, not a degraded mode to shrug at.
    if [ -n "$BOOTPART" ]; then
        mkdir -p "$MNT/boot/firmware"
        mount "$BOOTPART" "$MNT/boot/firmware" || die \
"could not mount the FAT boot partition $BOOTPART on $MNT/boot/firmware.

Without it the operator never sees fpms-wifi.conf.example from Windows, and
every WiFi instruction in docs/FLASHING.md refers to a file that is not there.
Check:  blkid '$BOOTPART' ; dmesg | tail"
    else
        note "WARNING: no FAT partition in this image. The operator-visible boot"
        note "         files will land in the ext4 /boot, which Windows cannot read."
    fi
}

# --stage / --from used to call prepare_image(), which starts by copying the
# base image over $OUT. That destroys the build. When stage 10 fails 90 minutes
# in, re-running `--stage 10` must resume against the image that already has
# stage 00 in it, not start again from the vendor rootfs.
resume_image() {
    [ -f "$OUT" ] || die \
"--stage/--from asked to resume, but there is no image at
    $OUT
Run a full build first, or pass --fresh to build one from the base image."

    if [ "$DRY" = 1 ]; then
        log "[dry-run] would re-attach and mount $OUT"
        return 0
    fi

    log "resuming against the existing image ($(du -m "$OUT" | cut -f1) MB)"
    release_stale
    attach_loop
    mount_image
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

    setup_qemu

    # --- DNS ----------------------------------------------------------------
    #
    # The chroot has no working DNS otherwise, and every stage needs apt.
    #
    # But /etc/resolv.conf in this base image is a SYMLINK into
    # /run/systemd/resolve/, and we have just mounted an empty tmpfs on /run -
    # so a plain `cp` writes THROUGH the dangling symlink and fails with
    # "cannot create regular file: No such file or directory", killing the
    # build before stage 00 even starts. Replace the link, and remember what it
    # was so finalise() can put it back: shipping an image whose resolv.conf
    # holds the BUILD HOST's nameserver (a WSL 172.x address) would break DNS
    # on the rover in a way nothing here would ever report.
    if [ "$DRY" != 1 ]; then
        if [ ! -f "$WORK/resolv.orig" ]; then
            if [ -L "$MNT/etc/resolv.conf" ]; then
                printf 'symlink %s\n' "$(readlink "$MNT/etc/resolv.conf")" \
                    > "$WORK/resolv.orig"
            elif [ -f "$MNT/etc/resolv.conf" ]; then
                printf 'file\n' > "$WORK/resolv.orig"
                cp "$MNT/etc/resolv.conf" "$WORK/resolv.orig.body"
            else
                printf 'absent\n' > "$WORK/resolv.orig"
            fi
        fi
        rm -f "$MNT/etc/resolv.conf"
        cp /etc/resolv.conf "$MNT/etc/resolv.conf" \
            || die "could not install a build-time /etc/resolv.conf"
    fi

    # Keep services from starting inside the chroot. Without this, installing
    # mosquitto or ssh tries to start them against the host's init and either
    # fails the build or, worse, half-succeeds.
    run "printf '#!/bin/sh\\nexit 101\\n' > '$MNT/usr/sbin/policy-rc.d'"
    run "chmod +x '$MNT/usr/sbin/policy-rc.d'"

    smoke_test_chroot
}

# ---------------------------------------------------------------------------
# qemu placement
#
# THE PATH IT COPIES TO IS NOT /usr/bin.
#
# binfmt_misc resolves the interpreter path INSIDE the chroot, and on this host
# the registered interpreter is
#     /usr/libexec/qemu-binfmt/aarch64-binfmt-P
# not /usr/bin/qemu-aarch64-static. Copying the static binary to
# $MNT/usr/bin/ therefore satisfies nothing: the kernel looks for
# $MNT/usr/libexec/qemu-binfmt/aarch64-binfmt-P, does not find it, and every
# single command in every stage dies with "Exec format error" - the exact
# symptom check_arch() promises this cannot be.
#
# Unless the registration carries the F (fix-binary) flag, in which case the
# kernel holds an open fd to the interpreter and the chroot needs no copy at
# all. Read the registration; do what it actually says.
# ---------------------------------------------------------------------------
QEMU_PLACED=""
setup_qemu() {
    [ "$(uname -m)" = "aarch64" ] && return 0
    [ "$DRY" = 1 ] && { echo "[dry-run] would place qemu per binfmt_misc" >&2; return 0; }

    local reg=/proc/sys/fs/binfmt_misc/qemu-aarch64 interp flags
    interp="$(awk '/^interpreter /{print $2; exit}' "$reg" 2>/dev/null || true)"
    flags="$(awk -F':[[:space:]]*' '/^flags:/{print $2; exit}' "$reg" 2>/dev/null || true)"
    note "binfmt interpreter: ${interp:-?}   flags: ${flags:-none}"

    if [ -n "$flags" ] && [ "${flags#*F}" != "$flags" ]; then
        note "the F (fix-binary) flag is set - the kernel already holds the"
        note "interpreter open, so nothing needs copying into the image"
        return 0
    fi

    [ -n "$interp" ] || die "could not read the interpreter path from $reg"

    QEMU_PLACED="$MNT$interp"
    mkdir -p "$(dirname "$QEMU_PLACED")"
    cp /usr/bin/qemu-aarch64-static "$QEMU_PLACED" \
        || die "could not place qemu at $QEMU_PLACED"
    chmod 0755 "$QEMU_PLACED"
    note "placed the static qemu at ${interp} inside the image"

    # Also at the conventional path. Costs 8 MB for the length of the build and
    # covers anything that invokes it by name rather than through binfmt.
    if [ "$interp" != "/usr/bin/qemu-aarch64-static" ]; then
        cp /usr/bin/qemu-aarch64-static "$MNT/usr/bin/qemu-aarch64-static"
        chmod 0755 "$MNT/usr/bin/qemu-aarch64-static"
    fi
}

# Fail in ten seconds rather than twenty minutes into stage 00. Every previous
# emulation problem in this project has announced itself as an inscrutable
# error from apt; this asks the question directly.
smoke_test_chroot() {
    [ "$DRY" = 1 ] && return 0
    local out
    if ! out="$(chroot "$MNT" /bin/echo fpms-chroot-ok 2>&1)" \
       || [ "$out" != "fpms-chroot-ok" ]; then
        die "the chroot cannot execute aarch64 binaries.
  chroot said: ${out:-<nothing>}

If that is 'Exec format error', binfmt_misc points at an interpreter path that
does not exist inside the image. Check:
  cat /proc/sys/fs/binfmt_misc/qemu-aarch64
and make sure that 'interpreter' path exists under $MNT, or re-register with
the F flag:
  docker run --rm --privileged multiarch/qemu-user-static --reset -p yes"
    fi
    note "chroot executes aarch64 binaries: OK"
}

# ---------------------------------------------------------------------------
# The chroot environment.
#
# `env -i` means an explicit list, and a variable left off the list becomes
# EMPTY inside a stage that runs `set -u`, which kills it. Cross-checked
# against every ${...} the stage scripts expand:
#   FPMS_HOSTNAME FPMS_USER FPMS_HOME            00, 30, 40, 50, 90
#   PIP_PAHO PIP_NUMPY PIP_WEBSOCKET             20
#   RKNN_LITE_WHEEL_URL LIBRKNNRT_URL
#   RKNN_TOOLKIT2_TAG                            20, 25
#   FPMS_ALLOW_NPU_MISSING                       25   <- was missing
#   ROS_DISTRO                                   40, 90
#   FPMS_OS_VERSION                              90
# ---------------------------------------------------------------------------
CHROOT_ENV=(
    HOME=/root
    # /usr/local/bin ahead of the rest: pip installs console scripts there and
    # the stock root PATH on a chroot'd Ubuntu does not always include it.
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
    TERM=xterm
    DEBIAN_FRONTEND=noninteractive
    LC_ALL=C
    LANG=C.UTF-8
    FPMS_OS_VERSION="$FPMS_OS_VERSION"
    ROS_DISTRO="$ROS_DISTRO"
    ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
    RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION"
    FPMS_USER="$FPMS_USER"
    FPMS_HOME="$FPMS_HOME"
    FPMS_HOSTNAME="$FPMS_HOSTNAME"
    PIP_PAHO="$PIP_PAHO"
    PIP_NUMPY="$PIP_NUMPY"
    PIP_WEBSOCKET="$PIP_WEBSOCKET"
    RKNN_LITE_WHEEL_URL="$RKNN_LITE_WHEEL_URL"
    LIBRKNNRT_URL="$LIBRKNNRT_URL"
    RKNN_TOOLKIT2_TAG="$RKNN_TOOLKIT2_TAG"
    # scripts/25 documents an escape hatch for a host that cannot reach
    # GitHub's raw endpoint. It read this variable, which nothing passed in,
    # and a marker file that stage_all() deleted on its way past - so the
    # escape hatch could not actually be used. Both halves are fixed now.
    FPMS_ALLOW_NPU_MISSING="${FPMS_ALLOW_NPU_MISSING:-0}"
    # scripts/00 tells the operator, in the file it writes:
    #     export FPMS_SSH_PUBKEY='ssh-ed25519 AAAA...' and re-run
    # `env -i` would have swallowed that instruction whole.
    FPMS_SSH_PUBKEY="${FPMS_SSH_PUBKEY:-}"
    # scripts/90 zero-fills only when this is 1, because a zero-fill allocates
    # every free block of the image on THIS disk and cannot be undone before
    # the xz pass. Default 0. It is only safe to set on a host with the
    # headroom, and the pre-flight in prepare_image() has the numbers.
    FPMS_ZEROFILL="${FPMS_ZEROFILL:-0}"
)

in_chroot() {
    if [ "$DRY" = 1 ]; then echo "[dry-run] chroot: $*" >&2; return; fi
    chroot "$MNT" /usr/bin/env -i "${CHROOT_ENV[@]}" /bin/bash -c "$*"
}

stage_all() {
    # Payload the stages need, staged where the chroot can reach it.
    #
    # Preserve the NPU escape-hatch marker across the wipe: an operator who
    # touched it and then re-ran the build would otherwise find it gone.
    local keep_npu_marker=0
    [ -e "$MNT/opt/fpms-os/ALLOW_NPU_MISSING" ] && keep_npu_marker=1

    run "rm -rf '$MNT/opt/fpms-os'"
    run "mkdir -p '$MNT/opt/fpms-os'"
    run "cp -a '$HERE/scripts' '$HERE/overlay' '$HERE/selftest' '$HERE/docs' \
             '$HERE/firstboot' '$HERE/SPEC.md' '$MNT/opt/fpms-os/'"
    [ "$keep_npu_marker" = 1 ] && run "touch '$MNT/opt/fpms-os/ALLOW_NPU_MISSING'"

    # The rover source tree, two levels up from here. NOT silenced: the old
    # `2>/dev/null || true` swallowed "No space left on device" and turned a
    # disk-full into stage 30's much less obvious "required rover source files
    # were missing".
    run "mkdir -p '$MNT/opt/fpms-os/src'"
    if [ "$DRY" != 1 ]; then
        cp -a "$HERE/.."/*.py "$HERE/../stack" "$HERE/../nav2" "$HERE/../slam" \
              "$HERE/../STACK.md" "$MNT/opt/fpms-os/src/" \
            || die "could not stage the rover source tree into the image.
Check free space and that $HERE/../{stack,nav2,slam} exist."
    fi
    run "chmod +x '$MNT/opt/fpms-os/scripts/'*.sh"

    report_space "payload staged"

    local matched=0 s base t0 t1 el
    for s in "$MNT/opt/fpms-os/scripts/"[0-9]*.sh; do
        [ -f "$s" ] || die "no stage scripts found under $MNT/opt/fpms-os/scripts"
        base="$(basename "$s")"

        if [ -n "$ONE_STAGE" ]; then
            case "$base" in
                "$ONE_STAGE"|"$ONE_STAGE".sh|"$ONE_STAGE"-*) ;;
                *) continue ;;
            esac
        fi
        if [ -n "$FROM_STAGE" ]; then
            # Numeric compare on the NN- prefix, so --from 20 also runs 25, 30...
            [ "$(( 10#${base%%-*} ))" -lt "$(( 10#$FROM_STAGE ))" ] && continue
        fi

        matched=$(( matched + 1 ))
        space_floor "before $base"
        log "stage: $base    (free $(free_mb "$HERE") MB)"
        t0="$(date +%s)"
        in_chroot "/opt/fpms-os/scripts/$base" \
            || die "stage $base FAILED after $(( ($(date +%s) - t0) / 60 )) min.
The image is NOT usable, but it IS preserved at
    $OUT
Fix the stage and resume without losing the work already done:
    sudo $0 --from ${base%%-*}"
        t1="$(date +%s)"; el=$(( t1 - t0 ))
        log "stage $base OK in $(( el / 60 ))m$(( el % 60 ))s    (free $(free_mb "$HERE") MB)"
    done

    if [ "$matched" = 0 ]; then
        die "no stage matched --stage '${ONE_STAGE}' --from '${FROM_STAGE}'.
Available: $(cd "$HERE/scripts" && echo [0-9]*.sh)"
    fi
}

# scripts/50-overlay.sh now places overlay/boot/* onto the mounted FAT partition
# itself, which is the right place for it. This does NOT copy them again - it
# CHECKS, because the whole headless WiFi story in docs/FLASHING.md rests on
# these two files being on the partition Windows shows the operator, and a
# silent miss here is only discovered by a person holding a card that will not
# join a network. It fills a gap rather than failing the build, since by this
# point the image is otherwise finished.
verify_boot_files() {
    [ "$DRY" = 1 ] && return 0
    mountpoint -q "$MNT/boot/firmware" 2>/dev/null || return 0
    local f b missing=0
    for f in "$HERE"/overlay/boot/*; do
        [ -f "$f" ] || continue
        b="$(basename "$f")"
        if [ ! -f "$MNT/boot/firmware/$b" ]; then
            note "WARNING: $b is not on the FAT partition - placing it now"
            cp -f "$f" "$MNT/boot/firmware/$b" || missing=1
        fi
    done
    [ "$missing" = 0 ] && note "operator-visible boot files: present on the FAT partition"
    return 0
}

restore_resolv() {
    [ "$DRY" = 1 ] && return 0
    [ -f "$WORK/resolv.orig" ] || return 0
    local kind; kind="$(cut -d' ' -f1 < "$WORK/resolv.orig")"
    rm -f "$MNT/etc/resolv.conf"
    case "$kind" in
        symlink) ln -s "$(cut -d' ' -f2- < "$WORK/resolv.orig")" \
                       "$MNT/etc/resolv.conf" ;;
        file)    cp "$WORK/resolv.orig.body" "$MNT/etc/resolv.conf" ;;
        *)       : ;;
    esac
    note "restored the image's own /etc/resolv.conf (${kind})"
}

finalise() {
    log "finalising"
    run "rm -f '$MNT/usr/sbin/policy-rc.d'"
    run "rm -f '$MNT/usr/bin/qemu-aarch64-static'"
    if [ -n "$QEMU_PLACED" ] && [ "$DRY" != 1 ]; then
        rm -f "$QEMU_PLACED"
        rmdir -p "$(dirname "$QEMU_PLACED")" 2>/dev/null || true
    fi
    restore_resolv
    verify_boot_files

    # scripts/90-finalise.sh runs `fstrim /` from inside the chroot to punch the
    # image's free blocks back out of the sparse backing file. Do it again from
    # out here: it is idempotent, it costs a second, and it also covers a run
    # that stopped short of stage 90 or a stage 90 that took the opt-in
    # FPMS_ZEROFILL path. This is why mount_image() mounts with -o discard -
    # without discard support on the loop device fstrim is a no-op and the
    # zeros stay allocated, which on a 17 GB disk is the whole margin.
    if [ "$DRY" != 1 ]; then
        log "reclaiming free blocks before compressing"
        note "before: $(du -m "$OUT" | cut -f1) MB allocated of $(( $(stat -c %s "$OUT") / 1048576 )) MB apparent"
        fstrim -v "$MNT" >/dev/null 2>&1 \
            || note "fstrim unsupported here; the .img stays fully allocated"
        sync
    fi

    cleanup
    trap - EXIT INT TERM

    if [ "$DRY" != 1 ]; then
        note "image: $(du -m "$OUT" | cut -f1) MB allocated"
        report_space "before compressing"
        require_space "$COMPRESS_HEADROOM_MB" "the .xz, which exists alongside the .img"
    fi

    log "compressing (this takes a while)"
    run "xz -T0 -6 -f '$OUT'"

    # Relative filename, not the absolute path: docs/FLASHING.md tells the
    # operator to run `sha256sum -c fpms-os-....img.xz.sha256` from wherever
    # they downloaded it, and that only works if the sidecar names the file
    # relatively.
    run "(cd '$(dirname "$OUT")' && sha256sum '$(basename "$OUT").xz' \
             > '$(basename "$OUT").xz.sha256')"

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
    if [ "$DRY" != 1 ]; then
        mkdir -p "$WORK"
        # A build runs for hours under emulation and usually fails at the far
        # end of that. Keep the whole thing, timings included, on disk.
        LOGFILE="$WORK/build-$(date +%Y%m%d-%H%M%S).log"
        # stdbuf -oL so tee flushes each LINE to the file. Without it tee
        # block-buffers its file output and the last few KB - which is exactly
        # the part naming the failure - are lost when the shell exits.
        if command -v stdbuf >/dev/null 2>&1; then
            exec > >(stdbuf -oL tee -a "$LOGFILE") 2>&1
        else
            exec > >(tee -a "$LOGFILE") 2>&1
        fi
        log "logging to $LOGFILE"
    fi

    local started; started="$(date +%s)"
    check_arch
    report_space "at start"

    local resuming=0
    if [ "$FRESH" != 1 ] && { [ -n "$ONE_STAGE" ] || [ -n "$FROM_STAGE" ] \
                              || [ "$SHELL_MODE" = 1 ]; }; then
        # $OUT carries today's date. A build that starts in the evening and is
        # resumed the next morning would otherwise look at a filename that has
        # never existed and offer to start again from the vendor base - having
        # thrown away eight hours of emulated apt. Adopt the newest image of
        # this version instead, and say which one.
        if [ ! -f "$OUT" ]; then
            local prev
            prev="$(ls -1t "$HERE"/fpms-os-"${FPMS_OS_VERSION}"-*.img 2>/dev/null | head -1 || true)"
            if [ -n "$prev" ]; then
                note "no image for today; resuming the newest one instead:"
                note "$(basename "$prev")"
                OUT="$prev"
            fi
        fi
        [ -f "$OUT" ] && resuming=1
    fi

    if [ "$resuming" = 1 ]; then
        resume_image
    else
        if [ -n "$ONE_STAGE" ] || [ -n "$FROM_STAGE" ]; then
            [ "$FRESH" = 1 ] || die \
"--stage/--from with no image at
    $OUT
Run a full build first, or pass --fresh to start from the base image (which
throws away any work already in an image of a different date)."
        fi
        local base; base="$(fetch_base)"
        prepare_image "$base"
    fi
    enter_chroot_mounts

    if [ "$SHELL_MODE" = 1 ]; then
        log "chroot shell - exit when done (image will NOT be finalised)"
        chroot "$MNT" /usr/bin/env -i "${CHROOT_ENV[@]}" /bin/bash || true
        exit 0
    fi

    stage_all

    if [ -n "$ONE_STAGE" ]; then
        log "single stage complete; not finalising. The image is preserved at"
        note "$OUT"
        note "resume with:  sudo $0 --from <next stage>"
        exit 0
    fi
    finalise

    local el=$(( $(date +%s) - started ))
    log "total build time: $(( el / 3600 ))h$(( (el % 3600) / 60 ))m"
}

main "$@"
