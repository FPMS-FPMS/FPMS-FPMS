#!/usr/bin/env bash
# Stage 10 - ROS 2 Humble, Nav2, SLAM, bridges, and the micro-ROS agent.
#
# Humble is pinned, and not by preference:
#   - its rclpy extensions are cpython-310-aarch64 binaries, so the base must
#     be Ubuntu 22.04
#   - nav2_params.yaml uses progress_checker_plugin (SINGULAR), which Iron
#     renamed to progress_checker_plugins. The rename is silent - the old key
#     is simply ignored.
#   - Humble's controller_server has no odom_topic parameter; setting one on a
#     newer distro would change behaviour invisibly.
#
# THIS IS THE SLOWEST AND RISKIEST STAGE IN THE BUILD.
# =====================================================
# On an x86_64 host every command here runs under qemu-user aarch64 emulation:
# apt's dpkg unpack, ldconfig, python bytecode compilation, and - the big one -
# a full colcon C++ build of the micro-ROS agent. Expect HOURS, not minutes.
#
# MEASURED, 2026-08-11, on a WSL2/x86_64 host:
#     ==> stage 10-ros-humble.sh OK in 859m57s
# Fourteen hours and twenty minutes for this one stage, essentially all of it
# create_agent_ws.sh and build_agent.sh compiling Micro-XRCE-DDS-Agent,
# Fast-CDR and Fast-DDS from C++ source under emulation. That work is
# IDENTICAL between builds unless the micro-ROS sources or the Fast-DDS
# packages they link against move, and nothing used to preserve it - so every
# rebuild paid the fourteen hours again, and this build has been killed three
# times by unrelated causes without once reaching stage 20.
#
# It is now CACHED on the build host. See "micro-ROS workspace cache" below,
# and docs/BUILDING.md section 3.1 for the operator-facing half.
#
# Everything in here that used to end in `>/dev/null` now goes to
# /var/log/fpms-build/. A stage that can burn four hours and then die with no
# diagnostic is worse than a stage that fails fast, so every step that can fail
# says which step it was, and the micro-ROS build's compiler output is kept.
set -euo pipefail
echo "--- 10-ros-humble"

export DEBIAN_FRONTEND=noninteractive

# build.sh passes these through `env -i`; fail loudly rather than with an
# unbound-variable error 40 minutes in if this is ever run by hand.
: "${FPMS_HOME:?FPMS_HOME is not set - run this through build.sh}"
: "${FPMS_USER:?FPMS_USER is not set - run this through build.sh}"

LOGDIR=/var/log/fpms-build
mkdir -p "$LOGDIR"
APT_LOG="$LOGDIR/10-apt.log"
UROS_LOG="$LOGDIR/10-uros.log"
# Stage 90 truncates everything under /var/log, so these cost nothing in the
# shipped image but exist for the whole build.
: > "$APT_LOG"; : > "$UROS_LOG"

say()  { printf '    %s\n' "$*"; }
fatal() { printf 'FATAL: %s\n' "$*" >&2; exit 1; }

# Disk is the other thing that kills this stage. ROS + Nav2 + SLAM + the
# micro-ROS build is several GB into an image grown by only ROOTFS_GROW_MB, so
# report free space at each boundary: "it ran out of disk" must be visible in
# the log rather than inferred from a weird dpkg error.
disk() {
    printf '    disk: %s\n' \
        "$(df -Pm / | awk 'NR==2 {printf "%d MB free / %d MB total on /", $4, $2}')"
}

# Elapsed time per phase, because the only way anyone will ever know how long
# an emulated build of this actually takes is if it is written down.
_t0=$(date +%s)
phase() {
    local now; now=$(date +%s)
    printf '\n--- %s (t+%dm)\n' "$1" $(( (now - _t0) / 60 ))
}

# --- apt key ----------------------------------------------------------------
#
# curl, ca-certificates and gnupg all come from stage 00, which runs first.
#
# The key is fetched as a BINARY keyring, which is what `signed-by=` needs.
# Upstream has served this path in binary form for years, but the whole ROS
# repo goes dark the moment that changes (apt rejects an ASCII-armored file
# with a signature error on EVERY ros-humble package), and it costs one `if`
# to be immune to it. Same reasoning for the expiry check: this key ALREADY
# expired once, on 2025-06-01, and the replacement is only good until 2030.
phase "apt key and repository"
KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
TMPKEY="$(mktemp)"
curl -fsSL --retry 3 --retry-delay 5 \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$TMPKEY" \
    || fatal "could not download the ROS apt key (no network in the chroot?)"
[ -s "$TMPKEY" ] || fatal "the downloaded ROS apt key is empty"

if LC_ALL=C grep -q 'BEGIN PGP PUBLIC KEY' "$TMPKEY"; then
    say "ros.key is ASCII-armored - dearmoring (apt cannot use armor here)"
    gpg --dearmor < "$TMPKEY" > "$KEYRING" || fatal "gpg --dearmor of ros.key failed"
else
    cp "$TMPKEY" "$KEYRING"
fi
rm -f "$TMPKEY"
# Must be world-readable: apt drops to the _apt user to fetch.
chmod 0644 "$KEYRING"

# Print the key identity, and warn if it has expired. An expired key does not
# fail here - it fails as an unsigned-repository error from apt-get update -
# so the point of this block is only to make that error explainable.
KEY_INFO="$(gpg --show-keys --with-colons "$KEYRING" 2>/dev/null || true)"
# Herestrings, not pipelines. awk's `exit` would SIGPIPE a live producer and
# pipefail would put 141 into these plain assignments. A herestring is
# already-complete data, so there is nothing left running to signal.
# See apt_group() for the variant of this that actually killed a build.
KEY_EXP="$(awk -F: '$1=="pub" {print $7; exit}' <<<"$KEY_INFO" || true)"
KEY_FPR="$(awk -F: '$1=="fpr" {print $10; exit}' <<<"$KEY_INFO" || true)"
say "ROS key ${KEY_FPR:-<unreadable>}"
if [ -n "${KEY_EXP:-}" ] && [ "$KEY_EXP" -lt "$(date +%s)" ] 2>/dev/null; then
    echo "WARNING: the ROS signing key expired on $(date -d "@$KEY_EXP" 2>/dev/null || echo "$KEY_EXP")." >&2
    echo "         Every ros-humble package is about to fail to authenticate." >&2
    echo "         Upstream now ships the key as the 'ros2-apt-source' .deb;" >&2
    echo "         see the ROS signing key migration guide." >&2
fi

echo "deb [arch=$(dpkg --print-architecture) signed-by=${KEYRING}] \
http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

apt-get update -q 2>&1 | tee -a "$APT_LOG" \
    || fatal "apt-get update failed after adding the ROS repo (see $APT_LOG)"

# apt-get update does not always exit non-zero on a repo it could not verify,
# so prove the repo is actually usable before spending an hour on it.
# CAPTURE, THEN MATCH. Do NOT pipe into `grep -q` here.
#
# This gate failed a build with the repository working perfectly. Measured
# inside the image:
#
#     $ apt-cache policy ros-humble-ros-base
#     ros-humble-ros-base:
#       Installed: (none)
#       Candidate: 0.10.0-1jammy.20260804.223545
#     $ apt-cache policy ... | grep -q 'Candidate: [0-9]' ; echo $?
#     141
#
# `grep -q` exits the instant it matches, apt-cache is still writing, and it
# dies of SIGPIPE = 141. Under `set -o pipefail` the pipeline reports 141 --
# so a SUCCESSFUL match is indistinguishable from no match, and the check
# fires exactly backwards. Without pipefail the same line passes.
#
# The same trap is called out in 20-python-deps.sh and 25-npu-runtime.sh,
# where it was avoided. It was not avoided here, and it cost a build.
ROS_POLICY="$(apt-cache policy ros-humble-ros-base 2>/dev/null || true)"
case "$ROS_POLICY" in
    *"Candidate: "[0-9]*) : ;;
    *) fatal "the ROS repo is configured but ros-humble-ros-base has no candidate.
  Almost always one of: the signing key (see above), no network, or
  packages.ros.org having dropped jammy/$(dpkg --print-architecture).
  See $APT_LOG.
  Before believing it, run this INSIDE the image and read the real answer:
      apt-cache policy ros-humble-ros-base" ;;
esac
say "ros-humble-ros-base: $(awk '/Candidate:/{print $2; exit}' <<<"$ROS_POLICY" || true)"

disk

# --- installing --------------------------------------------------------------
#
# Installed in named groups, one apt invocation each, and every name is
# checked against the repo BEFORE any of it is downloaded. A single typo used
# to abort the whole stage with apt's own "Unable to locate package" and no
# indication of which of thirty names was wrong; now the group and the exact
# name are printed. Package names verified against
# packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-arm64 on 2026-08-10.
apt_group() {
    local label="$1"; shift
    local p pol missing=""
    for p in "$@"; do
        # NO PIPELINE. This line killed a build, silently.
        #
        # It used to be:
        #     cand="$(apt-cache policy "$p" 2>/dev/null | awk '/Candidate:/{print $2; exit}')"
        #
        # awk's `exit` closes the pipe while apt-cache is still writing, so
        # apt-cache dies of SIGPIPE (141). Under `set -o pipefail` the command
        # substitution carries 141, and a PLAIN assignment adopts that status,
        # so `set -e` killed the whole function on the FIRST package -- before
        # apt ran, with no output at all. The stage reported
        # "FAILED after 0 min" and the log ended mid-sentence.
        #
        # (`local cand="$(...)"` would have masked it, because `local` supplies
        # its own exit status. That difference is the entire bug, and it is why
        # the declaration above no longer initialises anything.)
        #
        # Matching on the captured text needs no subprocess and cannot race.
        pol="$(apt-cache policy "$p" 2>/dev/null || true)"
        case "$pol" in
            *"Candidate: (none)"*|"") missing="$missing $p" ;;
            *"Candidate: "*)          : ;;
            *)                        missing="$missing $p" ;;
        esac
    done
    if [ -n "$missing" ]; then
        echo "FATAL: apt group '$label' names packages that do not exist for" >&2
        echo "       jammy/$(dpkg --print-architecture) in the configured repos:" >&2
        for p in $missing; do echo "         $p" >&2; done
        exit 1
    fi
    say "installing group: $label"
    echo "=== group: $label" >> "$APT_LOG"
    # Output goes to the console as well as the log. Under emulation a single
    # group can take half an hour, and silence is indistinguishable from a hang.
    apt-get install -y -q --no-install-recommends "$@" 2>&1 | tee -a "$APT_LOG" \
        || { disk
             fatal "apt group '$label' failed to install (full output in $APT_LOG)"; }
}

phase "ROS 2 Humble core"
apt_group core \
    ros-humble-ros-base \
    ros-humble-rclpy \
    ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-nav-msgs \
    ros-humble-geometry-msgs ros-humble-diagnostic-msgs \
    ros-humble-tf2-ros ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs ros-humble-tf2-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions python3-rosdep python3-vcstool

# Two RMWs are installed on purpose (cyclonedds for A/B comparison when the
# transport is under suspicion), and that makes RMW_IMPLEMENTATION load-bearing
# rather than decorative: every unit sets it, and so does the profile below.

# A TOOLCHAIN. Not optional, and nothing above provides it: no ros-humble-*
# package depends on build-essential or g++ - only `ros-build-essential`, which
# is part of ros-dev-tools and is not installed here. Ubuntu server has no
# compiler either. Without this the micro-ROS colcon build dies with
# "No CMAKE_CXX_COMPILER could be found" AFTER the ROS install has already
# spent an hour, and (before this stage was rewritten) printed nothing at all.
#
# cmake, python3-dev, libssl-dev and libtinyxml2-dev arrive transitively via
# ros-humble-ament-cmake-core / python-cmake-module / ros-humble-fastrtps.
apt_group toolchain build-essential

# Nav2. Installed and tuned, but NOT started at boot -- see fpms-nav2.service
# for the three preconditions that are unmet.
#
# Some of these are shipped only so the metapackage is complete and must never
# be launched:
#   nav2-velocity-smoother  "the single worst thing you could add to this stack"
#   nav2-dwb-controller, nav2-smac-planner, nav2-mppi-controller  all rejected
#     with reasons in nav2_params.yaml
#   robot-localization      "NOT RUN, and should not be"
phase "Nav2 and SLAM"
apt_group nav2 \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander ros-humble-nav2-msgs \
    ros-humble-slam-toolbox

# Bridges. rosbridge is the browser's only way into ROS, and its params file
# whitelist is the safety mechanism (drive topics absent by construction).
phase "bridges"
apt_group bridges \
    ros-humble-rosbridge-suite \
    ros-humble-foxglove-bridge

# Joystick packages: installed for characterisation, but NO UNIT LAUNCHES THEM.
# teleop_twist_joy publishes /cmd_vel DIRECTLY, so running it means two writers
# on /cmd_vel -- stop fpms-teleop first or you will have exactly the race the
# masked units exist to prevent.
apt_group joystick \
    ros-humble-joy ros-humble-teleop-twist-joy

# Reclaim the .deb archives now rather than at stage 90. Nav2 alone is well
# over a gigabyte of downloads and the micro-ROS build still has to fit.
# Only the archives go: /var/lib/apt/lists must survive, because stage 20
# installs without running apt-get update again.
apt-get clean
disk

# ros-humble-robot-state-publisher IS installed, and cannot be avoided:
# ros-humble-ros-base depends on it. (An earlier version of this comment
# claimed it was not installed, which was simply wrong.) What holds is the
# thing that actually matters: NO UNIT LAUNCHES IT. There is no URDF in this
# project; the whole tree below base_footprint is static_transform_publisher,
# and a second publisher on base_footprint->base_link->laser_frame is
# something TF_TREE.md forbids absolutely.
#
# NOT installed: ros-humble-xacro. Nothing generates a URDF here.
#
# NOT installed: python3-transforms3d / tf_transformations. It is deliberately
# avoided in fpms_missions.py because it was installed-and-broken on the old
# Pi. The rover computes quaternions inline, in two lines.

# --- micro-ROS agent --------------------------------------------------------
#
# NOT an apt package, and there is no prebuilt alternative that fits: verified
# against the jammy/arm64 index on 2026-08-10, packages.ros.org has
# micro-ros-msgs and micro-ros-diagnostic-* but NO micro-ros-agent and no
# microxrcedds-agent. The snap and the Docker image both exist upstream but
# neither gives us `ros2 run micro_ros_agent`, which is exactly what
# /usr/local/bin/fpms-uros-agent-run execs. So it is built from source.
#
# micro-ros-agent.service sources ${FPMS_HOME}/uros_ws/install/setup.bash under
# `set -e`, so if this workspace is not built the wrapper dies instantly and the
# drive link never comes up.
phase "micro-ROS agent workspace"
UROS_WS="${FPMS_HOME}/uros_ws"
UROS_STAMP="$UROS_WS/.fpms-built"

# Where colcon puts a node executable for an ament_cmake package, without
# assuming the install layout.
#
# `find | head -n1` SIGPIPEs find, and under pipefail the pipeline reports 141;
# the trailing `|| true` binds to the whole pipeline and is what makes this
# safe. Do not remove it. (See apt_group() for the version of this that killed
# a build.)
uros_agent_bin() {
    find "$UROS_WS/install" -type f -name micro_ros_agent -perm -u+x 2>/dev/null \
        | head -n1 || true
}

# Is this file an aarch64 ELF64?
#
# Checked BY BYTES, the same four facts 25-npu-runtime.sh checks on
# librknnrt.so: \x7fELF, ELFCLASS64, little-endian, and e_machine == 183
# (EM_AARCH64) in the two bytes at offset 18. `file` is not installed in this
# chroot; od is coreutils and cannot be missing.
#
# No pipeline anywhere: od writes a fixed twenty bytes, so the capture is
# already-complete data and `set --` word-splits it into positional parameters.
# ${19} and ${20} MUST be braced - $19 means $1 followed by a literal 9, which
# would silently compare "7f9" against "b7" and pass nothing, ever.
elf_is_aarch64() {
    local f="$1" hex
    [ -f "$f" ] || return 1
    if ! command -v od >/dev/null 2>&1; then
        echo "WARNING: od is not installed, so the architecture of" >&2
        echo "         $f cannot be verified. Treating it as UNVERIFIED," >&2
        echo "         which counts as a failure - a wrong-architecture agent" >&2
        echo "         binary is the exact thing this check exists to stop." >&2
        return 1
    fi
    hex="$(od -An -tx1 -N20 -- "$f" 2>/dev/null || true)"
    [ -n "$hex" ] || return 1
    set -- $hex
    [ "$#" -eq 20 ] || return 1                 # shorter than an ELF header
    [ "$1$2$3$4" = "7f454c46" ] || return 1     # \x7fELF
    [ "$5" = "02" ] || return 1                 # ELFCLASS64
    [ "$6" = "01" ] || return 1                 # ELFDATA2LSB
    if [ "${19}" = "b7" ] && [ "${20}" = "00" ]; then return 0; fi
    return 1                                    # e_machine is not EM_AARCH64
}

# THE definition of "this workspace is usable", used by the resume check, by
# the cache restore, and by the cache write.
#
# Checking install/setup.bash is not enough and never was: the FIRST colcon
# build - micro_ros_setup itself - creates it, so it exists even when the agent
# never built. /usr/local/bin/fpms-uros-agent-run execs
# `ros2 run micro_ros_agent micro_ros_agent`, so THAT executable, and its
# architecture, is the whole claim.
uros_ws_ok() {
    local b
    [ -f "$UROS_WS/install/setup.bash" ] || return 1
    b="$(uros_agent_bin)"
    [ -n "$b" ] || return 1
    elf_is_aarch64 "$b" || return 1
    return 0
}

# Dump whatever colcon actually said. This is the whole point of the rewrite:
# a C++ build that can run for hours must never fail silently.
uros_dump_failure() {
    echo "----- last 60 lines of $UROS_LOG -----" >&2
    tail -n 60 "$UROS_LOG" >&2
    local f
    for f in "$UROS_WS"/log/latest_build/*/stdout_stderr.log; do
        [ -f "$f" ] || continue
        grep -qiE 'error|fatal' "$f" || continue
        echo "----- ${f} (tail) -----" >&2
        tail -n 40 "$f" >&2
    done
    disk
}

# Emulated parallel compiles thrash: every g++ is a qemu process holding real
# host memory, and Fast-DDS headers peak well over a gigabyte per translation
# unit. Budget two gigabytes per job, cap at four, never go below one.
# /proc is bind-mounted from the host, so this sees the host's real CPUs and RAM
# - which is correct, because that is what the emulation actually runs on.
UROS_JOBS="$(nproc 2>/dev/null || echo 1)"
_memkb="$(awk '/^MemTotal:/ {print $2; exit}' /proc/meminfo 2>/dev/null || echo 0)"
if [ "${_memkb:-0}" -gt 0 ]; then
    _bymem=$(( _memkb / 2000000 ))
    if [ "$_bymem" -lt 1 ]; then _bymem=1; fi
    if [ "$_bymem" -lt "$UROS_JOBS" ]; then UROS_JOBS="$_bymem"; fi
fi
if [ "$UROS_JOBS" -gt 4 ]; then UROS_JOBS=4; fi
if [ "$UROS_JOBS" -lt 1 ]; then UROS_JOBS=1; fi
export MAKEFLAGS="-j${UROS_JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$UROS_JOBS"
say "colcon: 1 package at a time, ${UROS_JOBS} compile job(s) per package"
say "log: $UROS_LOG"

# --- micro-ROS workspace cache ------------------------------------------------
#
# WHY
# ===
# 859m57s, measured. Essentially all of it is the two colcon invocations below,
# and the result is a pure function of its inputs. Nothing preserved it, so
# every rebuild paid it again. It is now packed to the BUILD HOST and restored
# when the inputs are unchanged, turning fourteen hours into a few minutes.
#
# WHERE IT LIVES, AND THE ONE build.sh CHANGE THIS NEEDS
# ======================================================
# The cache MUST live outside the image: a cache inside the image dies with the
# image, which is the thing being rebuilt. The chroot can only see the host
# through a mount, and this stage does not get to create one - build.sh owns
# the mounts, and build.sh is not this file's to edit. Note that
# /opt/fpms-os is a `cp -a` COPY made by stage_all(), not a mount: anything
# written there lands inside the image and is lost with it.
#
# So the directory is DISCOVERED, in this order, and its absence is not an
# error:
#
#   $FPMS_UROS_CACHE_DIR      explicit; for a hand run inside `build.sh --shell`
#   /opt/fpms-cache/uros      the bind mount build.sh should provide
#   /opt/fpms-os/.cache/uros  if .cache is ever added to stage_all()'s cp list
#                             (restore only - see uros_cache_external)
#
# The build.sh change, which is two lines and has NOT been made here:
#
#     # enter_chroot_mounts(), alongside the other bind mounts:
#     mkdir -p "$CACHE/uros" "$MNT/opt/fpms-cache"
#     mount --bind "$CACHE" "$MNT/opt/fpms-cache"
#
#     # cleanup()'s unmount loop, which must release it BEFORE "$MNT" itself:
#     for m in opt/fpms-cache dev/pts dev proc sys run boot/firmware; do
#
# Until that lands, restore misses and populate is skipped, each with a printed
# reason, and this stage builds from source exactly as it always did. Caching
# is an accelerator; it is never a dependency.
#
# WHAT THE KEY COVERS, AND WHY EACH LINE IS IN IT
# ===============================================
# A stale uros_ws is far worse than a slow build. An agent linked against
# different Fast-DDS headers than the ones the image ships does not fail here -
# it fails on the rover, as a micro-ROS link that never establishes, which is a
# diagnosis this project has repeatedly got wrong. So the key is the actual ABI
# surface, not a timestamp:
#
#   schema      bumped by hand when the cache FORMAT changes, so entries
#               written by older code can never be misread by newer code
#   distro      $ROS_DISTRO
#   arch        dpkg architecture, and uname -m as `machine`. An x86_64 tarball
#   machine     restored into an aarch64 image is the nightmare case; it is
#               keyed out here AND re-checked on the binary itself, by bytes.
#   os          ID-VERSION_ID of the base rootfs: the glibc/libstdc++ era.
#               build.sh pins the base image by sha256, so this moves only when
#               someone changes the base on purpose.
#   gcc         the compiler that produced the objects
#   fastrtps    Fast-DDS, and fastcdr with it: the libraries the agent links
#   fastcdr     and whose headers it compiled against. THE point of this key.
#   rmw-fastrtps-cpp
#   rmw-fastrtps-shared-cpp
#   rclcpp      the rest of the C++ ABI the agent is built against
#   micro_ros_setup   the exact commit. Resolved with `git ls-remote` BEFORE
#               the clone when deciding whether to restore, and re-read with
#               `rev-parse HEAD` from the real checkout when an entry is
#               WRITTEN - so a stored key always names the commit that was
#               actually built, not the one we hoped for.
#
# WHAT THE KEY DOES NOT COVER, stated plainly rather than left to be discovered
# on the rover:
#   create_agent_ws.sh vcs-imports Micro-XRCE-DDS-Agent, micro-ROS-Agent and
#   micro_ros_msgs BY BRANCH, from a .repos file inside micro_ros_setup.
#   Pinning micro_ros_setup's commit pins that file, but not where the branches
#   it names point today. A cache entry can therefore hold an agent built from
#   slightly older upstream commits than a fresh build would produce. That is
#   staleness of DEGREE, not of ABI - it cannot produce the linked-against-the-
#   wrong-Fast-DDS failure above, because every library the agent links comes
#   from the apt packages that ARE keyed. The resolved commit of every imported
#   repo is recorded in the entry's .info file and printed on every restore.
#   When upstream moves and you want it: delete the entry, or FPMS_UROS_NOCACHE=1.
UROS_CACHE_SCHEMA=1
UROS_SETUP_URL="https://github.com/micro-ROS/micro_ros_setup.git"
UROS_SETUP_BRANCH="humble"
UROS_CACHE_DIR=""
UROS_CACHE_TAR=""
UROS_CACHE_KEYFILE=""
UROS_CACHE_INFO=""
UROS_CACHE_KEY=""
UROS_FROM_CACHE=0
UROS_SOURCE="built from source"

uros_cache_paths() {
    UROS_CACHE_TAR="$UROS_CACHE_DIR/uros_ws.tar.gz"
    UROS_CACHE_KEYFILE="$UROS_CACHE_DIR/uros_ws.key"
    UROS_CACHE_INFO="$UROS_CACHE_DIR/uros_ws.info"
}

uros_cache_locate() {
    local d
    for d in "${FPMS_UROS_CACHE_DIR:-}" /opt/fpms-cache/uros /opt/fpms-os/.cache/uros; do
        [ -n "$d" ] || continue
        [ -d "$d" ] || continue
        UROS_CACHE_DIR="$d"; uros_cache_paths; return 0
    done
    # The bind mount is there but empty: this is the first build on this host.
    if [ -d /opt/fpms-cache ]; then
        mkdir -p /opt/fpms-cache/uros 2>/dev/null || return 1
        UROS_CACHE_DIR=/opt/fpms-cache/uros; uros_cache_paths; return 0
    fi
    return 1
}

# "Outside the image" is not a promise anyone made - it is a property that can
# be MEASURED. A host directory reached through a bind mount is on a different
# filesystem from the image's /, so its st_dev differs. If they match, the
# directory is inside the image: writing a few hundred MB of tarball there
# would ship it in the .img, cache nothing, and die with the image. Refuse.
uros_cache_external() {
    local a b
    a="$(stat -c %d "$UROS_CACHE_DIR" 2>/dev/null || true)"
    b="$(stat -c %d / 2>/dev/null || true)"
    [ -n "$a" ] || return 1
    [ -n "$b" ] || return 1
    [ "$a" != "$b" ] || return 1
    return 0
}

# dpkg-query exits 1 for an unknown package and 0-with-empty-output for a known
# but uninstalled one. Capture, then decide; neither may kill the stage.
# The single quotes on -f are load-bearing: ${Version} is dpkg's, not bash's.
uros_pkg_ver() {
    local p="$1" v
    v="$(dpkg-query -W -f='${Version}' "$p" 2>/dev/null || true)"
    [ -n "$v" ] || v="absent"
    printf 'pkg:%s=%s\n' "$p" "$v"
}

# The commit the clone WILL resolve to. `git ls-remote` prints
# "<sha>\trefs/heads/humble"; strip at the first non-hex character instead of
# piping into awk or cut, then gate on the length - 40 (sha1) or 64 (sha256).
# Anything else, including git's own error text on a network failure, fails the
# gate, and a key that cannot be computed means NO RESTORE. That direction is
# deliberate: an unkeyable cache must never be used.
uros_remote_sha() {
    local out sha
    out="$(git ls-remote "$UROS_SETUP_URL" "refs/heads/${UROS_SETUP_BRANCH}" 2>/dev/null || true)"
    sha="${out%%[!0-9a-f]*}"
    case "${#sha}" in 40|64) printf '%s\n' "$sha"; return 0 ;; esac
    return 1
}

# The commit that was ACTUALLY cloned. Used when WRITING an entry, so that a
# race between ls-remote and the clone cannot store a key naming the wrong tree.
uros_local_sha() {
    local sha
    sha="$(git -C "$UROS_WS/src/micro_ros_setup" rev-parse HEAD 2>/dev/null || true)"
    case "${#sha}" in 40|64) printf '%s\n' "$sha"; return 0 ;; esac
    return 1
}

uros_cache_key() {   # uros_cache_key [<micro_ros_setup sha>]
    local sha="${1:-}" arch mach os gccv k p
    if [ -z "$sha" ]; then sha="$(uros_remote_sha || true)"; fi
    if [ -z "$sha" ]; then return 1; fi
    arch="$(dpkg --print-architecture 2>/dev/null || true)"; [ -n "$arch" ] || arch="unknown"
    mach="$(uname -m 2>/dev/null || true)";                  [ -n "$mach" ] || mach="unknown"
    gccv="$(gcc -dumpfullversion 2>/dev/null || true)";      [ -n "$gccv" ] || gccv="absent"
    # The -r guard inside the subshell is not defensive padding: dash's `.` on
    # a missing file exits ON THE SPOT, taking the printf - and therefore the
    # fallback value - with it. Proved in a scratch shell before it went in.
    os="$(sh -c 'if [ -r /etc/os-release ]; then . /etc/os-release; fi
                 printf "%s-%s" "${ID:-unknown}" "${VERSION_ID:-unknown}"' || true)"
    [ -n "$os" ] || os="unknown"

    k="schema=${UROS_CACHE_SCHEMA}
distro=${ROS_DISTRO:-humble}
arch=${arch}
machine=${mach}
os=${os}
gcc=${gccv}"
    for p in fastrtps fastcdr rmw-fastrtps-cpp rmw-fastrtps-shared-cpp rclcpp; do
        k="${k}
$(uros_pkg_ver "ros-${ROS_DISTRO:-humble}-${p}")"
    done
    printf '%s\nmicro_ros_setup=%s\n' "$k" "$sha"
}

# Human-readable provenance, written alongside the entry. NOT authoritative:
# the .key file is what decides a hit, and this is what a person reads when
# they want to know where an agent binary came from.
uros_cache_info() {
    local b g d s
    b="$(uros_agent_bin)"
    printf 'micro-ROS workspace cache entry\n'
    printf '  written    %s\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
    printf '  workspace  %s\n' "$UROS_WS"
    printf '  agent      %s\n' "${b:-<none>}"
    printf '  sources    (resolved commits. The key pins micro_ros_setup only -\n'
    printf '              the repos it imports track branches, so these are\n'
    printf '              recorded rather than keyed.)\n'
    while IFS= read -r g; do
        [ -n "$g" ] || continue
        d="${g%/.git}"
        s="$(git -C "$d" rev-parse HEAD 2>/dev/null || true)"
        printf '    %-34s %s\n' "${d#"$UROS_WS/src/"}" "${s:-<not a git checkout>}"
    done < <(find "$UROS_WS/src" -maxdepth 4 -name .git 2>/dev/null || true)
    return 0
}

# Restore. Returns 0 only when a verified workspace is on disk; every other
# path returns 1 with a printed reason and leaves nothing behind, so the caller
# falls through to a normal build.
#
# Called from an `if` condition, which suppresses errexit for the whole body -
# so every command that can fail carries its own guard rather than relying on
# `set -e` to be either present or absent.
uros_cache_restore() {
    local rc=0 stored b
    if [ "${FPMS_UROS_NOCACHE:-0}" = "1" ]; then
        say "cache: restore disabled by FPMS_UROS_NOCACHE=1"
        return 1
    fi
    if ! uros_cache_locate; then
        say "cache: no cache directory is visible in the chroot - building from source."
        say "cache: build.sh needs one bind mount for this; docs/BUILDING.md 3.1."
        return 1
    fi
    if [ -e "$UROS_CACHE_DIR/NOCACHE" ]; then
        say "cache: $UROS_CACHE_DIR/NOCACHE exists - restore disabled by the operator"
        return 1
    fi
    say "cache: $UROS_CACHE_DIR"

    # Existence first, key second: computing the key costs a `git ls-remote`
    # round trip, and there is nothing to compare it against on the first build
    # on a host. An entry is only complete when BOTH files are present - see
    # the publish order in uros_cache_store for why that is the safe half.
    if [ ! -f "$UROS_CACHE_TAR" ] || [ ! -f "$UROS_CACHE_KEYFILE" ]; then
        say "cache: MISS - no complete entry yet. This build will write one."
        return 1
    fi

    UROS_CACHE_KEY="$(uros_cache_key || true)"
    if [ -z "$UROS_CACHE_KEY" ]; then
        say "cache: cannot resolve ${UROS_SETUP_BRANCH} of micro_ros_setup (network?),"
        say "cache: so no entry can be keyed. Refusing to restore on an unknown commit."
        return 1
    fi

    stored="$(cat "$UROS_CACHE_KEYFILE" 2>/dev/null || true)"
    if [ "$stored" != "$UROS_CACHE_KEY" ]; then
        say "cache: MISS - the key moved. Building from source (14+ hours)."
        say "cache: the two keys follow; the differing line is the reason."
        printf '      cached key:\n' >&2
        printf '%s\n' "$stored" | sed 's/^/        /' >&2
        printf '      current key:\n' >&2
        printf '%s\n' "$UROS_CACHE_KEY" | sed 's/^/        /' >&2
        return 1
    fi

    say "cache: HIT - the stored key matches the current one byte for byte"
    if [ -f "$UROS_CACHE_INFO" ]; then
        sed 's/^/      /' "$UROS_CACHE_INFO" >&2 || true
    fi

    # colcon bakes ABSOLUTE paths into setup.bash and the installed .cmake
    # files, so this tree may only ever be restored to the directory it was
    # packed from. The tarball holds one top-level "uros_ws" and is unpacked at
    # $FPMS_HOME, which makes that structural rather than a convention.
    rm -rf "$UROS_WS"
    mkdir -p "$FPMS_HOME"
    rc=0
    tar -xzf "$UROS_CACHE_TAR" -C "$FPMS_HOME" >>"$UROS_LOG" 2>&1 || rc=$?
    if [ "$rc" != 0 ]; then
        say "cache: tar exited ${rc} unpacking the entry - discarding it and building."
        tail -n 10 "$UROS_LOG" >&2 || true
        rm -rf "$UROS_WS"
        return 1
    fi

    # VERIFY WHAT CAME BACK. A tarball that unpacked cleanly proves nothing
    # whatever about what is inside it.
    if ! uros_ws_ok; then
        say "cache: the restored workspace FAILED verification - discarding it."
        if [ -f "$UROS_WS/install/setup.bash" ]; then
            say "cache:   install/setup.bash: present"
        else
            say "cache:   install/setup.bash: MISSING"
        fi
        b="$(uros_agent_bin)"
        if [ -n "$b" ]; then
            say "cache:   micro_ros_agent:   $b (not an aarch64 ELF64)"
        else
            say "cache:   micro_ros_agent:   MISSING"
        fi
        rm -rf "$UROS_WS"
        return 1
    fi

    b="$(uros_agent_bin)"
    say "cache: RESTORED $UROS_WS"
    say "cache: agent: $b"
    say "cache: verified: install/setup.bash present, micro_ros_agent is an aarch64 ELF64"
    UROS_FROM_CACHE=1
    # Provenance the artefact carries: the resume check prints this stamp, and
    # so does anyone who mounts the finished .img and cats it.
    printf '%s (restored from cache %s)\n' \
        "$(cat "$UROS_STAMP" 2>/dev/null || echo unknown)" \
        "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" > "$UROS_STAMP" || true
    return 0
}

# Populate. Never fatal: a build that produced a correct image must not fail
# because a cache could not be written. Every refusal says why.
uros_cache_store() {
    local rc=0 tmp sha size
    if [ "${FPMS_UROS_NOCACHE:-0}" = "1" ]; then
        return 0
    fi
    if [ "$UROS_FROM_CACHE" = "1" ]; then
        return 0                       # it came from there; rewriting it buys nothing
    fi
    if [ -z "$UROS_CACHE_DIR" ]; then
        if ! uros_cache_locate; then
            say "cache: NOT populated - no cache directory is visible in the chroot."
            say "cache: this build's ~14 hours will have to be paid again next time."
            say "cache: docs/BUILDING.md 3.1 has the two-line build.sh change."
            return 0
        fi
    fi
    if [ -e "$UROS_CACHE_DIR/NOCACHE" ]; then
        say "cache: not populated - $UROS_CACHE_DIR/NOCACHE exists"
        return 0
    fi
    # ONLY AFTER A VERIFIED BUILD. Writing an entry for a workspace we have not
    # just proved good is the one mistake this whole design exists to prevent.
    if ! uros_ws_ok; then
        say "cache: not populated - the workspace does not verify"
        return 0
    fi
    if ! uros_cache_external; then
        say "cache: $UROS_CACHE_DIR is on the image's own filesystem, not the host's."
        say "cache: NOT populating - the tarball would be shipped inside the .img and"
        say "cache: would be deleted with it. Provide the bind mount instead."
        return 0
    fi
    if [ ! -w "$UROS_CACHE_DIR" ]; then
        say "cache: not populated - $UROS_CACHE_DIR is not writable"
        return 0
    fi

    # Key on the commit that was ACTUALLY built, falling back to the remote
    # only if this workspace has no git checkout to ask.
    sha="$(uros_local_sha || true)"
    if [ -z "$sha" ]; then sha="$(uros_remote_sha || true)"; fi
    UROS_CACHE_KEY="$(uros_cache_key "$sha" || true)"
    if [ -z "$UROS_CACHE_KEY" ]; then
        say "cache: not populated - the key could not be computed"
        return 0
    fi
    if [ -f "$UROS_CACHE_KEYFILE" ] && [ -f "$UROS_CACHE_TAR" ]; then
        if [ "$(cat "$UROS_CACHE_KEYFILE" 2>/dev/null || true)" = "$UROS_CACHE_KEY" ]; then
            say "cache: the stored entry is already current - nothing to write"
            return 0
        fi
    fi

    phase "cache: packing the workspace for the next build"
    say "cache: -> $UROS_CACHE_TAR"
    tmp="$UROS_CACHE_DIR/.uros_ws.$$.tmp"
    rm -f "$tmp"
    rc=0
    tar -czf "$tmp" -C "$FPMS_HOME" uros_ws || rc=$?
    # tar exits 1 for "some files differ / changed as we read them" and 2 for
    # fatal. NEITHER is good enough to publish. An entry we are unsure of is
    # worse than no entry at all - the cost of no entry is time, and the cost
    # of a wrong entry is a rover whose micro-ROS link never comes up.
    if [ "$rc" != 0 ]; then
        say "cache: tar exited ${rc} - NOT publishing an entry we cannot vouch for"
        rm -f "$tmp"
        return 0
    fi

    # Publish in the order that makes any torn write a MISS rather than a
    # mismatch: drop the key first, move the tarball into place, write the key
    # last. Restore demands BOTH files AND a byte-identical key, so every
    # intermediate state here fails safe - including a build killed mid-write,
    # which this build has been three times.
    rm -f "$UROS_CACHE_KEYFILE"
    if ! mv -f "$tmp" "$UROS_CACHE_TAR"; then
        say "cache: could not move the new tarball into place - entry left absent"
        rm -f "$tmp"
        return 0
    fi
    uros_cache_info > "$UROS_CACHE_INFO" 2>/dev/null || true
    if ! printf '%s\n' "$UROS_CACHE_KEY" > "$UROS_CACHE_KEYFILE"; then
        say "cache: could not write the key file; the entry stays unusable (a MISS)"
        rm -f "$UROS_CACHE_KEYFILE"
        return 0
    fi

    size="$(stat -c %s "$UROS_CACHE_TAR" 2>/dev/null || true)"
    [ -n "$size" ] || size=0
    say "cache: WROTE $(( size / 1048576 )) MB"
    say "cache: the next build whose key matches skips ~14 hours of compiling"
    # NOT disk(): that reports the IMAGE's filesystem, and the cache is on the
    # host's. This is the number that fills up and kills the next build.
    printf '    cache disk: %s\n' \
        "$(df -Pm "$UROS_CACHE_DIR" 2>/dev/null \
           | awk 'NR==2 {printf "%d MB free on the host filesystem holding the cache", $4}' || true)"
    return 0
}

# Idempotency, and it is worth real money here: `build.sh --from 10` re-run
# after a later stage failed must not spend another fourteen hours rebuilding a
# workspace that is already good. The stamp is written ONLY after the agent
# executable has been verified, so a half-built workspace is never mistaken
# for a finished one.
#
# Three ways to arrive at a built workspace, most-local first:
#   1. it is already in this image  (a resumed build)
#   2. it is in the host cache and the key matches
#   3. compile it, which is the fourteen hours
if [ -f "$UROS_STAMP" ] && uros_ws_ok; then
    say "workspace already built on $(cat "$UROS_STAMP") - skipping"
    say "delete $UROS_STAMP to force a rebuild"
    UROS_SOURCE="already present in this image (resumed build)"
elif uros_cache_restore; then
    UROS_SOURCE="RESTORED FROM THE HOST CACHE - no C++ was compiled"
else
    rm -rf "$UROS_WS"; mkdir -p "$UROS_WS/src"

    # ROS's setup.bash references unset variables and can return non-zero on paths
    # that are not errors, so both `set -e` and `set -u` come off across it.
    set +eu; . /opt/ros/humble/setup.bash; set -eu

    # One network hiccup in a four-hour build should not cost the whole stage.
    retry() {
        local n=0
        until "$@"; do
            n=$((n + 1))
            if [ "$n" -ge 3 ]; then return 1; fi
            echo "    retry $n/3: $*" >&2
            sleep 5
        done
        return 0
    }

    # URL and branch come from the variables the cache key is computed from.
    # If these two ever disagree, the key would name a commit on a repo that
    # was never cloned - which is precisely the class of silent staleness the
    # cache is built to make impossible.
    retry git clone -q --depth 1 -b "$UROS_SETUP_BRANCH" \
        "$UROS_SETUP_URL" "$UROS_WS/src/micro_ros_setup" \
        || fatal "could not clone micro_ros_setup (network? github?)"

    # rosdep is NOT optional and its failure must not be silent. create_agent_ws.sh
    # runs `rosdep install` internally, under `set -e`: with an uninitialised or
    # stale rosdep database that fails as an unresolved key, minutes later, in a
    # script nobody has open. Both of these used to be `|| true`.
    #
    # `rosdep init` is the one that legitimately "fails" on a re-run, so test for
    # its output file instead of swallowing every error it can produce.
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        say "rosdep already initialised"
    else
        rosdep init >>"$UROS_LOG" 2>&1 \
            || { tail -n 20 "$UROS_LOG" >&2
                 fatal "rosdep init failed - it needs network and write access to /etc/ros/rosdep"; }
    fi

    # Running this as root prints a recommendation not to; that is expected and
    # harmless in a chroot where everything is root. The cache lands in /root/.ros,
    # so the `ubuntu` user on the flashed image gets no rosdep cache - which is
    # fine, nothing at runtime uses rosdep.
    retry rosdep update --rosdistro humble >>"$UROS_LOG" 2>&1 \
        || { tail -n 20 "$UROS_LOG" >&2
             fatal "rosdep update failed after 3 attempts - the dependency database is
      empty, and create_agent_ws.sh would fail later with an unresolved key
      instead of this message. See $UROS_LOG."; }

    # This one stays non-fatal: micro_ros_setup's only dependency is ament_cmake,
    # which is already installed, so a rosdep hiccup here is recoverable. It is
    # loud now instead of invisible.
    if ! (cd "$UROS_WS" && rosdep install --from-paths src --ignore-src -y >>"$UROS_LOG" 2>&1); then
        echo "WARNING: rosdep install for micro_ros_setup failed; continuing because its" >&2
        echo "         dependencies are already satisfied by the apt groups above." >&2
        tail -n 15 "$UROS_LOG" >&2
    fi

    # No --symlink-install. It buys nothing for a workspace that is built once and
    # then shipped, and it would leave install/ pointing into build/ - which is
    # what makes it safe to delete build/ at the end and reclaim the space.
    #
    # console_direct+ streams every compiler line to the terminal AND the log.
    # It is verbose, and that is the point: colcon's default handlers print two
    # lines per package, which over a multi-hour emulated build is
    # indistinguishable from a hang, and on failure they print nothing useful.
    phase "colcon: micro_ros_setup"
    if ! (cd "$UROS_WS" && colcon build \
            --parallel-workers "$UROS_JOBS" \
            --event-handlers console_direct+) 2>&1 | tee -a "$UROS_LOG"; then
        uros_dump_failure
        fatal "colcon build of micro_ros_setup failed"
    fi
    set +eu; . "$UROS_WS/install/setup.bash"; set -eu

    # create_agent_ws.sh vcs-imports Micro-XRCE-DDS-Agent, micro-ROS-Agent and
    # micro_ros_msgs and then runs rosdep over them.
    phase "create_agent_ws"
    if ! (cd "$UROS_WS" && ros2 run micro_ros_setup create_agent_ws.sh) 2>&1 | tee -a "$UROS_LOG"; then
        uros_dump_failure
        fatal "create_agent_ws.sh failed (it needs network for vcs import and a working rosdep)"
    fi
    apt-get clean          # rosdep install inside that script downloads .debs too
    disk

    # THE LONG ONE. build_agent.sh is `colcon build --packages-up-to
    # micro_ros_agent $@ --cmake-args ...`, so extra colcon flags can be passed
    # through here - but NOT --cmake-args, which the script appends itself and
    # which argparse would let it overwrite (taking -DUAGENT_BUILD_EXECUTABLE=OFF
    # with it). Hence parallelism via --parallel-workers and MAKEFLAGS only.
    #
    # No CMAKE_BUILD_TYPE is forced: an unoptimised build compiles appreciably
    # faster under emulation, and this process is blocked on a 230400-baud serial
    # link at runtime, not on the CPU.
    #
    # Under qemu this is measured in hours. If it looks hung, tail the log before
    # assuming it is:  tail -f $UROS_LOG
    phase "colcon: micro_ros_agent (this is the multi-hour one)"
    if ! (cd "$UROS_WS" && ros2 run micro_ros_setup build_agent.sh \
            --parallel-workers "$UROS_JOBS" \
            --event-handlers console_direct+) 2>&1 | tee -a "$UROS_LOG"; then
        uros_dump_failure
        fatal "build_agent.sh failed - the micro-ROS agent did not build.
      The compiler output above is the reason; the full log is $UROS_LOG
      and the per-package logs are under $UROS_WS/log/."
    fi

    # Verify, because the wrapper's `set -e` makes a missing setup.bash an
    # instant, silent death of the entire drive link. Checking only setup.bash is
    # not enough: the first colcon build creates it, so it exists even if the
    # agent itself never built. Check for the executable the wrapper actually
    # execs -- `ros2 run micro_ros_agent micro_ros_agent`.
    [ -f "$UROS_WS/install/setup.bash" ] \
        || { uros_dump_failure; fatal "micro-ROS workspace did not build (no install/setup.bash)"; }
    AGENT_BIN="$(uros_agent_bin)"
    [ -n "$AGENT_BIN" ] \
        || { uros_dump_failure
             fatal "micro-ROS workspace built but there is no micro_ros_agent executable.
      fpms-uros-agent-run would fail at 'ros2 run micro_ros_agent micro_ros_agent'."; }
    # And it must be for the TARGET, not the host. Under a broken binfmt setup,
    # or a colcon that picked up a host toolchain, this stage can produce an
    # x86_64 binary that is present, executable, and useless on the rover. It
    # is also the gate that decides whether this workspace is fit to be cached.
    elf_is_aarch64 "$AGENT_BIN" \
        || { uros_dump_failure
             fatal "micro_ros_agent was built, but it is not an aarch64 ELF64.
      $AGENT_BIN
      Something compiled for the build host, not the target. Check that
      qemu-aarch64 binfmt is registered and that no host toolchain leaked
      into the chroot. Read the first bytes yourself:
          od -An -tx1 -N20 '$AGENT_BIN'
      Byte 4 must be 02 (ELF64) and bytes 18-19 must be 'b7 00' (EM_AARCH64)."; }
    say "agent: $AGENT_BIN (aarch64 ELF64, verified)"

    date -u '+%Y-%m-%dT%H:%M:%SZ' > "$UROS_STAMP"

    # Reclaim the intermediates. Safe precisely because the builds above did not
    # use --symlink-install, so nothing in install/ points into build/. src/ stays:
    # it is small next to build/ and it is what makes the agent debuggable on the
    # rover.
    rm -rf "$UROS_WS/build" "$UROS_WS/log"
    say "install tree: $(du -sh "$UROS_WS/install" 2>/dev/null | cut -f1)"
fi   # end of the build-if-not-already-built block

# Hand the next build the fourteen hours. Guarded with `|| true` on top of a
# function that already returns 0 on every refusal: a finished, verified image
# must never fail because a cache could not be written.
uros_cache_store || true

# One line, in the build log, that settles "did it rebuild or did it restore?"
# without anyone having to read timings or guess.
say "micro-ROS workspace: ${UROS_SOURCE}"

chown -R "${FPMS_USER}:${FPMS_USER}" "$UROS_WS"
disk

# --- shell environment ------------------------------------------------------
#
# The units set all of this explicitly and do NOT rely on the profile -- an
# interactive shell's environment must never be what makes a service work.
# This is for the operator SSHing in to debug, so that an ad-hoc
# `ros2 topic list` sees what the services see.
#
# Every ad-hoc `ros2 topic list` in this project's history that forgot
# ROS_DOMAIN_ID saw an EMPTY LIST and concluded the link was dead.
#
# It must also be HARMLESS, because /etc/profile.d is sourced by every login
# shell including the `bash -lc` in fpms-rosbridge and fpms-console: ROS's
# setup.bash reads unset variables and can return non-zero, so this file saves
# the caller's shell options, drops -e and -u across the sourcing, and puts
# them back. A profile that aborts a `bash -lc` kills the service before its
# ExecStart is ever reached.
phase "shell environment"
cat > /etc/profile.d/fpms-ros.sh <<'EOF'
# FPMS-OS interactive ROS environment.
# The services do NOT depend on this file -- they set everything in their unit.
# This exists so an operator's ad-hoc `ros2 topic list` sees the same world.
#
# Sourced by `bash -lc` in several units, so it must never fail and must never
# leave the caller's shell options changed.
__fpms_opts="$-"
set +eu
[ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
EOF
# The workspace path comes from the build config rather than being hardcoded a
# second time.
cat >> /etc/profile.d/fpms-ros.sh <<EOF
[ -f ${FPMS_HOME}/uros_ws/install/setup.bash ] && . ${FPMS_HOME}/uros_ws/install/setup.bash
EOF
cat >> /etc/profile.d/fpms-ros.sh <<'EOF'
case "$__fpms_opts" in *e*) set -e ;; esac
case "$__fpms_opts" in *u*) set -u ;; esac
unset __fpms_opts
EOF
# ROS_DOMAIN_ID and RMW_IMPLEMENTATION come from config/fpms-os.conf so this
# file cannot drift away from what the units carry. SPEC.md fixes them at 20
# and rmw_fastrtps_cpp; the defaults here match, and exist only so a hand-run
# of this stage without build.sh still produces a correct file.
cat >> /etc/profile.d/fpms-ros.sh <<EOF
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-20}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fpms/fastdds_udp_only.xml
export ROS_LOCALHOST_ONLY=1
# Never let this file be the reason a login shell reports failure.
:
EOF
chmod 0644 /etc/profile.d/fpms-ros.sh
bash -n /etc/profile.d/fpms-ros.sh || fatal "generated /etc/profile.d/fpms-ros.sh does not parse"

phase "done"
disk
echo "--- 10-ros-humble OK"
