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
KEY_EXP="$(printf '%s\n' "$KEY_INFO" | awk -F: '$1=="pub" {print $7; exit}')"
KEY_FPR="$(printf '%s\n' "$KEY_INFO" | awk -F: '$1=="fpr" {print $10; exit}')"
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
say "ros-humble-ros-base: $(printf '%s\n' "$ROS_POLICY" | awk '/Candidate:/{print $2; exit}')"

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
    local p cand missing=""
    for p in "$@"; do
        cand="$(apt-cache policy "$p" 2>/dev/null | awk '/Candidate:/ {print $2; exit}')"
        if [ -z "$cand" ] || [ "$cand" = "(none)" ]; then
            missing="$missing $p"
        fi
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
uros_agent_bin() {
    find "$UROS_WS/install" -type f -name micro_ros_agent -perm -u+x 2>/dev/null \
        | head -n1 || true
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

# Idempotency, and it is worth real money here: `build.sh --stage 10` re-run
# after a later stage failed must not spend another four hours rebuilding a
# workspace that is already good. The stamp is written ONLY after the agent
# executable has been verified, so a half-built workspace is never mistaken
# for a finished one.
if [ -f "$UROS_STAMP" ] && [ -f "$UROS_WS/install/setup.bash" ] && [ -n "$(uros_agent_bin)" ]; then
    say "workspace already built on $(cat "$UROS_STAMP") - skipping"
    say "delete $UROS_STAMP to force a rebuild"
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

    retry git clone -q --depth 1 -b humble \
        https://github.com/micro-ROS/micro_ros_setup.git "$UROS_WS/src/micro_ros_setup" \
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
    say "agent: $AGENT_BIN"

    date -u '+%Y-%m-%dT%H:%M:%SZ' > "$UROS_STAMP"

    # Reclaim the intermediates. Safe precisely because the builds above did not
    # use --symlink-install, so nothing in install/ points into build/. src/ stays:
    # it is small next to build/ and it is what makes the agent debuggable on the
    # rover.
    rm -rf "$UROS_WS/build" "$UROS_WS/log"
    say "install tree: $(du -sh "$UROS_WS/install" 2>/dev/null | cut -f1)"
fi   # end of the build-if-not-already-built block

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
