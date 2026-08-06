#!/usr/bin/env bash
# =============================================================================
# FPMS — install the Foxglove operator console on the rover Pi.
#
# RUN THIS ON THE PI, as user `ubuntu`, with sudo available:
#     bash ~/install_foxglove.sh
#
# It is idempotent: run it as many times as you like.
#
# What it does, in order:
#   1. sanity-checks the machine (arm64, Ubuntu 22.04, ROS 2 Humble present)
#   2. installs ros-humble-foxglove-bridge from apt, and if apt does not have
#      it, builds it from source into ~/fox_ws
#   3. copies fpms_foxglove_cmd.py to /home/ubuntu
#   4. installs and ENABLES both systemd units (enabled = survives a cold boot)
#   5. verifies: port 8765 listening, both units active, ROS_DOMAIN_ID=20,
#      services advertised
#
# It does NOT touch fpms_missions.py, fpms_teleop.py, fpms-rover-agent,
# fpms_lidar_ros.py or any of their units.
# =============================================================================
set -uo pipefail

GREEN=$'\033[32m'; RED=$'\033[31m'; YEL=$'\033[33m'; OFF=$'\033[0m'
ok()   { echo "${GREEN}[ok]${OFF}   $*"; }
warn() { echo "${YEL}[warn]${OFF} $*"; }
die()  { echo "${RED}[FAIL]${OFF} $*"; exit 1; }
step() { echo; echo "=== $* ==="; }

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_NODE="$HERE/fpms_foxglove_cmd.py"
SRC_UNITS="$HERE/units"
# Fall back to files sitting next to this script in $HOME, which is how they
# arrive if they were copied over one at a time rather than as a directory.
[ -f "$SRC_NODE" ] || SRC_NODE="$HOME/fpms_foxglove_cmd.py"
[ -d "$SRC_UNITS" ] || SRC_UNITS="$HOME"

# ---------------------------------------------------------------- 1. sanity
step "1/5  Checking the machine"

ARCH="$(uname -m)"
echo "arch: $ARCH"
[ "$ARCH" = "aarch64" ] || warn "expected aarch64 (arm64); got $ARCH"

if [ -r /etc/os-release ]; then
    . /etc/os-release
    echo "os:   $PRETTY_NAME"
    [ "${VERSION_CODENAME:-}" = "jammy" ] || \
        warn "expected Ubuntu 22.04 (jammy); got ${VERSION_CODENAME:-unknown}"
fi

[ -f /opt/ros/humble/setup.bash ] || die \
    "/opt/ros/humble/setup.bash is missing. This Pi does not have ROS 2 Humble
     where every fpms-*.service expects it. Stop and fix that first."
ok "ROS 2 Humble present"

command -v mosquitto_pub >/dev/null 2>&1 \
    && ok "mosquitto client tools present" \
    || warn "mosquitto_pub not found — the command bridge does not need it, but
             you will not be able to test the MQTT verbs by hand."

python3 -c 'import paho.mqtt.client' 2>/dev/null \
    && ok "python3 paho-mqtt present" \
    || die "python3 paho-mqtt is missing and fpms_foxglove_cmd.py needs it.
            Install with: sudo apt-get install -y python3-paho-mqtt"

# ------------------------------------------------------- 2. foxglove_bridge
step "2/5  Installing foxglove_bridge"

if dpkg -s ros-humble-foxglove-bridge >/dev/null 2>&1; then
    ok "ros-humble-foxglove-bridge already installed ($(dpkg-query -W -f='${Version}' ros-humble-foxglove-bridge))"
else
    echo "apt-get update ..."
    sudo apt-get update -qq || warn "apt-get update failed; trying install anyway"
    if sudo apt-get install -y ros-humble-foxglove-bridge; then
        ok "installed from apt"
    else
        # ---------------------------------------------------------------
        # SOURCE FALLBACK.
        #
        # The Humble binary for arm64/jammy is expected to exist (Humble is
        # Tier 1 on jammy arm64 and foxglove_bridge is a released package,
        # currently 3.4.3). This path exists because that was NOT verified
        # against this Pi's own apt — the Pi was powered off when this was
        # written — and "apt had no candidate" must not be a dead end at a
        # competition.
        #
        # It builds into ~/fox_ws. The systemd unit already sources
        # ~/fox_ws/install/setup.bash if it exists, so nothing else changes.
        # Budget 10-25 minutes on this board; it is a C++ package with an
        # embedded websocket library.
        # ---------------------------------------------------------------
        warn "apt could not install ros-humble-foxglove-bridge — building from source"
        echo "This takes 10-25 minutes on this board. Leave it alone."
        sudo apt-get install -y git python3-rosdep python3-colcon-common-extensions \
             build-essential cmake libasio-dev libssl-dev libwebsocketpp-dev nlohmann-json3-dev \
            || warn "some build dependencies failed to install; continuing"
        mkdir -p "$HOME/fox_ws/src"
        if [ ! -d "$HOME/fox_ws/src/ros-foxglove-bridge/.git" ]; then
            git clone --branch main --depth 1 \
                https://github.com/foxglove/ros-foxglove-bridge.git \
                "$HOME/fox_ws/src/ros-foxglove-bridge" \
                || die "git clone failed — no network, or GitHub unreachable."
        fi
        ( set -e
          source /opt/ros/humble/setup.bash
          cd "$HOME/fox_ws"
          sudo rosdep init 2>/dev/null || true
          rosdep update || true
          rosdep install --from-paths src --ignore-src -y -r || true
          # -j2, not the default. This board has run out of RAM building C++
          # before and an OOM-killed colcon looks like an unexplained failure.
          MAKEFLAGS="-j2" colcon build --symlink-install \
              --cmake-args -DCMAKE_BUILD_TYPE=Release
        ) || die "colcon build failed. Read the output above; the usual causes
                  are a missing dependency and running out of RAM."
        ok "built from source into ~/fox_ws"
    fi
fi

# ------------------------------------------------------------ 3. the node
step "3/5  Installing the command bridge node"

[ -f "$SRC_NODE" ] || die "cannot find fpms_foxglove_cmd.py (looked in $HERE and \$HOME)"
if [ "$SRC_NODE" != "$HOME/fpms_foxglove_cmd.py" ]; then
    sed 's/\r$//' "$SRC_NODE" > "$HOME/fpms_foxglove_cmd.py" \
        || die "could not copy fpms_foxglove_cmd.py"
fi
chmod +x "$HOME/fpms_foxglove_cmd.py"
python3 -m py_compile "$HOME/fpms_foxglove_cmd.py" \
    || die "fpms_foxglove_cmd.py does not compile — do not install a broken
            command bridge."
ok "fpms_foxglove_cmd.py installed and compiles"

# ------------------------------------------------------------- 4. the units
step "4/5  Installing and ENABLING the systemd units"

for u in fpms-foxglove-bridge.service fpms-foxglove-cmd.service; do
    [ -f "$SRC_UNITS/$u" ] || die "missing unit file: $SRC_UNITS/$u"
    # Strip CR before installing. These files are edited on a Windows laptop,
    # and a CRLF inside ExecStart hands bash a trailing carriage return: the
    # unit dies at start with an error that names nothing you would guess.
    # .gitattributes already forces LF; this is the belt to that pair of
    # braces, because the file can also arrive by copy-paste or by a text
    # editor that "helpfully" converted it.
    sed 's/\r$//' "$SRC_UNITS/$u" > "/tmp/$u.clean" || die "could not read $u"
    sudo install -m 0644 "/tmp/$u.clean" "/etc/systemd/system/$u" \
        || die "could not install $u"
    rm -f "/tmp/$u.clean"
    ok "installed /etc/systemd/system/$u"
done

sudo systemctl daemon-reload
# `enable` (not just start) is the whole point: the rover must come up with the
# operator console working after a cold boot at the venue, with nobody at a
# keyboard.
sudo systemctl enable --now fpms-foxglove-bridge.service || die "enable bridge failed"
sudo systemctl enable --now fpms-foxglove-cmd.service    || die "enable cmd bridge failed"
ok "both units enabled for cold boot and started"

# -------------------------------------------------------------- 5. verify
step "5/5  Verifying"

sleep 6

fail=0

for u in fpms-foxglove-bridge fpms-foxglove-cmd; do
    if systemctl is-active --quiet "$u"; then
        ok "$u is active"
    else
        echo "${RED}[FAIL]${OFF} $u is NOT active. Last 30 log lines:"
        sudo journalctl -u "$u" -n 30 --no-pager
        fail=1
    fi
    if systemctl is-enabled --quiet "$u"; then
        ok "$u is enabled (survives reboot)"
    else
        echo "${RED}[FAIL]${OFF} $u is NOT enabled — it will not come back after a reboot"
        fail=1
    fi
done

# The port. `ss` is in the base image; grep for the literal port so this works
# whether it binds v4, v6 or both.
if ss -ltn 2>/dev/null | grep -q ':8765'; then
    ok "something is LISTENING on :8765"
    ss -ltn | grep ':8765'
else
    echo "${RED}[FAIL]${OFF} nothing is listening on :8765"
    fail=1
fi

# THE DOMAIN TRAP. Checked explicitly because an empty topic list here is the
# single most expensive false alarm in this project's history.
echo
echo "--- topics visible to the bridge's domain ---"
DOM="$(systemctl show fpms-foxglove-bridge -p Environment --value | tr ' ' '\n' | grep '^ROS_DOMAIN_ID=' | head -1)"
echo "unit says: ${DOM:-<NOT SET — THIS IS THE BUG>}"
[ "$DOM" = "ROS_DOMAIN_ID=20" ] && ok "ROS_DOMAIN_ID=20 is set in the unit" || {
    echo "${RED}[FAIL]${OFF} the unit does not set ROS_DOMAIN_ID=20."
    echo "        Every topic will look absent. This is THE trap."
    fail=1; }

N="$( (source /opt/ros/humble/setup.bash; ROS_DOMAIN_ID=20 timeout 12 ros2 topic list 2>/dev/null | wc -l) )"
echo "topics on domain 20: $N"
if [ "${N:-0}" -lt 3 ]; then
    warn "very few topics on domain 20. If the drive board and the LiDAR agent
          are running, this means something is wrong. If they are NOT running,
          this is expected and Foxglove will simply show an empty topic list."
fi

echo
echo "--- services the Foxglove buttons call ---"
(source /opt/ros/humble/setup.bash; ROS_DOMAIN_ID=20 timeout 12 ros2 service list 2>/dev/null | grep '^/fpms/') \
    || warn "no /fpms/* services yet — give fpms-foxglove-cmd a few more seconds"

echo
echo "=========================================================="
if [ "$fail" = "0" ]; then
    echo "${GREEN}DONE.${OFF} On the laptop, run Open-FPMS-Foxglove.cmd, or point"
    echo "Foxglove Studio (DESKTOP app) at:"
    echo
    echo "        ws://fpms-pi.local:8765"
    echo
else
    echo "${RED}FINISHED WITH FAILURES — read the [FAIL] lines above.${OFF}"
fi
echo "=========================================================="
exit "$fail"
