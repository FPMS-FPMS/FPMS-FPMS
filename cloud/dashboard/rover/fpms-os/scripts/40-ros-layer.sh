#!/usr/bin/env bash
# Stage 40 - the ROS configuration layer: DDS profile, nav2/slam params,
# the patched behaviour tree, and the arena map.
set -euo pipefail
echo "--- 40-ros-layer"

H="${FPMS_HOME}"
OVL=/opt/fpms-os/overlay

# --- the DDS profile --------------------------------------------------------
#
# VERIFY IT PARSES, AND FAIL THE BUILD IF IT DOES NOT.
#
# Fast DDS does not fail loudly on a profile it cannot read. It logs at a level
# nobody reads and falls back to defaults -- which silently reintroduces the
# exact bug this file exists to remove: shared-memory segments that do not
# survive across process eras, so discovery succeeds and NO DATA FLOWS.
#
# A malformed profile would produce an image that looks correct in every way
# and reproduces "connected, subscribed, silent - no error anywhere".
PROFILE="$OVL/etc/fpms/fastdds_udp_only.xml"
[ -f "$PROFILE" ] || { echo "FATAL: $PROFILE missing" >&2; exit 1; }
python3 - "$PROFILE" <<'PY' || { echo "FATAL: the Fast DDS profile does not parse. Fast DDS would SILENTLY ignore it and fall back to shared memory." >&2; exit 1; }
import sys, xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
body = open(sys.argv[1]).read()
# useBuiltinTransports=false is the load-bearing line. Listing a UDPv4
# transport alone leaves the builtin set -- which includes SHM -- in place,
# and Fast DDS still prefers SHM for same-host peers. A profile without it
# looks right and fixes nothing.
assert "<useBuiltinTransports>false</useBuiltinTransports>" in body, \
    "useBuiltinTransports is not false -- SHM would still be used"
assert "UDPv4" in body, "no UDPv4 transport declared"
print("    DDS profile parses, UDPv4 only, builtin transports disabled")
PY

# --- nav2 / slam params -----------------------------------------------------
install -d /etc/fpms/nav2 /etc/fpms/slam
for f in "$OVL"/etc/fpms/nav2/*.yaml "$OVL"/etc/fpms/nav2/*.xml; do
    [ -f "$f" ] && install -m 0644 "$f" /etc/fpms/nav2/
done
for f in "$OVL"/etc/fpms/slam/*.yaml; do
    [ -f "$f" ] && install -m 0644 "$f" /etc/fpms/slam/
done

# --- the arena map ----------------------------------------------------------
#
# arena_map.pgm is NOT committed to the repository -- only the .yaml and the
# generator are. Generate it and verify the emitted yaml matches the committed
# one, so a drift in make_arena_map.py's constants cannot silently produce a
# map whose origin disagrees with the dashboard's arena frame.
if [ -f "$H/nav2/make_arena_map.py" ]; then
    echo "--- generating the arena map"
    ( cd "$H/nav2" && python3 make_arena_map.py --out /tmp/arena_map >/dev/null )
    if [ -f "$H/nav2/arena_map.yaml" ]; then
        if diff -q /tmp/arena_map.yaml "$H/nav2/arena_map.yaml" >/dev/null; then
            echo "    generated yaml matches the committed one"
        else
            echo "FATAL: generated arena_map.yaml does not match the committed file." >&2
            echo "make_arena_map.py's constants have drifted from the checked-in map," >&2
            echo "which means the map origin and the dashboard's arena frame disagree." >&2
            diff /tmp/arena_map.yaml "$H/nav2/arena_map.yaml" >&2 || true
            exit 1
        fi
    fi
    install -m 0644 -o "${FPMS_USER}" -g "${FPMS_USER}" \
        /tmp/arena_map.pgm "$H/nav2/arena_map.pgm"
    echo "    $H/nav2/arena_map.pgm"
else
    echo "    WARNING: make_arena_map.py absent; no arena map generated" >&2
fi

# slam/maps stays EMPTY on purpose. fpms_room.posegraph/.data describe one
# physical room and cannot be baked into a shared image. The localisation unit
# refuses to start without them, which is correct.
install -d -o "${FPMS_USER}" -g "${FPMS_USER}" "$H/slam/maps"

# --- the patched behaviour tree ---------------------------------------------
#
# Stock Nav2's BackUp recovery uses backup_speed="0.025" m/s -- under a fifth
# of this rover's firmware velocity floor -- and NO PARAMETER CAN OVERRIDE IT,
# because the speed lives in the XML. Every BackUp recovery with the stock file
# is a guaranteed stall-then-lurch.
BT_SRC="/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
if [ -f "$BT_SRC" ] && [ ! -f /etc/fpms/nav2/fpms_bt_navigate_to_pose.xml ]; then
    sed 's/backup_speed="0\.025"/backup_speed="0.18"/g' "$BT_SRC" \
        > /etc/fpms/nav2/fpms_bt_navigate_to_pose.xml
    chmod 0644 /etc/fpms/nav2/fpms_bt_navigate_to_pose.xml
    echo "    patched BT: backup_speed 0.025 -> 0.18 (the firmware floor)"
fi

# --- record what we actually got --------------------------------------------
set +u; source /opt/ros/humble/setup.bash; set -u
{
    echo "ros_distro=${ROS_DISTRO}"
    echo "nav2=$(dpkg-query -W -f='${Version}' ros-humble-navigation2 2>/dev/null || echo absent)"
    echo "slam_toolbox=$(dpkg-query -W -f='${Version}' ros-humble-slam-toolbox 2>/dev/null || echo absent)"
    echo "rosbridge=$(dpkg-query -W -f='${Version}' ros-humble-rosbridge-suite 2>/dev/null || echo absent)"
    echo "rmw_fastrtps=$(dpkg-query -W -f='${Version}' ros-humble-rmw-fastrtps-cpp 2>/dev/null || echo absent)"
} | tee /etc/fpms/ros-versions.txt

echo "--- 40-ros-layer OK"
