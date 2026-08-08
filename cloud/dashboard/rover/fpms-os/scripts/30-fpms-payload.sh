#!/usr/bin/env bash
# Stage 30 - install the FPMS rover code.
#
# This is what deploy_stack.sh does at deploy time, done once at image build
# time instead. Same destinations, same modes, so an operator who knows the
# deploy script recognises the layout.
set -euo pipefail
echo "--- 30-fpms-payload"

SRC=/opt/fpms-os/src
H="${FPMS_HOME}"

[ -d "$SRC" ] || { echo "FATAL: $SRC missing (build.sh should have staged it)" >&2; exit 1; }

inst() {  # inst <src> <dst> <mode>
    if [ -f "$SRC/$1" ]; then
        install -m "$3" -o "${FPMS_USER}" -g "${FPMS_USER}" "$SRC/$1" "$2"
        echo "    $2"
    else
        echo "    MISSING: $1  (expected at $SRC/$1)" >&2
        return 1
    fi
}

MISSING=0

# --- services that run from the home directory ------------------------------
inst fpms_missions.py   "$H/fpms_missions.py"   0755 || MISSING=1
inst fpms_lidar_ros.py  "$H/fpms_lidar_ros.py"  0755 || MISSING=1
inst fpms_teleop.py     "$H/fpms_teleop.py"     0755 || MISSING=1
inst fpms_odom_tf.py    "$H/fpms_odom_tf.py"    0755 || MISSING=1
inst fpms_duty_driver.py "$H/fpms_duty_driver.py" 0755 || MISSING=1

# fpms_cloud_uplink.py MUST sit in the same directory as the agent: the agent
# imports it by path with sys.path.insert(dirname(__file__)). A wrong layout
# silently disables the cloud uplink -- it is wrapped in try/except.
inst fpms_cloud_uplink.py "$H/fpms_cloud_uplink.py" 0755 || MISSING=1
inst fpms_yolo26_npu.py   "$H/yolo/fpms_yolo26_npu.py" 0755 || MISSING=1

inst stack/fpms_cored.py   "$H/fpms_cored.py"   0755 || MISSING=1
inst stack/fpms_charact.py "$H/fpms_charact.py" 0755 || MISSING=1
inst STACK.md              "$H/STACK.md"        0644 || true

# The masked second-writers. Installed so their units can be MASKED rather
# than merely absent -- see the unit headers.
inst fpms_ros_tunnel.py   "$H/fpms_ros_tunnel.py"   0755 || true
inst fpms_rtos_follower.py "$H/fpms_rtos_follower.py" 0755 || true

# --- /usr/local/bin ---------------------------------------------------------
#
# TWO AGENT FILES EXIST IN THE REPO with near-identical names:
#   fpms_rover_agent.py   (underscore)
#   fpms-rover-agent.py   (hyphen)
# They have drifted. The unit runs /usr/local/bin/fpms-rover-agent. We install
# the LONGER one, and say which, loudly -- silently picking is how they drift
# further.
AGENT=""
for cand in fpms-rover-agent.py fpms_rover_agent.py; do
    [ -f "$SRC/$cand" ] || continue
    if [ -z "$AGENT" ]; then AGENT="$cand"; else
        a=$(wc -l < "$SRC/$AGENT"); b=$(wc -l < "$SRC/$cand")
        [ "$b" -gt "$a" ] && AGENT="$cand"
    fi
done
if [ -n "$AGENT" ]; then
    echo "    installing agent from $AGENT ($(wc -l < "$SRC/$AGENT") lines)"
    echo "    NOTE: the repo carries two agent files with near-identical names."
    echo "    Confirm this is the live one before a competition."
    install -m 0755 "$SRC/$AGENT" /usr/local/bin/fpms-rover-agent
else
    echo "    MISSING: no rover agent found" >&2; MISSING=1
fi

install -m 0755 "$SRC/stack/fpms-uros-supervisor" /usr/local/bin/ 2>/dev/null \
    || { echo "    MISSING: stack/fpms-uros-supervisor" >&2; MISSING=1; }
# fpms-uros-agent-run comes from the OVERLAY, not the repo: FPMS-OS reads the
# device and baud from config.env instead of hardcoding them in two places
# that disagreed.

# --- nav2 / slam trees ------------------------------------------------------
#
# fpms-tf.service has always referenced /home/ubuntu/nav2/fpms_tf.launch.py and
# NOTHING EVER DEPLOYED IT. deploy_stack.sh installs no nav2 content at all.
install -d -o "${FPMS_USER}" -g "${FPMS_USER}" "$H/nav2" "$H/slam/maps"
cp -a "$SRC/nav2/." "$H/nav2/" 2>/dev/null || echo "    WARNING: no nav2/ tree" >&2
cp -a "$SRC/slam/." "$H/slam/" 2>/dev/null || echo "    WARNING: no slam/ tree" >&2
chown -R "${FPMS_USER}:${FPMS_USER}" "$H/nav2" "$H/slam"

# --- things that exist only on the old Pi -----------------------------------
cat <<'EOF'

    ------------------------------------------------------------------
     NOT IN THE REPOSITORY, AND THEREFORE NOT IN THIS IMAGE:

       ~/yolo/yolo26n-rk3588.rknn   the detection model
       ~/yolo/fpms_yolo_npu.py      the v8 decode path
       ~/fpms_console/              the operator console web UI
       ~/nav2/arena_zones.json      arena zone definitions

     These live only on the old Pi. Copy them across before relying on
     detection or the console:

       scp -r ubuntu@<old-pi>:~/yolo ubuntu@fpms-pi.local:~/
       scp -r ubuntu@<old-pi>:~/fpms_console ubuntu@fpms-pi.local:~/

     config.env sets FPMS_YOLO_VARIANT=v26 so the agent uses the decode
     path that IS in the repo. Left at the code default (v8) it would
     import the missing file and run blind.

     fpms-selftest reports each of these as a FAIL until they are present.
    ------------------------------------------------------------------

EOF

chown -R "${FPMS_USER}:${FPMS_USER}" "$H"

if [ "$MISSING" = 1 ]; then
    echo "FATAL: required rover source files were missing. See above." >&2
    exit 1
fi
echo "--- 30-fpms-payload OK"
