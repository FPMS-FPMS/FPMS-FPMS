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

# --- did the payload actually arrive? ---------------------------------------
#
# build.sh stages this tree with:
#
#     cp -a "$HERE/.."/*.py "$HERE/../stack" ... "$MNT/opt/fpms-os/src/" \
#         2>/dev/null || true
#
# THAT `|| true` IS THE MOST DANGEROUS LINE IN THE BUILD. It exists because
# `cp -a` from a Windows-hosted source (WSL /mnt/c) routinely fails to preserve
# ownership and exits non-zero having copied everything correctly - so the
# status is genuinely uninformative and cannot be trusted either way. The cost
# is that a copy which really did fail is indistinguishable from one that
# worked: `mkdir -p src` has already run, so the directory EXISTS and is
# EMPTY, and a build that only checked `[ -d ]` would sail past it and produce
# a fully-booting rover with no software on it, silently.
#
# So: check for CONTENT, not for the directory, and name the cause in the
# error, because the person reading it will be looking at stage 30 and the
# fault is one stage earlier.
if [ ! -d "$SRC" ] || [ -z "$(ls -A "$SRC" 2>/dev/null)" ]; then
    cat >&2 <<EOF
FATAL: $SRC is missing or EMPTY.

build.sh's stage_all() copies the rover source tree into the image with a
trailing "2>/dev/null || true", so a failed copy does not fail the build. This
check is the only thing standing between that and an image that boots
perfectly and contains no rover software at all.

Look at, on the BUILD HOST:
    $(dirname "$SRC")            (should hold scripts/ overlay/ selftest/ src/)
and on the host side of the repo, two levels above fpms-os/:
    *.py  stack/  nav2/  slam/  STACK.md
Re-run the copy without the "2>/dev/null || true" to see the real error.
EOF
    exit 1
fi

# --- the ubuntu user must already exist -------------------------------------
#
# Everything below installs with `-o ubuntu -g ubuntu`. This script runs INSIDE
# the chroot (build.sh chroots first, then execs it), so install(1) resolves
# that name through the chroot's own /etc/passwd, not the host's - which is
# what we want, and is why stage 00's useradd is the thing that has to have
# run, not anything on the build machine.
#
# Checked explicitly because otherwise the first `install` fails with "invalid
# user", the inst() wrapper turns that into MISSING=1, and the build reports
# "required rover source files were missing" - which is a lie that costs an
# hour.
if ! getent passwd "${FPMS_USER}" >/dev/null 2>&1; then
    echo "FATAL: user '${FPMS_USER}' does not exist in the chroot's /etc/passwd." >&2
    echo "Stage 00 creates it. Run the full build, or --stage 00 first." >&2
    exit 1
fi

# --- CRLF: normalise the payload before anything is installed ---------------
#
# fpms-os/.gitattributes forces LF, but it only governs fpms-os/. The rover
# source tree lives TWO LEVELS ABOVE it and is covered by no .gitattributes at
# all, so with core.autocrlf=true on the Windows workstation EVERY file in this
# payload is checked out CRLF. Verified: all of *.py, stack/, nav2/ and slam/.
#
# What that costs, concretely:
#   - /usr/local/bin/fpms-uros-supervisor and /usr/local/bin/fpms-rover-agent
#     get a shebang ending "\r". fpms-uros-supervisor.service ExecStart=s the
#     path directly, so exec fails with
#         /usr/local/bin/fpms-uros-supervisor: /usr/bin/env: bad interpreter
#     which is spectacularly misleading - /usr/bin/env plainly exists and the
#     file looks perfect in every editor. That unit is the drive link.
#   - $H/nav2/arena_map.yaml stays CRLF, so stage 40's comparison against the
#     freshly generated (LF) yaml can never match on a text diff.
#   - Every ~/*.py is installed 0755 with a \r shebang, so it works under
#     `python3 file.py` (the units) and breaks the moment anyone runs ./file.py.
#
# Stage 50 has a CRLF guard, but it only sweeps /usr/local/{bin,sbin}/fpms-*,
# the units and config.env - none of the payload - and it runs AFTER stage 40
# needs the yaml to be clean. Fix it here, at the source tree, once, so
# everything derived from it downstream is already correct.
# Strip unconditionally and count by SIZE rather than testing for \r first.
# `grep $'\r'` is the obvious detector and it is not portable enough to build
# on: MSYS/Git-Bash grep matches CR in files that contain no CR byte at all
# (measured), and a detector that lies in either direction on a check like this
# is worse than no detector. A byte count cannot: if the file got shorter, CRs
# came out of it. sed -i on an already-LF file is a no-op, so this is also
# idempotent across `--stage 30` re-runs.
CRLF_N=0
while IFS= read -r f; do
    [ -n "$f" ] || continue
    before="$(wc -c < "$f")"
    sed -i 's/\r$//' "$f"
    after="$(wc -c < "$f")"
    if [ "$before" -ne "$after" ]; then
        CRLF_N=$((CRLF_N + 1))
    fi
done < <(find "$SRC" -type f \
    \( -name '*.py' -o -name '*.yaml' -o -name '*.yml' -o -name '*.json' \
       -o -name '*.xml' -o -name '*.md' -o -name 'fpms-*' \))
if [ "$CRLF_N" -gt 0 ]; then
    echo "    normalised CRLF -> LF in $CRLF_N payload files"
    echo "    (expected: the rover tree sits above fpms-os/.gitattributes' reach)"
fi

# yolo/ is created by stage 00, but install(1) will not create a missing parent
# and fpms_yolo26_npu.py goes into it. Make this stage stand on its own so
# `--stage 30` against a chroot in any state does the same thing.
install -d -m 0755 -o "${FPMS_USER}" -g "${FPMS_USER}" "$H/yolo"

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
#
# Written as an explicit if/then rather than `[ cond ] && VAR=x` as the last
# statement of the loop body: that idiom leaves the whole `for` compound with a
# non-zero status whenever the final comparison is false, which under
# `set -euo pipefail` is a stage that dies for no reason at all on exactly the
# input we expect (the hyphen file is the longer one, so the last comparison
# IS false). `wc -l` is likewise only ever reached for a file that exists --
# the -f guard `continue`s first -- so a missing candidate cannot produce a
# "wc: no such file" that gets misread as the agent being broken.
AGENT=""
AGENT_LINES=0
for cand in fpms-rover-agent.py fpms_rover_agent.py; do
    [ -f "$SRC/$cand" ] || continue
    n="$(wc -l < "$SRC/$cand")"
    echo "    agent candidate: $cand ($n lines)"
    if [ "$n" -gt "$AGENT_LINES" ]; then
        AGENT="$cand"
        AGENT_LINES="$n"
    fi
done
if [ -n "$AGENT" ]; then
    echo "    installing agent from $AGENT ($AGENT_LINES lines)"
    echo "    NOTE: the repo carries two agent files with near-identical names."
    echo "    Confirm this is the live one before a competition."
    install -m 0755 "$SRC/$AGENT" /usr/local/bin/fpms-rover-agent
else
    echo "    MISSING: no rover agent found (looked for fpms-rover-agent.py and" >&2
    echo "             fpms_rover_agent.py in $SRC)" >&2
    MISSING=1
fi

# Installed root-owned into /usr/local/bin on purpose: it is executed by
# fpms-uros-supervisor.service, not imported, and nothing running as ubuntu
# should be able to rewrite the process that owns the drive link.
#
# No 2>/dev/null here. Swallowing install's stderr hides the difference
# between "the file is not in the payload" and "the destination is read-only",
# and those want different fixes.
if [ -f "$SRC/stack/fpms-uros-supervisor" ]; then
    install -m 0755 "$SRC/stack/fpms-uros-supervisor" /usr/local/bin/fpms-uros-supervisor
    echo "    /usr/local/bin/fpms-uros-supervisor"
else
    echo "    MISSING: stack/fpms-uros-supervisor" >&2
    MISSING=1
fi
# fpms-uros-agent-run comes from the OVERLAY, not the repo: FPMS-OS reads the
# device and baud from config.env instead of hardcoding them in two places
# that disagreed.

# --- nav2 / slam trees ------------------------------------------------------
#
# fpms-tf.service has always referenced /home/ubuntu/nav2/fpms_tf.launch.py and
# NOTHING EVER DEPLOYED IT. deploy_stack.sh installs no nav2 content at all.
#
# These are NOT optional and a warning is not enough. fpms-tf.service is in
# stage 60's BOOT_UNITS and ExecStart=s the launch file by absolute path, so a
# missing nav2/ means no TF tree on every boot; stage 40 generates the arena
# map from nav2/make_arena_map.py; and both slam launch files refuse to start
# on a params_file that does not exist. A `|| echo WARNING` here turns all of
# that into a line nobody reads in a 40-minute build log.
install -d -o "${FPMS_USER}" -g "${FPMS_USER}" "$H/nav2" "$H/slam" "$H/slam/maps"
for tree in nav2 slam; do
    if [ -d "$SRC/$tree" ]; then
        cp -a "$SRC/$tree/." "$H/$tree/"
        echo "    $H/$tree/"
    else
        echo "    MISSING: $tree/ tree (expected at $SRC/$tree)" >&2
        MISSING=1
    fi
done

# Name the individual files, because a partial copy is the failure mode the
# `|| true` in build.sh makes possible and a directory check cannot see it.
for f in nav2/fpms_tf.launch.py nav2/fpms_nav2.launch.py nav2/nav2_params.yaml \
         nav2/nav2_params_slam.yaml nav2/make_arena_map.py nav2/arena_map.yaml \
         slam/fpms_slam_localization.launch.py slam/fpms_slam_mapping.launch.py \
         slam/mapper_params_localization.yaml slam/mapper_params_mapping.yaml; do
    [ -f "$H/$f" ] || { echo "    MISSING: $f (not in the staged payload)" >&2; MISSING=1; }
done

chown -R "${FPMS_USER}:${FPMS_USER}" "$H/nav2" "$H/slam"

# --- things that exist only on the old Pi -----------------------------------
cat <<'EOF'

    ------------------------------------------------------------------
     NOT IN THE REPOSITORY, AND THEREFORE NOT IN THIS IMAGE:

       ~/yolo/yolo26n-rk3588.rknn   the detection model
       ~/fpms_console/              the operator console web UI

     (fpms_yolo_npu.py, the v8 decode path, IS in the repo at
      rover/rescued/fpms_yolo_npu.py -- but it targets yolov8n.rknn,
      not yolo26, so it does not rescue the v26 path. The MODEL is
      still the thing that exists nowhere but the old Pi, and unlike
      a script it cannot be reconstructed from prose. See
      npu/models/README.md for the rescue procedure, and
      npu/convert/ for rebuilding one from ONNX.)

     These live only on the old Pi. Copy them across before relying on
     detection or the console:

       scp -r ubuntu@<old-pi>:~/yolo ubuntu@fpms-pi.local:~/
       scp -r ubuntu@<old-pi>:~/fpms_console ubuntu@fpms-pi.local:~/

     config.env sets FPMS_YOLO_VARIANT=v26 so the agent uses the decode
     path that IS in the repo. Left at the code default (v8) it would
     import the missing file and run blind.

     NO LONGER on this list: ~/nav2/arena_zones.json. It is derived from
     the same ARENA_MM/Z_FRAC/M_FRAC constants as arena.ts, so unlike the
     model it CAN be reconstructed -- make_arena_map.py already emits it
     as a by-product and stage 40 now installs it.

     fpms-selftest reports each of these as a FAIL until they are present.
    ------------------------------------------------------------------

EOF

chown -R "${FPMS_USER}:${FPMS_USER}" "$H"

if [ "$MISSING" = 1 ]; then
    echo "FATAL: required rover source files were missing. See above." >&2
    echo "This almost always means build.sh's staging copy was partial;" >&2
    echo "it ends in '2>/dev/null || true' and cannot report that itself." >&2
    exit 1
fi
echo "--- 30-fpms-payload OK"
