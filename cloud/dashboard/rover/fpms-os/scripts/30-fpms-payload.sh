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
# THE REASON THIS CHECK EXISTS HAS CHANGED, and the old reason is now false, so
# it is restated rather than left standing to be re-derived by the next reader.
#
# It used to say that build.sh's stage_all() ends its copy in
# `2>/dev/null || true` and that THAT was the most dangerous line in the build:
# `cp -a` from a Windows-hosted source (WSL /mnt/c) can fail to preserve
# ownership and exit non-zero having copied everything correctly, so the `|| true`
# was there to tolerate an uninformative status - at the price of making a copy
# that really did fail indistinguishable from one that worked. `mkdir -p src`
# has already run, so the directory EXISTS and is EMPTY, and a build that only
# checked `[ -d ]` sails past it and produces a fully-booting rover with no
# rover software on it, silently.
#
# RE-READ 2026-08-11 against the build.sh in this worktree: that `|| true` is
# GONE. stage_all() now runs the copy unsilenced and ends it in
#
#     || die "could not stage the rover source tree into the image. ..."
#
# with the note that the old form swallowed "No space left on device" and
# turned a disk-full into this stage's much less obvious "required rover source
# files were missing". So the copy now reports its own failure, on the host,
# with the real error visible - and the empirical evidence that /mnt/c -> ext4
# `cp -a` does return 0 on this host is that stage_all() completed and stages
# 00 and 10 ran.
#
# THE CHECK STAYS ANYWAY, and not out of sentiment. It is one `ls` against a
# 14-hour build, and it still covers the cases build.sh's `|| die` cannot see:
#   - `--stage 30` or `--from 30` against an image whose src/ was staged by an
#     older build.sh, or emptied by hand in a `build.sh --shell` session;
#   - a copy that succeeded into a different mount than the one now chroot'd;
#   - anything that removes files between stage_all() and this stage.
# Check for CONTENT, not for the directory, and name the stage that is actually
# at fault - the person reading this will be looking at stage 30 and the fault
# is one stage earlier.
if [ ! -d "$SRC" ] || [ -z "$(ls -A "$SRC" 2>/dev/null)" ]; then
    cat >&2 <<EOF
FATAL: $SRC is missing or EMPTY.

build.sh's stage_all() copies the rover source tree into the image and dies if
that copy fails, so reaching THIS message means the tree was lost after it was
staged, or that the image being chroot'd is not the one that was staged into.

Look at, on the BUILD HOST:
    $(dirname "$SRC")            (should hold scripts/ overlay/ selftest/ src/)
and on the host side of the repo, two levels above fpms-os/:
    *.py  stack/  nav2/  slam/  STACK.md
Re-run the whole build, or --from 30 after confirming the staging copy ran.
EOF
    exit 1
fi

# --- the ubuntu user must already exist -------------------------------------
#
# Everything below installs with `-o ubuntu`. This script runs INSIDE the
# chroot (build.sh chroots first, then execs it), so install(1) resolves that
# name through the chroot's own /etc/passwd, not the host's - which is what we
# want, and is why stage 00's useradd is the thing that has to have run, not
# anything on the build machine. (Confirmed by reading build.sh's in_chroot():
# `chroot "$MNT" /usr/bin/env -i ... /bin/bash -c "$*"` - the whole stage runs
# with $MNT as /, so there is no path by which the host's passwd is consulted.)
#
# Checked explicitly, and the reason is sharper than it was: a missing user
# makes EVERY `install -o` below fail identically, and the summary at the
# bottom then says "required rover source files were missing" - which names the
# wrong stage, sends the reader to build.sh's staging copy, and costs an hour.
# One getent turns that into one line naming stage 00.
if ! getent passwd "${FPMS_USER}" >/dev/null 2>&1; then
    echo "FATAL: user '${FPMS_USER}' does not exist in the chroot's /etc/passwd." >&2
    echo "Stage 00 creates it. Run the full build, or --stage 00 first." >&2
    exit 1
fi

# --- and the GROUP is asked for, not assumed --------------------------------
#
# Every install below used to pass `-g "${FPMS_USER}"`, i.e. it assumed the
# primary group is named after the user. That is true of Ubuntu's cloud image
# and of `useradd -m` with USERGROUPS_ENAB yes, and it is exactly the
# assumption stage 00 refuses to make: it writes
#
#     FPMS_GROUP="$(id -gn "${FPMS_USER}")"
#
# and uses THAT for ${FPMS_HOME}, ~/.ssh and ~/yolo. Two stages disagreeing
# about the group of the same home directory is how ~/yolo ends up owned by one
# group and the file stage 30 puts inside it by another. Ask the same question
# stage 00 asked, and get the same answer by construction.
#
# It matters more here than it looks, because of the inst() bug this file used
# to have: `install -g <nonexistent>` fails with "invalid group", and until the
# rewrite below that failure was SWALLOWED and printed as a success line. The
# whole payload could have gone missing while every line said OK.
#
# `if !`, not a bare assignment: `FPMS_GROUP="$(id -gn ...)"` adopts id's exit
# status, and an id that fails would kill this stage with only id's own
# one-liner between the getent check above and a stage that appeared to stop
# for no reason.
if ! FPMS_GROUP="$(id -gn "${FPMS_USER}")"; then
    echo "FATAL: could not read the primary group of '${FPMS_USER}' in the chroot." >&2
    echo "getent found the user, so /etc/group is the suspect. Stage 00 sets both." >&2
    exit 1
fi
if [ -z "$FPMS_GROUP" ]; then
    echo "FATAL: '${FPMS_USER}' has an empty primary group name." >&2
    exit 1
fi
[ "$FPMS_GROUP" = "${FPMS_USER}" ] \
    || echo "    note: ${FPMS_USER}'s primary group is '$FPMS_GROUP', not '${FPMS_USER}'"

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
#
# THE FILE LIST IS CAPTURED AND ITS STATUS IS CHECKED. This was
#
#     done < <(find "$SRC" -type f \( ... \))
#
# and a process substitution has NO exit status the shell can see - `$?` after
# the loop is the loop's, never find's. So a find that died (a permission
# error, a vanished $SRC, an argument list this shell rejected) produced an
# empty stream, the loop ran zero times, CRLF_N stayed 0, the "normalised"
# message was correctly suppressed, and the stage reported nothing wrong while
# normalising nothing at all. That is the same silent-no-op class as the empty
# $SRC above, and it lands on the one job this block exists to do: an
# unnormalised fpms-uros-supervisor is a "/usr/bin/env: bad interpreter" on a
# rover that otherwise boots.
#
# Capture into a variable, check the status with `if !`, then feed the loop a
# herestring. No pipeline, so nothing to adopt a status from and no SIGPIPE to
# raise. The one thing a herestring cannot survive is a newline INSIDE a
# filename - these are repository paths under our own control, and the trade is
# deliberate: an unreportable find is the failure that actually happens here,
# a newline in "nav2_params.yaml" is not.
if ! CRLF_LIST="$(find "$SRC" -type f \
    \( -name '*.py' -o -name '*.yaml' -o -name '*.yml' -o -name '*.json' \
       -o -name '*.xml' -o -name '*.md' -o -name 'fpms-*' \))"; then
    echo "FATAL: could not list $SRC to normalise line endings." >&2
    echo "Skipping this would ship a CRLF fpms-uros-supervisor, whose unit then" >&2
    echo "fails with '/usr/bin/env: bad interpreter' - the drive link, silently." >&2
    exit 1
fi
if [ -z "$CRLF_LIST" ]; then
    # $SRC was already proved non-empty above, so a list with nothing in it
    # means the payload is there but contains none of *.py/*.yaml/fpms-* -
    # i.e. it is not the rover tree. Every inst() below would fail; say why here.
    echo "FATAL: $SRC contains no *.py, *.yaml, *.json, *.xml, *.md or fpms-* files." >&2
    echo "Something was staged into it, but it is not the rover source tree." >&2
    exit 1
fi
CRLF_N=0
while IFS= read -r f; do
    [ -n "$f" ] || continue
    before="$(wc -c < "$f")"
    sed -i 's/\r$//' "$f"
    after="$(wc -c < "$f")"
    if [ "$before" -ne "$after" ]; then
        CRLF_N=$((CRLF_N + 1))
    fi
done <<<"$CRLF_LIST"
if [ "$CRLF_N" -gt 0 ]; then
    echo "    normalised CRLF -> LF in $CRLF_N payload files"
    echo "    (expected: the rover tree sits above fpms-os/.gitattributes' reach)"
fi

# yolo/ is created by stage 00, but install(1) will not create a missing parent
# and fpms_yolo26_npu.py goes into it. Make this stage stand on its own so
# `--stage 30` against a chroot in any state does the same thing.
install -d -m 0755 -o "${FPMS_USER}" -g "$FPMS_GROUP" "$H/yolo"

# inst <src> <dst> <mode>
#
# EVERY FAILURE HERE IS SURFACED. THE PREVIOUS VERSION SWALLOWED HALF OF THEM,
# and it is the worst bug this file has had, because it fails in the direction
# of a clean build log. It was:
#
#     inst() {
#         if [ -f "$SRC/$1" ]; then
#             install -m "$3" -o ... "$SRC/$1" "$2"
#             echo "    $2"                     # <-- runs even if install died
#         else
#             echo "    MISSING: $1" >&2
#             return 1
#         fi
#     }
#
# `set -e` DOES NOT APPLY INSIDE THIS FUNCTION. Every call site is
# `inst ... || MISSING=1`, and a function invoked as the left operand of `||`
# runs with -e suppressed throughout its whole body. So a failing `install` did
# not abort, did not return, and fell through to `echo "    $2"` - which
# PRINTED THE DESTINATION PATH AS A SUCCESS LINE and made the function return
# echo's 0. MISSING stayed 0, the summary at the bottom said nothing, and the
# stage exited OK having installed nothing.
#
# MEASURED, not reasoned: with `install -m 0755 src /nonexistent-dir/x.py`, the
# old function printed
#     install: cannot create regular file '/nonexistent-dir/x.py': ...
#     /nonexistent-dir/x.py
# and left MISSING=0. The install error is two lines above a success line for
# the same path, in a build log tens of thousands of lines long.
#
# Real cases that produce exactly that: a full image (the `install` writes into
# a filesystem stage 90 has not shrunk yet), a $H that does not exist, and -
# the one that motivated deriving FPMS_GROUP above - `install -g` naming a
# group that is not in the chroot's /etc/group, which fails on EVERY file at
# once and would have reported a completely clean build.
#
# Written as two guarded early returns rather than if/else so the final `echo`
# is only ever reached on a real success, and is the function's exit status.
inst() {
    if [ ! -f "$SRC/$1" ]; then
        echo "    MISSING: $1  (expected at $SRC/$1)" >&2
        return 1
    fi
    if ! install -m "$3" -o "${FPMS_USER}" -g "$FPMS_GROUP" "$SRC/$1" "$2"; then
        echo "    FAILED: $1 -> $2  (install's own error is above this line)" >&2
        return 1
    fi
    echo "    $2"
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
#
# `|| true` on these three (STACK.md above included) means ABSENT IS FINE, not
# that failure is fine. Since the inst() rewrite a genuine install error still
# reaches stderr as "FAILED: ..." with install's own message above it; what the
# `|| true` suppresses is only the MISSING=1 that would fail the build. Keep it
# that way round -- a masked unit's script not being in the repo is a design
# choice, a read-only /home is not.
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
# Written as an explicit if/then rather than `[ cond ] && VAR=x`. The reason
# recorded here used to be that the `&&` idiom "leaves the whole `for` compound
# with a non-zero status ... a stage that dies for no reason at all".
#
#   THAT IS NOT TRUE, and it is worth correcting rather than leaving as a rule
#   this repo enforces for a reason that does not exist. MEASURED under
#   `set -euo pipefail` (bash 5): bash exempts an AND-OR list from -e whenever
#   the command that failed is not the one after the final `&&`/`||`, and the
#   exemption covers the list's own status -- so `[ "$n" -gt "$AL" ] && A=$c`
#   as the last statement of a loop body does NOT abort, and neither does the
#   `for` around it.
#
#   The shape that IS fatal is a FUNCTION whose last statement is a failing
#   AND-OR list: the function returns 1 and the -e trips at the CALL SITE, not
#   inside. There is no such function here; `inst()` above ends in a plain
#   `echo` precisely so it cannot become one.
#
# The if/then stays anyway: it says "pick the longer file" in the shape of the
# thing it does, and it cannot be quietly converted into the fatal form by a
# later edit that wraps this block in a function. `wc -l` is likewise only ever
# reached for a file that exists -- the -f guard `continue`s first -- so a
# missing candidate cannot produce a "wc: no such file" that gets misread as
# the agent being broken.
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
    # Routed through MISSING like everything else, rather than left as a bare
    # `install` for `set -e` to catch. A bare one aborts the stage on the spot,
    # which skips the named-file sweep further down and the FATAL summary at the
    # bottom -- so the operator gets install's single line and has to guess
    # whether anything else was also wrong. Collecting it means one run reports
    # everything that is broken, which on a 14-hour build is the difference
    # between one more build and three.
    if ! install -m 0755 "$SRC/$AGENT" /usr/local/bin/fpms-rover-agent; then
        echo "    FAILED: $AGENT -> /usr/local/bin/fpms-rover-agent" >&2
        MISSING=1
    fi
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
if [ ! -f "$SRC/stack/fpms-uros-supervisor" ]; then
    echo "    MISSING: stack/fpms-uros-supervisor" >&2
    MISSING=1
elif ! install -m 0755 "$SRC/stack/fpms-uros-supervisor" /usr/local/bin/fpms-uros-supervisor; then
    # Same reason as the agent above. This one was NOT the swallowed shape --
    # it sat bare inside a then-block where `set -e` is live, so a failing
    # install did abort rather than print a false success. What it did instead
    # was abort BEFORE the nav2/slam sweep and the FATAL summary, so a
    # read-only /usr/local/bin cost one 14-hour run per missing file discovered.
    echo "    FAILED: stack/fpms-uros-supervisor -> /usr/local/bin/" >&2
    MISSING=1
else
    echo "    /usr/local/bin/fpms-uros-supervisor"
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
install -d -o "${FPMS_USER}" -g "$FPMS_GROUP" "$H/nav2" "$H/slam" "$H/slam/maps"
# `if ! cp -a`, not a bare `cp -a`. Note this is NOT the "cp -a exits non-zero
# having worked" case that forced build.sh's staging copy to be tolerant: that
# one crosses from a Windows-hosted source into the image and cannot preserve
# ownership. THIS one is ext4 -> ext4, inside the chroot, running as root, and
# the chown below fixes ownership regardless -- so a non-zero here is a real
# failure (no space, an I/O error on the loop device) and is treated as one.
#
# Collected into MISSING rather than left to `set -e`, for the reason above:
# the per-file sweep immediately below is the diagnostic that says WHICH of the
# ten files stage 40 needs did not arrive, and aborting on cp's one-liner
# throws it away on a stage that only runs after two other stages have.
for tree in nav2 slam; do
    if [ ! -d "$SRC/$tree" ]; then
        echo "    MISSING: $tree/ tree (expected at $SRC/$tree)" >&2
        MISSING=1
    elif ! cp -a "$SRC/$tree/." "$H/$tree/"; then
        echo "    FAILED: could not copy $SRC/$tree/ into $H/$tree/" >&2
        MISSING=1
    else
        echo "    $H/$tree/"
    fi
done

# Name the individual files. A directory check cannot see a PARTIAL copy, and
# every one of these ten is consumed by name later: stage 40 reads
# make_arena_map.py, nav2_params*.yaml and mapper_params_*.yaml out of $H and
# installs them into /etc/fpms/{nav2,slam}; stage 60 puts fpms-tf.service into
# BOOT_UNITS, and its ExecStart names fpms_tf.launch.py by absolute path. A
# name missing here is a unit that fails on every boot of the finished image.
for f in nav2/fpms_tf.launch.py nav2/fpms_nav2.launch.py nav2/nav2_params.yaml \
         nav2/nav2_params_slam.yaml nav2/make_arena_map.py nav2/arena_map.yaml \
         slam/fpms_slam_localization.launch.py slam/fpms_slam_mapping.launch.py \
         slam/mapper_params_localization.yaml slam/mapper_params_mapping.yaml; do
    [ -f "$H/$f" ] || { echo "    MISSING: $f (not in the staged payload)" >&2; MISSING=1; }
done

chown -R "${FPMS_USER}:$FPMS_GROUP" "$H/nav2" "$H/slam"

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

chown -R "${FPMS_USER}:$FPMS_GROUP" "$H"

if [ "$MISSING" = 1 ]; then
    echo "FATAL: required rover payload files are missing or could not be installed." >&2
    echo "Every failing line above is prefixed MISSING: (not in \$SRC) or FAILED:" >&2
    echo "(present, but install/cp refused) - and those want different fixes:" >&2
    echo "  MISSING -> the staging copy did not bring the file in. Check that it" >&2
    echo "             exists two levels above fpms-os/ and re-run the full build." >&2
    echo "  FAILED  -> the file is here and the write was rejected. Check free" >&2
    echo "             space in the image and that '${FPMS_USER}:$FPMS_GROUP' resolves." >&2
    exit 1
fi
echo "--- 30-fpms-payload OK"
