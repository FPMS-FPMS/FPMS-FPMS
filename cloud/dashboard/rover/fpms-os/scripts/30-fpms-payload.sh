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

# --- the LiDAR mount calibration tool ---------------------------------------
#
# NOTHING IS INSTALLED HERE, DELIBERATELY. Both halves of the tool ship in the
# OVERLAY:
#
#     overlay/usr/local/bin/fpms-calibrate-lidar     the watch-only ROS 2 node
#     overlay/usr/local/lib/fpms/fpms_scanmatch.py   the pure-numpy estimator
#
# and stage 50 already copies overlay/usr/local/** into /, chmods
# /usr/local/bin/* to 0755, and sweeps /usr/local/bin/fpms-* for CRLF. A second
# `install` here would duplicate all three and give the image two sources of
# truth for the same two files - which is exactly how $SRC came to hold two
# rover-agent files that had drifted apart, and why the block above has to pick
# between them by line count.
#
# WHAT IS ADDED INSTEAD IS THE CHECK STAGE 50 CANNOT MAKE, plus one directory.
#
# THE ORDERING IS THE WHOLE REASON THIS BLOCK LOOKS BACK-TO-FRONT. build.sh
# iterates "$MNT/opt/fpms-os/scripts/"[0-9]*.sh in glob order, so 30 runs
# BEFORE 50: /usr/local/lib/fpms/fpms_scanmatch.py DOES NOT EXIST YET at this
# point in the build, and a `[ -f ]` against the installed path would fail on
# every single run - a check that is always red is a check that gets deleted.
# What DOES exist is build.sh's staged copy of the overlay: stage_all() copies
# scripts/ overlay/ selftest/ docs/ firstboot/ under /opt/fpms-os before any
# stage runs. So this inspects THE SOURCE STAGE 50 IS ABOUT TO COPY FROM. A
# file that is absent, empty or malformed there is absent, empty or malformed
# in the image twenty minutes later - and this stage is where somebody is still
# reading the log.
#
# Everything below reports its verdict through MISSING, like the rest of this
# stage, so one run names everything that is broken instead of dying on the
# first fault of a fourteen-hour build.

CAL_OVL=/opt/fpms-os/overlay/usr/local
CAL_LIB="$CAL_OVL/lib/fpms/fpms_scanmatch.py"
CAL_BIN="$CAL_OVL/bin/fpms-calibrate-lidar"

# The functions fpms-calibrate-lidar calls. Named one at a time rather than
# checked as "the file parses", because a rename in the library is silent: the
# module imports perfectly and the tool dies on an AttributeError at the moment
# the operator has both hands on the rover.
CAL_REQUIRED="scan_to_xy wrap_pi estimate_rotation estimate_translation \
mirror_verdict yaw_from_straight_push lever_arm_from_rotation \
plane_level_diagnostic"

# ast.parse, NOT `python3 -m py_compile` and NOT an import.
#
#   - py_compile writes __pycache__/ NEXT TO THE SOURCE, i.e. inside the staged
#     overlay, and stage 50 would then copy that bytecode into the image as
#     /usr/local/lib/fpms/__pycache__ - build scaffolding shipped to a rover,
#     stale the moment anyone edits the library.
#   - an import would execute the module and drag in numpy under qemu for no
#     extra information: the question here is whether the FILE is shaped like
#     the library the tool expects, and stage 20 has already verified numpy.
#
# Heredoc-assigned so the snippet can contain quotes of both kinds without
# fighting the shell over them.
CAL_PY_SHAPE="$(cat <<'PYEOF'
import ast, sys
path, required = sys.argv[1], sys.argv[2].split()
try:
    src = open(path, 'rb').read().decode('utf-8')
except (OSError, UnicodeDecodeError) as e:
    sys.stderr.write('cannot read as UTF-8: %s' % e)
    raise SystemExit(1)
try:
    mod = ast.parse(src, path)
except SyntaxError as e:
    sys.stderr.write('SyntaxError at line %s: %s' % (e.lineno, e.msg))
    raise SystemExit(1)
have = set()
for n in mod.body:
    if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
        have.add(n.name)
gone = [n for n in required if n not in have]
if gone:
    sys.stderr.write('parses, but these top-level functions are gone: %s'
                     % ' '.join(gone))
    raise SystemExit(1)
sys.stdout.write('parses, %d top-level functions, all %d the tool needs'
                 % (len(have), len(required)))
PYEOF
)"

if [ ! -d "$CAL_OVL" ]; then
    echo "    MISSING: $CAL_OVL - the staged overlay has no usr/local tree." >&2
    echo "             Not a calibration fault: stage_all() copies overlay/ to" >&2
    echo "             /opt/fpms-os before any stage runs, so either that copy" >&2
    echo "             did not happen or this chroot was staged by an older" >&2
    echo "             build.sh. Stage 50 will fail on the same tree." >&2
    MISSING=1
else
    # --- the library --------------------------------------------------------
    #
    # -s as well as -f. A zero-byte file copies, chmods and ships perfectly, and
    # only announces itself when the operator is standing over the rover.
    CAL_SHAPE=""
    if [ ! -f "$CAL_LIB" ]; then
        echo "    MISSING: overlay/usr/local/lib/fpms/fpms_scanmatch.py" >&2
        echo "             (expected at $CAL_LIB)" >&2
        echo "             Without it the mount transform can never be measured," >&2
        echo "             FPMS_TF_OFFSETS_MEASURED stays 0, and fpms-nav2 and" >&2
        echo "             both SLAM units refuse to start forever." >&2
        MISSING=1
    elif [ ! -s "$CAL_LIB" ]; then
        echo "    FAILED: $CAL_LIB is zero bytes." >&2
        MISSING=1
    elif ! CAL_SHAPE="$(/usr/bin/python3 -c "$CAL_PY_SHAPE" "$CAL_LIB" "$CAL_REQUIRED" 2>&1)"; then
        echo "    FAILED: fpms_scanmatch.py is not importable-shaped." >&2
        echo "            python3 said: $CAL_SHAPE" >&2
        echo "            fpms-calibrate-lidar imports it at start-up, so this is" >&2
        echo "            a tool that dies on its first line, on a rover." >&2
        MISSING=1
    else
        echo "    overlay lib: fpms_scanmatch.py - $CAL_SHAPE"
    fi

    # --- the node -----------------------------------------------------------
    #
    # The shebang is read with `read`, not with `head -c`/`grep`. This file
    # comes off a Windows checkout, so a \r is a live possibility, and a
    # producer piped into head or grep -q takes SIGPIPE, pipefail reports 141,
    # and `set -e` kills the stage silently - the bug class that has killed
    # this build three times. `read` opens the file directly: no pipeline, no
    # producer, nothing to signal. It also keeps a trailing \r, which is the
    # byte being looked for.
    if [ ! -f "$CAL_BIN" ]; then
        echo "    MISSING: overlay/usr/local/bin/fpms-calibrate-lidar" >&2
        echo "             (expected at $CAL_BIN)" >&2
        echo "             The library alone measures nothing - it is a set of" >&2
        echo "             pure functions with no ROS and no I/O. This is the" >&2
        echo "             node that subscribes to /scan_lidar and /imu." >&2
        MISSING=1
    elif [ ! -s "$CAL_BIN" ]; then
        echo "    FAILED: $CAL_BIN is zero bytes." >&2
        MISSING=1
    else
        CAL_FIRST=""
        read -r CAL_FIRST < "$CAL_BIN" || true
        case "$CAL_FIRST" in
            '#!'*) ;;
            *)
                echo "    FAILED: $CAL_BIN does not start with '#!'." >&2
                echo "            Stage 50 chmods it 0755 regardless, so it would" >&2
                echo "            ship executable and fail at exec." >&2
                MISSING=1 ;;
        esac
        # A note, not a failure: stage 50's CRLF sweep covers
        # /usr/local/bin/fpms-* and repairs this one after the copy. Said out
        # loud anyway, because the sweep does NOT cover /usr/local/lib/fpms,
        # and a reader who sees this line knows which of the two was fixed.
        case "$CAL_FIRST" in
            *$'\r')
                echo "    note: fpms-calibrate-lidar has a CRLF shebang in the staged" >&2
                echo "          overlay. Stage 50 converts it; check .gitattributes." >&2 ;;
        esac
        echo "    overlay bin: fpms-calibrate-lidar (stage 50 installs it 0755)"
    fi
fi

# --- /usr/local/lib/fpms must be ON THE INTERPRETER'S PATH -------------------
#
# fpms_scanmatch.py is a bare module in a directory no Python has ever heard
# of. Nothing about copying it to /usr/local/lib/fpms makes `import
# fpms_scanmatch` work, and the failure is an ImportError at start-up on the
# rover, which is the worst possible place to discover it.
#
# The directory is created HERE rather than left to stage 50's `cp -a`, for two
# reasons that both bite:
#
#   1. site.addpackage SILENTLY DROPS a .pth line naming a directory that does
#      not exist. Written before the directory, the .pth below would be a file
#      that looks completely correct and adds nothing at all - and the sys.path
#      assertion after it would fail here, at build time, for a reason that has
#      nothing to do with the tool.
#   2. cp -a stamps the SOURCE directory's mode onto the destination, and this
#      overlay is staged from a Windows filesystem where every directory reads
#      back 0777. Creating it 0755 root:root first means stage 50's DIR_SNAP
#      block has a real mode to snapshot and put back. A world-writable
#      directory on every interpreter's sys.path is a place any process on the
#      rover could drop a module that every other one then imports.
#
# Nothing here can shadow anything: the directory holds one module, named
# fpms_scanmatch, which collides with no stdlib and no ROS package. It is
# appended by site, after the stdlib, not prepended.
install -d -m 0755 -o root -g root /usr/local/lib/fpms

# Ask the interpreter where its site directories are rather than writing
# python3.10 into this file. jammy is 3.10 today; a hardcoded path that stops
# existing produces a .pth nothing reads, which fails in the direction of a
# clean build log.
CAL_PY_SITE="$(cat <<'PYEOF'
import site, sys
try:
    dirs = [d for d in site.getsitepackages() if isinstance(d, str)]
except AttributeError:
    dirs = []
pref = [d for d in dirs if d.startswith('/usr/local/')]
cand = pref or dirs
if not cand:
    sys.stderr.write('python3 reports no site-packages directories at all')
    raise SystemExit(1)
sys.stdout.write(cand[0])
PYEOF
)"

CAL_SITE=""
if ! CAL_SITE="$(/usr/bin/python3 -c "$CAL_PY_SITE" 2>&1)"; then
    echo "    FAILED: could not ask python3 for its site directories." >&2
    echo "            python3 said: $CAL_SITE" >&2
    MISSING=1
    CAL_SITE=""
fi
if [ -n "$CAL_SITE" ]; then
    install -d -m 0755 -o root -g root "$CAL_SITE"
    printf '%s\n' /usr/local/lib/fpms > "$CAL_SITE/fpms-scanmatch.pth"
    chmod 0644 "$CAL_SITE/fpms-scanmatch.pth"
    chown root:root "$CAL_SITE/fpms-scanmatch.pth"
    # VERIFY THE OUTCOME, not that the write returned 0. The only question
    # worth asking is whether /usr/bin/python3 - the interpreter every unit
    # runs, either directly or as `python3` after sourcing ROS's setup.bash,
    # which changes PYTHONPATH but not the interpreter - actually has the
    # directory on sys.path now.
    if /usr/bin/python3 -c 'import sys; raise SystemExit(0 if "/usr/local/lib/fpms" in sys.path else 1)'; then
        echo "    $CAL_SITE/fpms-scanmatch.pth  (/usr/local/lib/fpms is on sys.path)"
    else
        echo "    FAILED: wrote $CAL_SITE/fpms-scanmatch.pth and /usr/local/lib/fpms" >&2
        echo "            is STILL not on python3's sys.path. Either that directory" >&2
        echo "            is not a site directory this interpreter reads, or .pth" >&2
        echo "            processing is disabled (python3 -S, or a venv). Until it" >&2
        echo "            is fixed, fpms-calibrate-lidar must add the path itself." >&2
        MISSING=1
    fi
fi

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

       scp -r ubuntu@<old-pi>:~/yolo ubuntu@fpms-rover1.local:~/
       scp -r ubuntu@<old-pi>:~/fpms_console ubuntu@fpms-rover1.local:~/

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
    echo "             EXCEPT for the two calibration files: those ship in" >&2
    echo "             fpms-os/overlay/usr/local/ and their message names the" >&2
    echo "             full staged path. Look there, not above fpms-os/." >&2
    echo "  FAILED  -> the file is here and the write was rejected. Check free" >&2
    echo "             space in the image and that '${FPMS_USER}:$FPMS_GROUP' resolves." >&2
    exit 1
fi
echo "--- 30-fpms-payload OK"
