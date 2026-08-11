#!/usr/bin/env bash
# Stage 25 - the NPU runtime, and a version chain that can be checked instead
# of hoped at.
#
# WHAT THIS STAGE OWNS
# ====================
#   /usr/lib/librknnrt.so          the userspace runtime
#   rknn-toolkit-lite2             the aarch64 Python wheel (NOT on PyPI)
#   /etc/fpms/npu-versions.json    the build record all three of them go into
#
# It supersedes the "--- the NPU" block in 20-python-deps.sh.
#
#   BOTH HANDOVERS TO STAGE 20'S OWNER HAVE BEEN TAKEN UP. Re-read 2026-08-11,
#   so the warnings that used to stand here are deleted rather than left to rot
#   into a lie that a later reader has to disprove:
#
#     - the duplicate download is GONE. scripts/20-python-deps.sh says
#       "WHAT IT NO LONGER OWNS: the NPU", and its "--- the NPU" block is a
#       pointer to this stage with no wget in it. There is exactly one
#       librknnrt.so download in this repository now, and it is below.
#     - --break-system-packages is DETECTED in stage 20 too - same
#       `pip3 install --help` probe as the one below, under its
#       "--- how to talk to this pip" heading - so the build no longer dies in
#       stage 20 on pip 22.0.2 before it can reach here.
#
#   NO LINE NUMBERS INTO STAGE 20 ANY MORE, and that is a correction, not a
#   style preference. This block used to cite "line 210 onward" and "lines
#   178-187"; re-checked 2026-08-11 those are line 249 and lines 175-228. The
#   references rotted within days of being written and a reader who followed
#   them landed in the middle of the paho-mqtt note. Cite the headings instead:
#   they are greppable, and they move with the text they name.
#     grep -n 'the NPU\|how to talk to this pip' scripts/20-python-deps.sh
#
#   Still do NOT edit stage 20 from here. Two agents editing one script is how
#   this repo gets a third near-identical copy of something.
#
# WHY A SEPARATE STAGE AT ALL
# ===========================
# Because "install the NPU runtime" is not a Python dependency. It is a
# three-way version contract between a kernel module this image does not
# build, a shared object, and a wheel - and NPU_SPEC.md rule 4 says match it
# by MEASUREMENT, not by hope. A stage that installs it and records what it
# installed is the measurement. See npu/versions/README.md for the contract.
set -euo pipefail
echo "--- 25-npu-runtime"

WHEEL_URL_CONF="${RKNN_LITE_WHEEL_URL:?RKNN_LITE_WHEEL_URL not set - build.sh must export it}"
SO_URL_CONF="${LIBRKNNRT_URL:?LIBRKNNRT_URL not set - build.sh must export it}"
TAG="${RKNN_TOOLKIT2_TAG:-unknown}"
SO_DEST=/usr/lib/librknnrt.so
JSON=/etc/fpms/npu-versions.json

# What was actually used, which is not necessarily what was configured - see
# "RESOLVING THE URLS AT BUILD TIME". Recorded in the JSON alongside the
# configured value so a divergence is visible in the image rather than only in
# a build log nobody kept.
SO_URL_USED="$SO_URL_CONF"
WHEEL_URL_USED="$WHEEL_URL_CONF"

# `if !`, not a bare `TMP="$(mktemp -d)"`. A plain assignment ADOPTS the exit
# status of its command substitution, so under `set -e` a failing mktemp kills
# this stage on its fourth line - before the trap below exists, before anything
# has been echoed, and with only mktemp's own one-liner on stderr to explain a
# stage that appeared to do nothing at all. This is the same shape that has
# killed this build twice; it is spelled out everywhere else in this file and
# it had no business being unguarded here.
if ! TMP="$(mktemp -d)"; then
    echo "FATAL: mktemp -d failed inside the chroot." >&2
    echo "This stage stages a 7 MB download through it. Check /tmp exists and the" >&2
    echo "image has free space (build.sh's report_space prints the numbers)." >&2
    exit 1
fi
trap 'rm -rf "$TMP"' EXIT

# /etc/fpms must exist NOW. The overlay that creates it is applied in stage 50,
# which runs LATER, so this stage cannot rely on it.
#
# RE-VERIFIED 2026-08-11: scripts/00-base-system.sh line 583 does
# `install -d -m 0755 /etc/fpms`, so on a full build the directory is already
# there.
#
# THE REASON THIS LINE EXISTS HAS CHANGED, and the old reason is now wrong, so
# it is restated rather than left standing. build.sh no longer calls
# prepare_image() for --stage/--from: main() (build.sh line 1016) calls
# resume_image() (line 611), which re-attaches the EXISTING $OUT image instead
# of re-copying the vendor base - precisely so that a stage failing 90 minutes
# in can be re-run without throwing away stage 00 and stage 10. So on the
# ordinary `--stage 25` re-run, /etc/fpms is already there and this line is a
# no-op.
#
# It is still NOT redundant, because of `--fresh`. build.sh line 1019-1027:
# --stage/--from with no image present REQUIRES --fresh, and --fresh takes the
# prepare_image() branch - a pristine base rootfs on which stage 00 has never
# run - and then runs ONLY this stage. Without this line
# `sudo ./build.sh --stage 25 --fresh` fails at write_json with "No such file
# or directory", ~15 minutes of download after the last thing it printed.
# `install -d` is idempotent, so the cost of being right in both worlds is one
# syscall.
install -d -m 0755 /etc/fpms

# ---------------------------------------------------------------------------
# THE BUILD RECORD
# ---------------------------------------------------------------------------
#
# Defined up here, before anything that can fail, because the failure path
# writes it too - an image that skipped the NPU must still carry a record
# saying so. (Shell functions must be defined before the line that calls them
# runs; putting this after the download would mean the download's own failure
# handler called an undefined function.)
#
# EVERY FIELD IS A MEASUREMENT OR A NULL. It used to write
#
#     "import_ok": status == "INSTALLED",
#     "rknnlite_constructible": status == "INSTALLED",
#
# which is not a measurement, it is the status restated in two extra places.
# Anything that ever read those fields would have been reading a tautology
# dressed as evidence - SPEC.md rule 2, and precisely the habit this whole
# directory exists to break. They now come from the verify step's own result,
# through the environment, and are null when the verify step never ran.
#
# THE KERNEL DRIVER VERSION CANNOT BE READ AT BUILD TIME. Say it plainly,
# because the temptation to paper over it is exactly how the old Pi ended up
# with no build record at all:
#
#   - the driver is a module in the base image's Rockchip BSP kernel, which
#     FPMS-OS does not build and does not boot;
#   - its version is exposed at /sys/kernel/debug/rknpu/version, which is
#     debugfs on a LIVE kernel. The chroot's /sys is the BUILD HOST's sysfs
#     (build.sh bind-mounts it), so reading it here would report the x86
#     workstation's kernel, or nothing - never the rover's.
#
# So this file records the EXPECTED driver version and hands the comparison to
# boot time. Agent 7's fpms-npu-selftest is the consumer: it reads
# /sys/kernel/debug/rknpu/version on the running board and compares it against
# driver.expected here.
#
# There is no measured expected value yet. `driver.expected` is null and stays
# null until somebody reads it off real hardware - NPU_SPEC.md §4 and SPEC.md
# rule 2: never claim a measurement you did not take. Once it IS measured, pin
# it by adding RKNN_EXPECTED_DRIVER_VERSION to config/fpms-os.conf (agent A's
# file) and to build.sh's in_chroot() env list, and this stage will record it.
write_json() {   # write_json <status>
    local status="$1"
    # The exit status of this heredoc IS consulted (`if ! python3 ...`), which
    # it was not before 2026-08-11. It used to be a bare command: a python that
    # died left `chmod 0644 "$JSON"` to fail on a file that was never created,
    # and the operator got "chmod: cannot access" as the diagnosis for "the
    # build record could not be written".
    #
    # And this is FATAL even on the ALLOW_NPU_MISSING path that calls it from
    # npu_fail. An image with no NPU and no record saying so is exactly the
    # artifact this stage exists to make impossible - indistinguishable from a
    # good one. If we cannot write the record, we do not ship the image.
    if ! python3 - "$JSON" "$status" <<'PY'
import json, os, sys, time

out, status = sys.argv[1], sys.argv[2]

def env(k, default=None):
    v = os.environ.get(k)
    return v if v else default

def flag(k):
    """Tri-state. A check that never ran is null, not False - "we did not look"
    and "we looked and it was broken" are different statements and the record
    must not blur them."""
    v = os.environ.get(k)
    if v == "1":
        return True
    if v == "0":
        return False
    return None

doc = {
    "schema": 2,               # 2: measured import/construct flags, url_used
    "generated_by": "fpms-os scripts/25-npu-runtime.sh",
    # strftime, not subprocess date(1). One fewer process to spawn under
    # qemu-user, where every exec is an emulator start-up.
    "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "fpms_os_version": env("FPMS_OS_VERSION"),
    "status": status,          # INSTALLED | ABSENT
    "rknn_toolkit2_tag": env("RKNN_TOOLKIT2_TAG"),

    # --- link 2 of the chain: the userspace runtime -----------------------
    "librknnrt": {
        "path": "/usr/lib/librknnrt.so",
        "version": env("SO_VER"),
        "version_string": env("SO_VER_FULL"),
        "sha256": env("SO_SHA"),
        "size_bytes": int(env("SO_SIZE", "0") or 0) or None,
        "url": env("SO_URL_USED") or env("LIBRKNNRT_URL"),
        "url_configured": env("LIBRKNNRT_URL"),
        "url_was_resolved_at_build_time":
            bool(env("SO_URL_USED")) and env("SO_URL_USED") != env("LIBRKNNRT_URL"),
        # The check that actually proves this file is usable. RKNNLite() does
        # NOT load it - see the verify section of stage 25.
        "dlopen_ok": flag("DLOPEN_OK"),
        "read_with": "strings /usr/lib/librknnrt.so | grep -i 'librknnrt version'",
    },

    # --- link 3 of the chain: the Python wheel ----------------------------
    "wheel": {
        "name": "rknn-toolkit-lite2",
        "version": env("WHEEL_VER"),
        "url": env("WHEEL_URL_USED") or env("RKNN_LITE_WHEEL_URL"),
        "url_configured": env("RKNN_LITE_WHEEL_URL"),
        "url_was_resolved_at_build_time":
            bool(env("WHEEL_URL_USED"))
            and env("WHEEL_URL_USED") != env("RKNN_LITE_WHEEL_URL"),
        "python_tag": "cp310",
        "platform_tag": "manylinux_2_17_aarch64",
        "import_ok": flag("IMPORT_OK"),
        "rknnlite_constructible": flag("CONSTRUCT_OK"),
        "read_with": "pip3 show rknn-toolkit-lite2",
        "note": ("import + construct only. load_rknn()/init_runtime() cannot "
                 "run in a chroot: no NPU device, and the .rknn model is not "
                 "in the repository. NOTE that RKNNLite() does not touch "
                 "librknnrt.so either - librknnrt.dlopen_ok is the field that "
                 "says anything about link 2."),
    },

    # --- link 1 of the chain: the kernel driver ---------------------------
    "driver": {
        "expected": env("RKNN_EXPECTED_DRIVER_VERSION"),
        "expected_source": (
            "MEASURE ME - unset. Nobody has ever written down which rknpu "
            "driver version the old Pi ran, and the board for this image has "
            "not arrived. Pin it in config/fpms-os.conf as "
            "RKNN_EXPECTED_DRIVER_VERSION once it has been read off hardware."),
        "actual_read_at": "/sys/kernel/debug/rknpu/version",
        "read_with": "sudo cat /sys/kernel/debug/rknpu/version",
        "read_without_root": (
            "RKNNLite().get_sdk_version() reports both the API (librknnrt) and "
            "DRV (kernel driver) versions after a successful init_runtime(). "
            "VERIFY ON HARDWARE."),
        "why_not_recorded_at_build_time": (
            "The rknpu driver lives in the base image's Rockchip BSP kernel, "
            "which this build never boots. /sys inside the chroot is the build "
            "HOST's sysfs, bind-mounted by build.sh, so reading it here would "
            "report the workstation's kernel - not the rover's. The comparison "
            "is therefore deferred to boot: fpms-npu-selftest reads the live "
            "value and checks it against driver.expected."),
    },

    "numpy_version": env("NUMPY_VER"),

    "verify_on_hardware": [
        "sudo cat /sys/kernel/debug/rknpu/version",
        "strings /usr/lib/librknnrt.so | grep -i 'librknnrt version'",
        "pip3 show rknn-toolkit-lite2",
        "ls -l /dev/rknpu* /dev/dri/",
        "sudo -u ubuntu python3 -c \"from rknnlite.api import RKNNLite; "
        "r=RKNNLite(); r.load_rknn('/home/ubuntu/yolo/yolo26n-rk3588.rknn'); "
        "print('init_runtime ->', r.init_runtime()); print(r.get_sdk_version())\"",
    ],
    "doc": "fpms-os/npu/versions/README.md",
}

# encoding= is explicit, and not decoration. build.sh's in_chroot() runs
# `env -i ... LC_ALL=C`, and what python does with a C locale is a moving
# target across versions and build options (PEP 538 locale coercion is skipped
# when LC_ALL is set; PEP 540 UTF-8 mode then usually rescues it - "usually"
# being the operative word). Naming the codec means this stage does not depend
# on which of those two applies. json.dump defaults to ensure_ascii=True, so
# the bytes written are ASCII either way; this only pins how they get there.
with open(out, "w", encoding="utf-8") as fh:
    json.dump(doc, fh, indent=2)
    fh.write("\n")
PY
    then
        echo "FATAL: could not write the NPU build record $JSON" >&2
        echo "An image whose NPU state is unrecorded is indistinguishable from a good" >&2
        echo "one. Refusing to continue. Check that /etc/fpms exists and is writable." >&2
        exit 1
    fi
    chmod 0644 "$JSON"
    echo "    $JSON  (status=$status)"
}

# ---------------------------------------------------------------------------
# FAILURE POLICY - and why it differs from stage 20's
# ---------------------------------------------------------------------------
#
# Stage 20 prints a large warning and CONTINUES, on the argument that "a rover
# that patrols and streams is still useful". That argument is right for stage
# 20 and wrong for this one.
#
# Stage 20 installs a dozen things. The NPU was one item on its list, and the
# other eleven still produce a rover that drives, stops, maps and reports. A
# DEDICATED NPU stage that cannot install the NPU has accomplished NOTHING,
# and what it hands the operator is worse than nothing: an .img that is
# indistinguishable from a good one - same name, same size, boots the same,
# every unit active - and detects nothing. That is precisely the failure
# NPU_SPEC.md §0 exists to make impossible, manufactured at build time.
#
# So: THIS STAGE FAILS THE BUILD. A failed build is loud, immediate, and
# happens on a workstation with a network connection and the operator sitting
# in front of it - which is the cheapest possible place for this to be found.
#
# The stream-only image is still buildable, but only DELIBERATELY, and it
# records that it is crippled so it can never be mistaken for a good one.
#
# HOW TO ACTUALLY SET IT. This used to say
#
#     sudo touch .build/mnt/opt/fpms-os/ALLOW_NPU_MISSING
#
# which could not work, because stage_all() opens with
# `rm -rf '$MNT/opt/fpms-os'` and re-copies the repo's tree over it. RE-READ
# 2026-08-11: build.sh has since closed BOTH halves of that hole, so the
# mechanism below is checked against the build.sh in this worktree rather than
# against the one that existed when the escape hatch was written.
#
#   1. build.sh line 819-826 now saves the marker across its own wipe:
#          local keep_npu_marker=0
#          [ -e "$MNT/opt/fpms-os/ALLOW_NPU_MISSING" ] && keep_npu_marker=1
#          run "rm -rf '$MNT/opt/fpms-os'" ; ... ; touch it back
#      so the second candidate path below survives.
#   2. build.sh line 797 now passes FPMS_ALLOW_NPU_MISSING through the `env -i`
#      list in CHROOT_ENV, so the environment variable is live on the normal
#      path and not only inside `build.sh --shell`.
#
# THREE ways in, all checked below, in the order they are looked for:
#
#   a. touch scripts/ALLOW_NPU_MISSING       # in the repo, on the host
#      sudo ./build.sh                       # or --stage 25
#      rm scripts/ALLOW_NPU_MISSING          # afterwards, deliberately
#
#      Lands at /opt/fpms-os/scripts/ALLOW_NPU_MISSING because build.sh copies
#      the whole scripts/ directory in. build.sh globs scripts/[0-9]*.sh, so a
#      file with no leading digit is never executed as a stage, and
#      `chmod +x .../scripts/*.sh` does not touch it either. PREFER THIS ONE:
#      it shows up in `git status`, which is the point. A deliberately crippled
#      image should be hard to produce by accident and impossible to produce
#      without leaving a trace.
#
#   b. sudo touch .build/mnt/opt/fpms-os/ALLOW_NPU_MISSING
#      Now works, per (1) - but only for runs that resume this image, and it
#      leaves no trace in the repository. It is honoured, not recommended.
#
#   c. sudo FPMS_ALLOW_NPU_MISSING=1 ./build.sh --stage 25
#      NOTE THE PLACEMENT. It must be on the sudo command line: sudo's default
#      env_reset drops an exported FPMS_ALLOW_NPU_MISSING from the operator's
#      shell before build.sh ever sees it, and build.sh then substitutes its
#      `${FPMS_ALLOW_NPU_MISSING:-0}` default. `export FPMS_...=1; sudo ./build.sh`
#      silently does nothing.
#
# The third path, /etc/fpms/ALLOW_NPU_MISSING, is inside the rootfs itself. It
# survives a resume but not a --fresh, and unlike (a) and (b) it would SHIP in
# the finished image (stage 50 chmods everything in /etc/fpms and removes
# nothing). Do not use it deliberately; it is checked because an older
# procedure told people to create it and a marker that is silently ignored is
# worse than one that is honoured.
#
# When the marker is present the stage writes npu-versions.json with
# "status": "ABSENT".
#
#   !! HONESTY NOTE, checked 2026-08-10 against the overlay as it stands:
#   !! nothing reads that field yet. fpms-npu-selftest's check_versions()
#   !! PASSes on the mere existence of the file, and fpms-npud only records
#   !! `npu_versions_recorded: os.path.exists(...)`. A degraded image is still
#   !! caught every boot, but by the selftest's real-inference probe, not by
#   !! this field. Agent 7 owns closing that: treat status == "ABSENT" as an
#   !! immediate FAIL with its own remedy text, so the reason is reported
#   !! rather than re-derived. Do not claim here that it already does.
ALLOW_MISSING=0
ALLOW_MARKER=""
for m in /opt/fpms-os/scripts/ALLOW_NPU_MISSING \
         /opt/fpms-os/ALLOW_NPU_MISSING \
         /etc/fpms/ALLOW_NPU_MISSING; do
    if [ -e "$m" ]; then ALLOW_MISSING=1; ALLOW_MARKER="$m"; break; fi
done
if [ "$ALLOW_MISSING" = 0 ] && [ "${FPMS_ALLOW_NPU_MISSING:-0}" = 1 ]; then
    ALLOW_MISSING=1; ALLOW_MARKER="FPMS_ALLOW_NPU_MISSING=1 in the environment"
fi
if [ "$ALLOW_MISSING" = 1 ]; then
    echo "    ALLOW_NPU_MISSING ($ALLOW_MARKER)" >&2
    echo "    NPU failures will NOT fail the build; the image will be recorded ABSENT" >&2
fi

npu_fail() {   # npu_fail <what> <how to fix on the running rover>
    cat >&2 <<EOF

########################################################################
 NPU RUNTIME NOT INSTALLED: $1

 An FPMS-OS image without the NPU runtime STREAMS VIDEO AND DETECTS
 NOTHING, and does not say so. RKNNLite.init_runtime() returns an int
 rather than raising, and fpms_rover_agent wraps the whole NPU block in
 one try/except that logs "NPU unavailable; streaming without
 detection" once and carries on. Every unit reports active.

 Fix and re-run:   sudo ./build.sh --stage 25

 Or fix on a running rover, then re-run fpms-npu-selftest:
   $2

########################################################################

EOF
    if [ "$ALLOW_MISSING" = 1 ]; then
        echo "    continuing anyway (ALLOW_NPU_MISSING); recording status=ABSENT" >&2
        write_json ABSENT
        echo "--- 25-npu-runtime DEGRADED (NPU absent, recorded in $JSON)"
        exit 0
    fi
    exit 1
}

# ---------------------------------------------------------------------------
# Platform preconditions
# ---------------------------------------------------------------------------
#
# The wheel filename encodes cp310 and aarch64. Both are hard: Humble's rclpy
# extensions are cpython-310-aarch64 (SPEC.md §1), so if either of these is
# wrong the chroot itself is wrong and every later stage is building the wrong
# image. pip would eventually say "not a supported wheel on this platform",
# which is a true but unhelpful message ~200 lines into a log.
#
# These use plain `exit 1`, NOT npu_fail: a chroot that is not aarch64/3.10 is
# not an NPU problem and ALLOW_NPU_MISSING must not wave it through.
#
# PYV IS CAPTURED THROUGH AN `if !`, not a bare assignment. `PYV="$(python3
# ...)"` adopts python3's exit status, so a python that is missing or that
# segfaults under qemu-user takes the stage out HERE - three lines before the
# message that would have named the problem, and with nothing of ours on
# stderr. That is the precise failure this block exists to prevent, and the
# block was committing it itself. `uname -m` is left as a plain assignment: it
# is a builtin-grade syscall wrapper with no failure mode short of a broken
# coreutils, and an unreadable arch is caught by the string compare below.
ARCH="$(uname -m)"
if ! PYV="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"; then
    echo "FATAL: could not run python3 in the chroot to read its version." >&2
    echo "Every stage from 20 onward needs it, and this stage needs cp310 exactly." >&2
    echo "Check that qemu-aarch64 binfmt is registered and that stage 00 completed." >&2
    exit 1
fi
[ "$ARCH" = "aarch64" ] || { echo "FATAL: chroot reports arch '$ARCH', expected aarch64." >&2
    echo "The rknn-toolkit-lite2 wheel is aarch64-only. Is qemu-aarch64 binfmt registered?" >&2; exit 1; }
[ "$PYV" = "3.10" ] || { echo "FATAL: chroot python is $PYV, expected 3.10." >&2
    echo "The wheel is cp310-only, and ROS Humble's rclpy extensions are cpython-310." >&2; exit 1; }
echo "    arch=$ARCH python=$PYV toolkit2_tag=$TAG"

# ---------------------------------------------------------------------------
# RESOLVING THE URLS AT BUILD TIME
# ---------------------------------------------------------------------------
#
# config/fpms-os.conf is not this agent's file, and a wrong URL in it used to
# mean a dead stage with a wget exit code for a diagnosis. There is precedent
# in that very file: BASE_IMAGE_URL once named "ubuntu-22.04.4-..." - a point
# release upstream does not publish - and 404'd.
#
# BOTH NPU URLS WERE VERIFIED 2026-08-10 AND ARE CORRECT AS CONFIGURED:
#
#   tag v2.3.0                       exists (api.github.com .../git/ref/tags/v2.3.0 -> 200)
#   rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so
#                                    200, 7,259,064 bytes,
#                                    sha256 73993ed4b440460825f21611731564503c
#                                           c1d5a0c123746477da6cd574f34885,
#                                    ELF64/EM_AARCH64, SONAME librknnrt.so,
#                                    "librknnrt version: 2.3.0
#                                     (c949ad889d@2024-11-07T11:35:33)"
#   rknn-toolkit-lite2/packages/rknn_toolkit_lite2-2.3.0-cp310-cp310-
#     manylinux_2_17_aarch64.manylinux2014_aarch64.whl
#                                    200, 559,372 bytes,
#                                    sha256 4b6733689bd09a262bcb6ba4744e690dd4
#                                           b37ebeac4ed427cf45242c4b4ce9a4
#
#   The "Ubuntu" spelling of the runtime directory - rknpu2/runtime/Ubuntu/... -
#   is a 404 AT THIS TAG. It is a real path in other eras of that repository,
#   which is why it is a fallback candidate below and not an assumption.
#
# So the happy path is a plain fetch of the configured URL. This resolver only
# runs when that fetch does not produce the right BYTES, and then it:
#
#   1. tries the known path variants Rockchip has used for the same artifact;
#   2. asks the GitHub contents API what is actually at that tag - which is how
#      a wheel with a build suffix in its filename gets found;
#   3. failing everything, prints every URL it tried, the HTTP status of each,
#      and the exact command to find the right one by hand.
#
# It validates by CONTENT, not by status code: an artifact is only accepted if
# its first bytes are an aarch64 ELF64 (.so) or a zip local header (.whl). A
# 200 that returns an HTML error page is rejected here rather than installed.
cat > "$TMP/resolve_url.py" <<'PY'
"""Resolve a Rockchip artifact URL at build time.

    resolve_url.py so    <configured-url>
    resolve_url.py wheel <configured-url>

Prints the URL that actually serves the right bytes on stdout. All diagnostics
go to stderr. Exit 1 if nothing resolves.

stdlib only, on purpose: this runs in a chroot where the only guaranteed
Python is the interpreter itself.
"""
import re
import sys
import json
import time
import urllib.error
import urllib.request

# Diagnostics must never be the thing that kills the resolver. Under
# `env -i ... LC_ALL=C` sys.stdout can come up with a strict ASCII codec, and a
# single non-ASCII byte in an upstream path or an exception message would then
# raise UnicodeEncodeError out of a sys.stderr.write() - turning "here are the
# eight URLs I tried" into a traceback. stderr already defaults to
# backslashreplace; this says so for both streams instead of relying on it.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="backslashreplace")
    except Exception:                                       # noqa: BLE001
        pass

UA = "fpms-os-25-npu-runtime"
TIMEOUT = 30
API = "https://api.github.com"

# TERMINATION, stated as numbers rather than hoped for. candidates() is finite
# by construction (a fixed 3x2 path grid, then two non-recursive directory
# listings), but "finite" is not the same as "finishes before the operator goes
# home": every candidate is a network probe with a 30 s timeout, and an upstream
# directory that grew could make that a lot of probes. These two caps make the
# worst case arithmetic instead of an argument.
MAX_CANDIDATES = 24
DEADLINE_S = 300
STARTED = time.monotonic()

if len(sys.argv) != 3 or sys.argv[1] not in ("so", "wheel"):
    sys.stderr.write("usage: resolve_url.py <so|wheel> <configured-url>\n")
    sys.exit(2)
kind, configured = sys.argv[1], sys.argv[2]
tried = []


def _get(url, headers=None, rng=None):
    h = {"User-Agent": UA}          # GitHub rejects requests with no UA
    if headers:
        h.update(headers)
    if rng:
        h["Range"] = rng
    return urllib.request.urlopen(
        urllib.request.Request(url, headers=h), timeout=TIMEOUT)


def probe(url):
    """(ok, size, head_bytes, note). Fetches only the first 64 bytes, so a
    misfire costs nothing even against a 7 MB artifact over a slow link."""
    try:
        with _get(url, rng="bytes=0-63") as r:
            body = r.read(64)
            size = None
            m = re.search(r"/(\d+)\s*$", r.headers.get("Content-Range") or "")
            if m:
                size = int(m.group(1))
            elif r.headers.get("Content-Length"):
                try:
                    size = int(r.headers["Content-Length"])
                except ValueError:
                    pass
            return True, size, body, "HTTP %s" % getattr(r, "status", 200)
    except urllib.error.HTTPError as e:
        return False, None, b"", "HTTP %s" % e.code
    except Exception as e:                                  # noqa: BLE001
        return False, None, b"", "%s: %s" % (type(e).__name__, e)


def content_ok(head):
    """The whole point of this function: a 404 HTML page also arrives with a
    200 from some mirrors, and always arrives with a .so or .whl name."""
    if kind == "so":
        return (len(head) >= 20
                and head[:4] == b"\x7fELF"
                and head[4] == 2                                   # ELF64
                and int.from_bytes(head[18:20], "little") == 183)  # EM_AARCH64
    return head[:2] == b"PK"                                       # zip = wheel


def check(url):
    # A URL we cannot even emit as ASCII is not a URL wget can be handed, and
    # printing it is how this function reports. Reject it here, with a note, so
    # it appears in the tried list like any other miss.
    try:
        url.encode("ascii")
    except UnicodeEncodeError:
        tried.append((url, "not an ASCII URL; refusing to hand it to wget"))
        return False
    ok, size, head, note = probe(url)
    if ok and not content_ok(head):
        ok, note = False, "%s but the bytes are not a %s (first 16: %r)" % (
            note, "aarch64 ELF64" if kind == "so" else "zip/wheel", head[:16])
    tried.append((url, note if not ok else "%s, %s bytes  <-- USED" % (
        note, size if size is not None else "unknown")))
    return ok


def parse(url):
    """-> (owner, repo, ref, path) for a github raw URL, else None."""
    m = re.match(r"^https://github\.com/([^/]+)/([^/]+)/raw/([^/]+)/(.+)$", url)
    if m:
        return m.groups()
    m = re.match(r"^https://raw\.githubusercontent\.com/([^/]+)/([^/]+)/([^/]+)/(.+)$",
                 url)
    return m.groups() if m else None


def raw(owner, repo, ref, path):
    return "https://raw.githubusercontent.com/%s/%s/%s/%s" % (owner, repo, ref, path)


def api_listdir(owner, repo, ref, path):
    """Directory listing at a ref. [] on any error - this is a fallback path
    and a rate-limited API must not turn a network hiccup into a traceback."""
    url = "%s/repos/%s/%s/contents/%s?ref=%s" % (API, owner, repo, path, ref)
    try:
        with _get(url, headers={"Accept": "application/vnd.github+json"}) as r:
            data = json.loads(r.read().decode("utf-8", "replace"))
        return data if isinstance(data, list) else []
    except Exception as e:                                  # noqa: BLE001
        sys.stderr.write("    (contents API %s: %s)\n" % (path, e))
        return []


def dirs(entries):
    """Directory entries that carry both the fields the caller goes on to use.
    Filtering here is what lets the callers below index d["path"]/sub["name"]
    without a KeyError."""
    for e in entries:
        if (isinstance(e, dict) and e.get("type") == "dir"
                and e.get("path") and e.get("name")):
            yield e


def candidates():
    """The configured URL first, always."""
    yield configured
    p = parse(configured)
    if not p:
        return
    owner, repo, ref, path = p

    if kind == "so":
        # Rockchip has shipped the same file under both spellings of the OS
        # directory and both spellings of the arch directory across releases.
        for osdir in ("Linux", "Ubuntu", "Android"):
            for arch in ("aarch64", "arm64"):
                alt = re.sub(r"runtime/[^/]+/librknn_api/[^/]+/",
                             "runtime/%s/librknn_api/%s/" % (osdir, arch), path)
                if alt != path:
                    yield raw(owner, repo, ref, alt)
        # Then ask the repository itself, one directory level at a time.
        # (Not the recursive trees API: rknn-toolkit2 is large enough that a
        # recursive tree can come back truncated, and a truncated tree that
        # happens not to contain the file is indistinguishable from absence.)
        # .get() with a default on EVERY field, never d["path"]. The contents
        # API is a fallback that runs precisely when things are already wrong,
        # and one unexpected entry shape raising KeyError out of this generator
        # would abort the loop below BEFORE it prints the tried list - losing
        # the one diagnostic this resolver exists to produce.
        for d in dirs(api_listdir(owner, repo, ref, "rknpu2/runtime")):
            for sub in dirs(api_listdir(
                    owner, repo, ref, "%s/librknn_api" % d["path"])):
                if "arch64" not in sub["name"] and sub["name"] != "arm64":
                    continue
                for f in api_listdir(owner, repo, ref, sub["path"]):
                    if not isinstance(f, dict) or not f.get("path"):
                        continue
                    if f.get("name") == "librknnrt.so":
                        yield raw(owner, repo, ref, f["path"])
    else:
        # Wheel filenames carry build suffixes and Rockchip has changed the
        # manylinux tag spelling between releases, so list the directory and
        # pick by the two things that are actually load-bearing: cp310 and
        # aarch64.
        parent = path.rsplit("/", 1)[0]
        for f in api_listdir(owner, repo, ref, parent):
            if not isinstance(f, dict) or not f.get("path"):
                continue
            n = f.get("name", "")
            if n.endswith(".whl") and "cp310" in n and "aarch64" in n:
                yield raw(owner, repo, ref, f["path"])


def safe_candidates():
    """candidates() with a fence around it. Anything that escapes the generator
    becomes a stderr line and an early end of the list, NOT a traceback that
    replaces the report at the bottom of this file."""
    try:
        for u in candidates():
            yield u
    except Exception as e:                                  # noqa: BLE001
        sys.stderr.write("    (candidate enumeration stopped: %s: %s)\n"
                         % (type(e).__name__, e))


seen = set()
for url in safe_candidates():
    if url in seen:
        continue
    if len(seen) >= MAX_CANDIDATES:
        sys.stderr.write("    (stopping after %d candidates - the cap)\n"
                         % MAX_CANDIDATES)
        break
    if time.monotonic() - STARTED > DEADLINE_S:
        sys.stderr.write("    (stopping after %d s - the budget)\n" % DEADLINE_S)
        break
    seen.add(url)
    if check(url):
        print(url)
        for u, note in tried:
            sys.stderr.write("    %-8s %s\n        %s\n" % ("tried", u, note))
        sys.exit(0)

sys.stderr.write("\n    NO WORKING URL for the %s. Tried:\n" % kind)
for u, note in tried:
    sys.stderr.write("      %s\n        -> %s\n" % (u, note))
p = parse(configured)
if p:
    owner, repo, ref, path = p
    sys.stderr.write(
        "\n    Find the right one by hand and put it in config/fpms-os.conf\n"
        "    (agent A's file - do NOT patch it from stage 25):\n\n"
        "      curl -s '%s/repos/%s/%s/git/ref/tags/%s' | head    # does the tag exist?\n"
        "      curl -s '%s/repos/%s/%s/contents/%s?ref=%s' | grep '\"path\"'\n\n"
        % (API, owner, repo, ref, API, owner, repo, path.rsplit("/", 1)[0], ref))
sys.exit(1)
PY

resolve() {   # resolve <so|wheel> <configured-url>
    python3 "$TMP/resolve_url.py" "$1" "$2"
}

# ---------------------------------------------------------------------------
# librknnrt.so
# ---------------------------------------------------------------------------
#
# DOWNLOAD TO A TEMPORARY FILE, VERIFY, THEN INSTALL. Never write $SO_DEST
# directly, which is what stage 20 does today:
#
#     wget -q -O /usr/lib/librknnrt.so "${LIBRKNNRT_URL}"
#
# On a 404 (a moved tag, a renamed path upstream) wget exits non-zero but has
# ALREADY CREATED the destination, empty or holding an HTML error page. That
# leaves a zero-byte /usr/lib/librknnrt.so in the image, which is strictly
# worse than no file at all: ldconfig indexes it, dlopen finds it, and the
# failure surfaces as an unintelligible loader error at init_runtime() instead
# of an honest "file not found".
#
# Clean up such a corpse if stage 20 left one.
if [ -e "$SO_DEST" ] && [ ! -s "$SO_DEST" ]; then
    echo "    removing zero-byte $SO_DEST left by an earlier stage" >&2
    rm -f "$SO_DEST"
fi

echo "    resolving librknnrt.so URL"
if ! SO_URL_USED="$(resolve so "$SO_URL_CONF")"; then
    npu_fail "no URL at tag $TAG serves an aarch64 librknnrt.so" \
             "see the candidate list above; fix LIBRKNNRT_URL in config/fpms-os.conf"
fi
if [ "$SO_URL_USED" != "$SO_URL_CONF" ]; then
    cat >&2 <<EOF
    !! LIBRKNNRT_URL in config/fpms-os.conf DOES NOT RESOLVE and was worked
    !! around at build time. Fix the config - this stage should not have to
    !! guess:
    !!   configured: $SO_URL_CONF
    !!   used:       $SO_URL_USED
EOF
fi

echo "    fetching librknnrt.so"
# --tries/--timeout so a flaky mirror fails in a minute rather than hanging the
# build. GitHub's /raw/ path redirects to raw.githubusercontent.com; wget
# follows that by default, and the resolver above has already followed it once
# and validated the bytes on the other side.
#
# -nv, NOT -q. `-q` is TOTALLY silent: on a 404, a TLS failure or a DNS failure
# it writes nothing at all, so npu_fail's "could not download librknnrt.so"
# below would be the entire diagnosis and the operator would have to re-run the
# fetch by hand to learn which of those three it was. -nv suppresses the
# progress bar (the only thing that made -q attractive - it is noise in a build
# log) and keeps the error line and the resolved redirect target, which is
# exactly the pair worth having when a Rockchip path moves.
if ! wget -nv --tries=3 --timeout=30 -O "$TMP/librknnrt.so" "$SO_URL_USED"; then
    npu_fail "could not download librknnrt.so from $SO_URL_USED" \
             "sudo wget -O $SO_DEST $SO_URL_USED && sudo ldconfig"
fi

# Is it actually an aarch64 shared object, or is it an HTML error page with a
# .so name, or a transfer that stopped halfway? Checked with python rather than
# file(1)/readelf, neither of which is guaranteed present in the base image.
#   bytes 0-3   \x7fELF
#   byte  4     EI_CLASS, 2 = ELF64
#   bytes 18-19 e_machine, 183 (0xB7) = EM_AARCH64
#
# The size floor is the third check and it is not decorative. The resolver only
# ever saw the first 64 bytes; a proxy or a half-open connection can deliver a
# valid ELF header followed by nothing, and a 3 kB "shared object" would sail
# through a magic-number test, get indexed by ldconfig, and fail at dlopen with
# a loader error. MEASURED 2026-08-10: the real v2.3.0 artifact is 7,259,064
# bytes. The floor is deliberately an order of magnitude below that rather than
# an equality test, because a future tag may legitimately differ in size and
# this stage must not have to be edited every time the pin moves.
if ! python3 - "$TMP/librknnrt.so" <<'PY'
import os
import sys
p = sys.argv[1]
sz = os.path.getsize(p)
with open(p, "rb") as fh:
    b = fh.read(20)
why = None
if len(b) < 20 or b[:4] != b"\x7fELF":
    why = "not an ELF file at all (a 404 page, or a truncated transfer)"
elif b[4] != 2:
    why = "ELF but not ELF64 (EI_CLASS=%d)" % b[4]
elif int.from_bytes(b[18:20], "little") != 183:
    why = "ELF64 but e_machine=%d, not 183/EM_AARCH64" % int.from_bytes(b[18:20], "little")
elif sz < 1000000:
    why = "a valid aarch64 ELF header on only %d bytes - truncated download" % sz
if why:
    sys.stderr.write("    %s: %s (%d bytes, first bytes %r)\n" % (p, why, sz, b[:16]))
    sys.exit(1)
PY
then
    npu_fail "the downloaded librknnrt.so is not a whole aarch64 ELF (a 404 page?)" \
             "check $SO_URL_USED resolves - upstream may have moved the path at tag $TAG"
fi

install -m 0644 -o root -g root "$TMP/librknnrt.so" "$SO_DEST"

# ldconfig, so the loader can find it by SONAME rather than by luck. Required
# after every replacement of the file, and harmless to repeat.
# VERIFIED 2026-08-10: the v2.3.0 artifact does carry DT_SONAME=librknnrt.so,
# so it is a file ldconfig can legitimately index and the warning below is a
# real signal rather than a permanent false alarm.
#
# `|| echo`, not a bare `ldconfig`. A bare one is a `set -e` trip-wire, and it
# is the wrong policy in this exact spot: eight lines below, this file argues
# that librknnrt.so being absent from the ld.so cache is NOT fatal, because the
# runtime may dlopen it by absolute path - and the verify step at the bottom of
# this stage does precisely that, `ctypes.CDLL("/usr/lib/librknnrt.so")`. So a
# cache the loader could not rebuild must not be a harder failure than a cache
# that simply lacks the entry. ldconfig under qemu-user is also the kind of
# thing that emits a non-zero for an unrelated malformed entry inherited from
# the vendor rootfs, which would have killed the stage one line after the .so
# installed correctly.
ldconfig || echo "    WARNING: ldconfig exited non-zero; the ld.so cache may be stale" >&2
# NOT `ldconfig -p | grep -q`. Under `set -o pipefail`, grep -q exits the
# instant it matches, ldconfig takes SIGPIPE, and the pipeline's status becomes
# 141 - so the SUCCESS case would have printed the failure warning. Capture
# first, match second.
#
# SWEPT 2026-08-11, because a rule written down in one place is not a rule.
#
#   RE-SWEPT the same day, because the sweep was WRONG. This comment used to
#   claim "THERE ARE NOW NO PIPELINES AT ALL IN THIS FILE" and listed
#   SO_SHA="$(sha256sum "$SO_DEST" | awk '{print $1}')" among the three it had
#   removed. That line was still there, ~70 lines below this one, exactly as
#   written. A sweep that certifies itself is worth nothing; verify it:
#
#     grep -nE '\|[^|]' scripts/25-npu-runtime.sh
#
#   (the surviving hits are comment text and JSON string literals - the
#   `strings ... | grep` recipes this file tells an operator to run on the
#   board, which are documentation, not pipelines this shell executes.)
#
# The four that existed were
#
#     SO_VER="$(printf ... | sed -n 1p)"
#     SO_VER_FULL="$(printf ... | sed -n 2p)"
#     SO_SHA="$(sha256sum "$SO_DEST" | awk '{print $1}')"
#     WHEEL_VER/NUMPY_VER/... via five separate python3 -c calls
#
# and none of them was the SIGPIPE variant - `sed -n Np` without `q` and
# `awk '{print $1}'` without `exit` both drain their input, so neither could
# raise 141. They are gone anyway, replaced by shell builtins and by parameter
# expansion, on the grounds that this build has now died twice to a pipeline
# inside a plain assignment and the cheapest way to never make that mistake a
# fourth time is to leave no pipeline in an assignment for a later edit to make
# early-exiting. (It also removes forks per run, which under qemu-user is an
# emulator start-up each.)
LDCACHE="$(ldconfig -p 2>/dev/null || true)"
case "$LDCACHE" in
    *librknnrt.so*) : ;;
    *)  # Not fatal on its own - the runtime may dlopen it by absolute path -
        # but if the cache does not have it, something about the install is off
        # and it is worth saying so out loud rather than discovering it at
        # init_runtime().
        echo "    WARNING: librknnrt.so is not in the ldconfig cache after ldconfig" >&2
        echo "    Check /etc/ld.so.conf.d/ includes /usr/lib" >&2 ;;
esac

# The version string is EMBEDDED IN THE BINARY. The canonical way to read it is
#     strings librknnrt.so | grep -i 'librknnrt version'
# which yields something of the form
#     librknnrt version: 2.3.0 (<commit>@<build date>)
# (VERIFIED 2026-08-10 on the pinned artifact: it yields exactly
#     librknnrt version: 2.3.0 (c949ad889d@2024-11-07T11:35:33)
# so the two regexes below are matched against a real string, not a guess.)
# strings(1) comes from binutils and is not guaranteed in the base image, so
# the same search is done here over the raw bytes. This is the ONLY place the
# runtime version can be obtained without hardware.
#
# ONE python invocation reading the file ONCE, not two reading it twice. Under
# qemu-user each interpreter start-up is an emulator start-up and this is a
# 7 MB read plus a regex scan; there is no reason to pay for it twice.
#
# The file is opened "rb" and the decodes below are "backslashreplace", NOT
# "replace": "replace" yields U+FFFD, and printing U+FFFD on a stdout that came
# up as strict ASCII (`env -i ... LC_ALL=C`) raises UnicodeEncodeError - which
# would turn "this .so has an odd version string" into a dead stage.
# backslashreplace produces \xNN, which is ASCII by construction.
#
# And the exit status IS checked. This was a bare `SO_VERS="$(python3 ...)"`,
# which is the assignment-adopts-the-status shape that has killed this build
# twice; a python that died here would have taken the stage with it and the
# only clue would have been a traceback with no line of our own around it.
# Failing to READ a version is not a reason to fail the build, so it warns.
if ! SO_VERS="$(python3 - "$SO_DEST" <<'PY'
import re
import sys
with open(sys.argv[1], "rb") as fh:
    data = fh.read()
short = re.search(rb"librknnrt version:?\s*([0-9][0-9A-Za-z._-]*)", data, re.I)
full = re.search(rb"librknnrt version[^\x00]{0,160}", data, re.I)
print(short.group(1).decode("ascii", "backslashreplace") if short else "")
print(full.group(0).decode("ascii", "backslashreplace").strip() if full else "")
PY
)"; then
    echo "    WARNING: the version scan of $SO_DEST failed (see the error above);" >&2
    echo "    librknnrt.version will be null in $JSON" >&2
    SO_VERS=""
fi
# Split with the shell, no `| sed -n Np`. `read` returns non-zero at EOF, so
# each one is paired with a `|| assignment` - the read is then not the final
# command of a `||` list and `set -e` leaves it alone, and the group's own exit
# status is the assignment's 0. Verified with `set -euo pipefail` before it went
# in; the two-empty-lines case (no version string found) is the one that ends
# at EOF on the second read.
SO_VER=""
SO_VER_FULL=""
{   IFS= read -r SO_VER      || SO_VER=""
    IFS= read -r SO_VER_FULL || SO_VER_FULL=""
} <<<"$SO_VERS"
if [ -z "$SO_VER" ]; then
    # Not fatal: the .so is a valid aarch64 ELF and may simply have a version
    # string this pattern does not match. But it means the record is
    # incomplete, so say so rather than writing a blank field nobody notices.
    echo "    WARNING: no 'librknnrt version' string found in $SO_DEST" >&2
    echo "    Confirm on hardware: strings $SO_DEST | grep -i 'librknnrt version'" >&2
    echo "    and put the answer in npu/versions/README.md's table." >&2
else
    echo "    librknnrt version: $SO_VER"
fi

# THE PIPELINE THE SWEEP ABOVE MISSED. It was
#
#     SO_SHA="$(sha256sum "$SO_DEST" | awk '{print $1}')"
#
# a pipeline inside a plain assignment, which is the exact shape that has taken
# this build down twice. `awk '{print $1}'` drains its input so it could not
# raise the SIGPIPE/141 variant - but sha256sum returning non-zero (the file
# vanished, an I/O error on the loop device) would still be adopted by the
# assignment and kill the stage silently, one line after a successful install
# and with no output of its own. Split with parameter expansion instead:
# "<64 hex> *<path>" -> the field before the first space. No fork, no pipe, no
# status to adopt.
#
# Neither of these is fatal: a build record with a null sha or size is worse
# than a complete one but far better than a dead stage, and the .so has already
# been proved to be a whole aarch64 ELF by this point.
if ! SO_SHA_RAW="$(sha256sum "$SO_DEST")"; then
    echo "    WARNING: sha256sum failed on $SO_DEST; librknnrt.sha256 will be null" >&2
    SO_SHA_RAW=""
fi
SO_SHA="${SO_SHA_RAW%% *}"
if ! SO_SIZE="$(stat -c %s "$SO_DEST")"; then
    echo "    WARNING: stat failed on $SO_DEST; librknnrt.size_bytes will be null" >&2
    SO_SIZE=""
fi
echo "    librknnrt.so  ${SO_SIZE:-?} bytes  sha256 ${SO_SHA:-?}"

# Exported NOW, not at the end. If the wheel half of this stage fails under
# ALLOW_NPU_MISSING, npu_fail writes the record immediately - and an ABSENT
# record that still names the .so that DID install is a more useful artifact
# than one with three nulls in it.
export SO_VER SO_VER_FULL SO_SHA SO_SIZE SO_URL_USED

# ---------------------------------------------------------------------------
# rknn-toolkit-lite2 (the wheel)
# ---------------------------------------------------------------------------
#
# NOT on PyPI - it is fetched from Rockchip's repo at the pinned tag, which is
# why it comes in as a URL.
#
# PIP AND --break-system-packages
# -------------------------------
# This stage used to pass --break-system-packages unconditionally, "required on
# this base", copied from stage 20. It is not required on this base; it is not
# even ACCEPTED on this base.
#
#   MEASURED 2026-08-10, from pip's own changelog: --break-system-packages was
#   added in pip 23.0.1. Ubuntu 22.04 ships python3-pip 22.0.2, and nothing in
#   this build upgrades it (grep the scripts: only npu/convert/, which runs on
#   an x86 workstation, touches pip itself). pip 22.0.2 answers an unknown
#   option with "no such option: --break-system-packages" and exit 2.
#
#   It is also unnecessary. --break-system-packages exists to override PEP 668,
#   which is only enforced when the interpreter ships an EXTERNALLY-MANAGED
#   marker. Jammy has no such marker; Debian/Ubuntu added it from bookworm and
#   lunar onward.
#
# So the flag is DETECTED, not assumed - because the day this image is rebased
# onto 24.04 the marker WILL be there and the flag WILL be required, and a
# stage that hardcodes either answer is wrong on one of the two bases.
#
# THE PROBE IS NOW BYTE-FOR-BYTE STAGE 20'S, and the two differences it used to
# have were both wrong in this direction:
#
#   - it captured `2>&1`, merging pip's STDERR into the text it then searches
#     for the literal "--break-system-packages". Anything pip writes to stderr
#     is thereby evidence: a deprecation notice, a "new release of pip is
#     available" banner, or - the case that actually matters - a pip that
#     rejects the flag while NAMING it ("no such option:
#     --break-system-packages"). A detector that reads its own failure message
#     as a positive is worse than no detector, because the flag then goes on
#     the pip3 command line below and jammy's pip 22.0.2 exits 2. Stage 20 uses
#     2>/dev/null for exactly this reason; so does this now.
#   - it had no arm for EMPTY output. Empty help is not evidence the flag is
#     absent, it is evidence pip is broken, and silently taking the "absent"
#     branch on it is a guess that happens to be right on jammy and wrong on
#     any PEP 668 base - where the install below then dies with
#     "externally-managed-environment", which reads like a policy decision
#     rather than a failed detection. Stage 20 says so out loud; so does this.
#
# The pip version is read the same way stage 20 reads it, by parameter
# expansion off `pip3 --version` - "pip 22.0.2 from /usr/... (python 3.10)" ->
# 22.0.2 - so the message names the pip that was actually probed rather than
# leaving the reader to infer it. No pipeline, nothing to adopt a status from.
PIP_VER_RAW="$(pip3 --version 2>/dev/null || true)"
PIP_VER="${PIP_VER_RAW#pip }"
PIP_VER="${PIP_VER%% *}"

PIP_HELP="$(pip3 install --help 2>/dev/null || true)"
PIP_FLAGS=()
case "$PIP_HELP" in
    "")
        echo "    WARNING: 'pip3 install --help' produced no output (pip ${PIP_VER:-?})." >&2
        echo "    Could not detect --break-system-packages; continuing without it." >&2
        echo "    If the wheel install below fails with 'externally-managed-environment'," >&2
        echo "    this detection is why, not the URL and not the numpy pin." >&2 ;;
    *--break-system-packages*)
        PIP_FLAGS+=(--break-system-packages)
        echo "    pip ${PIP_VER:-?} has --break-system-packages (PEP 668 base); using it" ;;
    *)
        echo "    pip ${PIP_VER:-?} has no --break-system-packages (pip < 23.0.1); not needed on jammy" ;;
esac

# NUMPY CONSTRAINT. rknn-toolkit-lite2's METADATA declares, verbatim:
#     Requires-Dist: numpy
#     Requires-Dist: psutil
#     Requires-Dist: ruamel.yaml
# An UNCONSTRAINED `numpy` on cp310/aarch64 resolves to 2.x, on top of ROS
# Humble's 1.x-ABI C extensions, and breaks tf_transformations three stages
# later with "np.maximum_sctype was removed".
#
# Stage 20 writes /etc/pip.conf -> /etc/fpms/pip-constraints.txt, and on a full
# build that file is already in place. But `build.sh --stage 25` rebuilds the
# rootfs from the pristine base image and runs ONLY this stage, so stage 20 has
# not run, /etc/pip.conf does not exist, and the constraint is not applied -
# which would make the single-stage re-run this file keeps recommending fail at
# the numpy gate below, for a reason that has nothing to do with the NPU.
#
# So: use stage 20's file when it exists, and otherwise pass an equivalent one
# of our own explicitly. Do NOT create stage 20's file - two stages writing one
# path is how the pin ends up saying different things depending on run order.
CONSTRAINTS=/etc/fpms/pip-constraints.txt
if [ ! -f "$CONSTRAINTS" ]; then
    CONSTRAINTS="$TMP/pip-constraints.txt"
    printf '%s\n' "${PIP_NUMPY:-numpy<2}" > "$CONSTRAINTS"
    echo "    /etc/fpms/pip-constraints.txt absent (stage 20 has not run);" >&2
    echo "    constraining this install with a temporary copy: ${PIP_NUMPY:-numpy<2}" >&2
fi

echo "    resolving rknn-toolkit-lite2 wheel URL"
if ! WHEEL_URL_USED="$(resolve wheel "$WHEEL_URL_CONF")"; then
    npu_fail "no cp310/aarch64 rknn-toolkit-lite2 wheel found at tag $TAG" \
             "see the candidate list above; fix RKNN_LITE_WHEEL_URL in config/fpms-os.conf"
fi
if [ "$WHEEL_URL_USED" != "$WHEEL_URL_CONF" ]; then
    cat >&2 <<EOF
    !! RKNN_LITE_WHEEL_URL in config/fpms-os.conf DOES NOT RESOLVE and was
    !! worked around at build time. Fix the config:
    !!   configured: $WHEEL_URL_CONF
    !!   used:       $WHEEL_URL_USED
EOF
fi
export WHEEL_URL_USED

# Idempotent: pip reinstalls from a direct URL every run, which is what we
# want - it makes the installed wheel a function of the config file only.
echo "    installing rknn-toolkit-lite2"
if ! pip3 install --no-cache-dir "${PIP_FLAGS[@]+"${PIP_FLAGS[@]}"}" \
        -c "$CONSTRAINTS" "$WHEEL_URL_USED"; then
    npu_fail "pip could not install the rknn-toolkit-lite2 wheel" \
             "pip3 install -c $CONSTRAINTS $WHEEL_URL_USED"
fi

# ---------------------------------------------------------------------------
# VERIFY. Do not ship an image whose NPU imports do not resolve.
# ---------------------------------------------------------------------------
#
# THE OLD VERSION OF THIS BLOCK CHECKED THE WRONG THING, and said so in a
# comment that read convincingly:
#
#     # The constructor does not touch the device - it wires up the runtime
#     # bindings. If librknnrt.so is missing, corrupt, or the wrong arch, this
#     # is where it shows, which is exactly the check we want at build time.
#     RKNNLite()
#
# That is false, and it is false in the direction that matters. Read
# rknnlite/api/rknn_lite.py out of the pinned wheel (2.3.0, checked
# 2026-08-10): RKNNLite.__init__ computes a path, checks whether the host is
# Windows, sets up a logger, tries `pkg_resources.get_distribution` inside a
# bare except, and assigns six attributes. It does not dlopen anything. The
# runtime is loaded by rknn_runtime, from init_runtime() - which cannot run
# here. So `RKNNLite()` succeeds perfectly on an image whose
# /usr/lib/librknnrt.so is a zero-byte file, an HTML page, or absent: exactly
# the three states the rest of this stage exists to prevent.
#
# The check that DOES prove link 2 is a plain dlopen, which needs no NPU
# device, no model and no driver - only that the file is a loadable aarch64
# object whose own NEEDED list is satisfiable. (VERIFIED 2026-08-10: the
# pinned artifact needs libstdc++.so.6, libgcc_s.so.1, libpthread.so.0,
# libdl.so.2, libm.so.6, libc.so.6, and exports rknn_init/rknn_run/rknn_query/
# rknn_outputs_get/rknn_destroy.) Resolving rknn_init on top of the dlopen
# turns "some .so loaded" into "the RKNN C API is present in it".
#
# What is still NOT proven here: load_rknn() and init_runtime() CANNOT be
# exercised - there is no NPU device in the chroot and no model in the image
# (the .rknn lives only on the old Pi - see stage 30). That proof is agent 7's
# fpms-npu-selftest, on hardware, and nothing here should be read as a
# substitute for it.
VERIFY_OUT="$TMP/verify.json"
if ! python3 - "$VERIFY_OUT" "$SO_DEST" <<'PY'
import json
import sys

res = {"dlopen_ok": False, "rknn_init_symbol": False,
       "import_ok": False, "rknnlite_constructible": False,
       "wheel_version": None, "numpy_version": None, "error": None}
out, so_path = sys.argv[1], sys.argv[2]
try:
    # --- link 2: the runtime itself ---------------------------------------
    import ctypes
    lib = ctypes.CDLL(so_path)
    res["dlopen_ok"] = True
    getattr(lib, "rknn_init")          # AttributeError if it is not the RKNN API
    res["rknn_init_symbol"] = True

    # --- link 3: the wheel -------------------------------------------------
    import importlib.metadata as md
    try:
        res["wheel_version"] = md.version("rknn-toolkit-lite2")
    except Exception:
        # Rockchip has shipped this under both spellings at different times.
        try:
            res["wheel_version"] = md.version("rknn_toolkit_lite2")
        except Exception:
            pass
    from rknnlite.api import RKNNLite
    res["import_ok"] = True
    # Cheap, and worth keeping now that it is honestly labelled: it proves the
    # module's own compiled extensions (rknn_runtime, rknn_log, ... all
    # cpython-310-aarch64) import under this interpreter. It proves NOTHING
    # about librknnrt.so - that is what dlopen_ok above is for.
    RKNNLite()
    res["rknnlite_constructible"] = True

    import numpy
    res["numpy_version"] = numpy.__version__
except Exception as exc:                                    # noqa: BLE001
    res["error"] = "%s: %s" % (type(exc).__name__, exc)

with open(out, "w") as fh:
    json.dump(res, fh)

# numpy is checked by the shell, not here, so that its failure message can name
# /etc/pip.conf. Everything else is a hard gate.
sys.exit(0 if (res["dlopen_ok"] and res["rknn_init_symbol"]
               and res["import_ok"] and res["rknnlite_constructible"]) else 1)
PY
then
    echo "    detail: $(cat "$VERIFY_OUT" 2>/dev/null || echo '(no detail)')" >&2
    npu_fail "the NPU runtime or wheel does not load after install" \
             "python3 -c \"import ctypes; ctypes.CDLL('$SO_DEST').rknn_init; from rknnlite.api import RKNNLite; RKNNLite()\""
fi

# ONE python invocation for all five fields, read out with the same
# capture-then-split idiom used for the version string above.
#
# THIS REPLACES jget()/jflag(), which were five separate
#
#     WHEEL_VER="$(jget wheel_version)"
#
# lines - five plain assignments, each adopting the exit status of its own
# `python3 -c`. A malformed or truncated $VERIFY_OUT (the verify step was
# SIGKILLed by the OOM killer after writing half its json, say) makes
# json.load raise, and the FIRST of those five assignments then takes the whole
# stage out with a traceback and no line of ours around it - after the .so and
# the wheel both installed correctly, and before the numpy gate that is the
# only remaining reason this stage exists. It is also five interpreter
# start-ups, which under qemu-user is five emulator start-ups to read five
# fields out of one small file this stage wrote itself moments ago.
#
# Failing to PARSE the verify record is not the same as failing verification -
# verification already passed, above, on that python's own exit status. So this
# warns and leaves the record's fields null rather than failing the build. The
# one field that must not be silently null is numpy, and it is not: an empty
# NUMPY_VER falls into the FATAL arm of the case below, deliberately.
if ! VERIFY_FIELDS="$(python3 - "$VERIFY_OUT" <<'PY'
import json
import sys
try:
    with open(sys.argv[1]) as fh:
        d = json.load(fh)
except Exception as exc:                                    # noqa: BLE001
    sys.stderr.write("    could not parse the verify record: %s: %s\n"
                     % (type(exc).__name__, exc))
    sys.exit(1)
# Order is load-bearing: the reads below are positional.
print(d.get("wheel_version") or "")
print(d.get("numpy_version") or "")
print(1 if d.get("dlopen_ok") else 0)
print(1 if d.get("import_ok") else 0)
print(1 if d.get("rknnlite_constructible") else 0)
PY
)"; then
    echo "    WARNING: could not read $VERIFY_OUT back (see the error above);" >&2
    echo "    the measured flags in $JSON will be null and the numpy gate below" >&2
    echo "    will fail the build rather than guess." >&2
    VERIFY_FIELDS=""
fi
WHEEL_VER=""
NUMPY_VER=""
DLOPEN_OK=""
IMPORT_OK=""
CONSTRUCT_OK=""
# `read` returns non-zero at EOF, so each is paired with a `|| assignment`: the
# read is then not the final command of its `||` list, `set -e` leaves it
# alone, and the group's own status is the assignment's 0. Proved under
# `set -euo pipefail` before it went in, including the all-fields-empty case
# (VERIFY_FIELDS="") that ends at EOF on the first read.
{   IFS= read -r WHEEL_VER    || WHEEL_VER=""
    IFS= read -r NUMPY_VER    || NUMPY_VER=""
    IFS= read -r DLOPEN_OK    || DLOPEN_OK=""
    IFS= read -r IMPORT_OK    || IMPORT_OK=""
    IFS= read -r CONSTRUCT_OK || CONSTRUCT_OK=""
} <<<"$VERIFY_FIELDS"
echo "    rknn-toolkit-lite2 ${WHEEL_VER:-(version unknown)}  numpy ${NUMPY_VER:-?}"
echo "    librknnrt.so dlopen ok, rknn_init present"

# numpy must still be 1.x AFTER the wheel's dependency resolve. A 2.x here does
# not break the NPU - it breaks tf_transformations, silently, three stages
# later, and this is the last stage that could have caused it.
#
# THIS FAILS THE BUILD, and deliberately does not go through npu_fail:
# ALLOW_NPU_MISSING permits an image with no NPU, not an image with a broken
# ROS. An empty NUMPY_VER falls into the same arm on purpose - "numpy did not
# import at all" is not a passing state either, and the verify record is
# printed so the actual exception is visible rather than inferred.
case "$NUMPY_VER" in
    1.*) : ;;
    *)   echo "FATAL: numpy is '${NUMPY_VER:-missing}' after installing rknn-toolkit-lite2." >&2
         echo "detail: $(cat "$VERIFY_OUT" 2>/dev/null || echo '(no detail)')" >&2
         echo "ROS Humble's C extensions are built against the NumPy 1.x ABI; a 2.x breaks" >&2
         echo "tf_transformations with 'np.maximum_sctype was removed'. The wheel declares a" >&2
         echo "bare 'Requires-Dist: numpy', so it WILL pull 2.x unless constrained. Check that" >&2
         echo "/etc/pip.conf points at /etc/fpms/pip-constraints.txt (stage 20), and that" >&2
         echo "$CONSTRAINTS says numpy<2." >&2
         exit 1 ;;
esac

# ---------------------------------------------------------------------------
# Record what was installed. See write_json() near the top of this file.
# ---------------------------------------------------------------------------
#
# Exported so the python heredoc inside write_json() can see them; build.sh's
# in_chroot() runs `env -i`, so nothing is inherited that was not put there
# deliberately, and nothing reaches a child that was not exported here.
export WHEEL_VER NUMPY_VER DLOPEN_OK IMPORT_OK CONSTRUCT_OK
write_json INSTALLED

# ---------------------------------------------------------------------------
# What is STILL missing after this stage - do not let the OK line imply more
# than it means.
# ---------------------------------------------------------------------------
cat <<'EOF'

    ------------------------------------------------------------------
     WHAT STAGE 25 DID NOT AND COULD NOT PROVE

       - that the kernel rknpu driver matches this runtime. Its version
         is not readable in a chroot. Recorded as expected=null;
         fpms-npu-selftest checks it on the running board.
       - that init_runtime() succeeds. dlopen proves the runtime LOADS;
         it says nothing about whether it can talk to the driver, which
         is the failure this whole layer is about.
       - that `ubuntu` can open the NPU device node. The node name on
         the Orange Pi 5B BSP is unconfirmed - see
         overlay/etc/udev/rules.d/99-fpms-npu.rules.
       - that any model loads. yolo26n-rk3588.rknn is not in the
         repository (stage 30 says so too); a .rknn also carries its own
         container version, which is a FOURTH link in this chain.
       - that inference is correct, or fast, or thermally sustainable.

     First thing on the board:
       sudo cat /sys/kernel/debug/rknpu/version
       cat /etc/fpms/npu-versions.json
       fpms-npu-selftest
     and read npu/versions/README.md before changing any pin.
    ------------------------------------------------------------------

EOF

echo "--- 25-npu-runtime OK"
