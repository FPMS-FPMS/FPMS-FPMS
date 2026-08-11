#!/usr/bin/env bash
# Stage 20 - Python dependencies.
#
# THIS IS THE STAGE MOST LIKELY TO PRODUCE A DEAD IMAGE, and the failures are
# all silent. Read the comments before changing a pin.
#
# WHAT THIS STAGE OWNS
# ====================
#   python3-opencv / python3-serial / python3-numpy / python3-yaml from apt
#   the numpy 1.x policy:  /etc/apt/preferences.d/fpms-numpy
#                          /etc/pip.conf -> /etc/fpms/pip-constraints.txt
#   paho-mqtt >= 2.0 and websocket-client from pip
#   the import verification that FAILS THE BUILD
#
# WHAT IT NO LONGER OWNS: the NPU. See the "--- the NPU" note below.
set -euo pipefail
echo "--- 20-python-deps"

export DEBIAN_FRONTEND=noninteractive

# build.sh's in_chroot() runs `env -i` with an explicit variable list, so a pin
# that is missing here means config/fpms-os.conf and build.sh have drifted
# apart. Say which one, rather than dying with "unbound variable" further down.
: "${PIP_PAHO:?PIP_PAHO not set - build.sh must export it (config/fpms-os.conf)}"
: "${PIP_NUMPY:?PIP_NUMPY not set - build.sh must export it (config/fpms-os.conf)}"
: "${PIP_WEBSOCKET:?PIP_WEBSOCKET not set - build.sh must export it (config/fpms-os.conf)}"
: "${FPMS_USER:?FPMS_USER not set - build.sh must export it}"
: "${FPMS_HOME:?FPMS_HOME not set - build.sh must export it}"

command -v pip3 >/dev/null 2>&1 \
    || { echo "FATAL: pip3 is not installed. Stage 00 installs python3-pip." >&2; exit 1; }

# Idempotency: this stage must survive `build.sh --stage 20` against a chroot
# whose apt lists were fetched days ago, where every apt-get install would
# otherwise 404 on a moved pool file.
apt-get update -qq

# --- numpy must stay on 1.x -------------------------------------------------
#
# ROS Humble's C extensions are built against the NumPy 1.x ABI. A 2.x breaks
# tf_transformations with "np.maximum_sctype was removed in the NumPy 2.0
# release" - and it broke it on the old Pi in the most confusing possible way,
# because the offending copy was a USER-LOCAL install in
# /home/ubuntu/.local/lib/python3.10/site-packages that shadowed the system
# one for every service.
#
# Pin it at the apt level so an archive that later ships a 2.x cannot walk over
# it, and at the pip level (below) so a stray `pip install -U numpy` cannot
# either. The apt pin does nothing about pip and the pip constraint does
# nothing about apt; both are needed.
#
# THE EPOCH, which is why this is computed rather than written out. jammy's
# python3-numpy is 1:1.21.5-1ubuntu22.04.1 - Debian's numpy has carried epoch 1
# for years, and python3-opencv even depends on `python3-numpy (>= 1:1.21.5)`.
# An apt version pin is matched against the version string EXACTLY as apt
# prints it, epoch included, and a trailing `*` is the only wildcard apt's
# version matcher understands. So the obvious
#
#     Pin: version 1.*
#
# matches NOTHING here: the next character after "1" is ":", not ".". apt does
# not complain - it just applies the pin to no version at all, and every later
# apt operation runs on default priorities. A no-op pin is worse than no pin,
# because the file reads like protection. Build the pattern from the epoch apt
# actually reports, then check that the priority landed.
NUMPY_APT_VER="$(apt-cache policy python3-numpy 2>/dev/null \
                 | awk -F': +' '/^[[:space:]]*Candidate:/{print $2; exit}' || true)"
case "${NUMPY_APT_VER:-}" in
    *:1.*) NUMPY_PIN="${NUMPY_APT_VER%%:*}:1.*" ;;   # keep the epoch: "1:1.*"
    1.*)   NUMPY_PIN="1.*" ;;
    *)     NUMPY_PIN="1.*"
           echo "    WARNING: apt reports python3-numpy candidate '${NUMPY_APT_VER:-none}';" >&2
           echo "    falling back to 'Pin: version 1.*', which may match nothing." >&2
           echo "    The fatal numpy-1.x check at the end of this stage still applies." >&2 ;;
esac

# Pin-Priority 1001 and not 990: above 1000 is what permits a DOWNGRADE, which
# is the whole point - if anything has already pulled a 2.x in, apt must be
# willing to go backwards rather than just decline to go forwards.
cat > /etc/apt/preferences.d/fpms-numpy <<EOF
Package: python3-numpy
Pin: version ${NUMPY_PIN}
Pin-Priority: 1001
EOF
# No extension on the filename, deliberately: apt IGNORES files in
# preferences.d whose name has an extension other than .pref, silently.

# A malformed preferences file does not break this stage - it breaks EVERY apt
# operation from here to stage 90, with an error nobody connects back to this
# file. Read the policy back; the read itself is the syntax check.
if ! APT_POLICY="$(apt-cache policy python3-numpy 2>&1)"; then
    echo "FATAL: apt cannot read its preferences after writing" >&2
    echo "       /etc/apt/preferences.d/fpms-numpy" >&2
    printf '%s\n' "$APT_POLICY" >&2
    exit 1
fi
case "$APT_POLICY" in
    *1001*) echo "    numpy apt pin active (Pin: version ${NUMPY_PIN})" ;;
    *)      echo "    WARNING: wrote 'Pin: version ${NUMPY_PIN}' but apt does not report" >&2
            echo "    priority 1001 for python3-numpy - the pin matches no version." >&2
            echo "    Check: apt-cache policy python3-numpy" >&2 ;;
esac

# --- from apt ---------------------------------------------------------------
#
# python3-opencv from apt, NOT opencv-python from pip. The pip wheel links its
# own FFmpeg/GStreamer and ABI-clashes with ROS's cv_bridge, which is a classic
# and extremely confusing breakage. The apt build also has V4L2 enabled, which
# the agent needs for its MJPG capture (CAP_PROP_FOURCC).
#
# On jammy arm64 this is python3-opencv 4.5.4+dfsg - it depends on
# python3-numpy (>= 1:1.21.5) and is BUILT against that 1.x, so it pulls in no
# competing numpy. Installed after the pin above, so the pin governs it.
#
# python3-serial from apt is pyserial. See the impostor note below.
apt-get install -y -qq --no-install-recommends \
    python3-opencv python3-serial python3-numpy python3-yaml

# --- the pip level of the same policy ---------------------------------------
#
# /etc/pip.conf is pip's system-wide config, and [global] applies to every
# command that has the option (pip ignores config keys its command does not
# know, which is what makes this safe across pip versions).
#
# This constraint is not local to this stage. It is the mechanism stage 25
# relies on when it installs the rknn-toolkit-lite2 wheel from a URL: that
# wheel declares an unbounded numpy dependency, and an unconstrained resolve
# would happily put NumPy 2.x on top of Humble's 1.x-ABI extensions. A
# constraint only applies a CEILING to something already being resolved, so
# `numpy<2` never forces an install and never conflicts with the wheel - the
# wheel asks for "numpy", the constraint answers "numpy<2", apt's 1.21.5
# already satisfies both and nothing is downloaded.
install -d -m 0755 /etc/fpms
cat > /etc/fpms/pip-constraints.txt <<EOF
# FPMS-OS pip constraints. Referenced by /etc/pip.conf, so this file applies to
# EVERY pip install on this image, including yours.
#
# ROS Humble's C extensions are built against the NumPy 1.x ABI; a 2.x breaks
# tf_transformations with "np.maximum_sctype was removed". See
# /etc/apt/preferences.d/fpms-numpy for the apt half of the same policy.
${PIP_NUMPY}
EOF
chmod 0644 /etc/fpms/pip-constraints.txt

cat > /etc/pip.conf <<'EOF'
[global]
# See /etc/apt/preferences.d/fpms-numpy. ROS Humble is a NumPy 1.x world.
#
# NOTE: if /etc/fpms/pip-constraints.txt is ever deleted, every pip install on
# this image fails with "Could not open requirements file". Stage 20 writes it
# and the end of that stage verifies it exists.
constraint = /etc/fpms/pip-constraints.txt
EOF

# --- how to talk to this pip ------------------------------------------------
#
# --break-system-packages arrived with pip 23.0.1 as the PEP 668 override.
# jammy ships pip 22.0.2, where passing it is a HARD FAILURE - "no such option:
# --break-system-packages", exit 2, and under `set -e` this stage dies on its
# first pip line with no image and a message that sounds like a typo.
#
# It is also UNNECESSARY on jammy: the flag overrides an EXTERNALLY-MANAGED
# marker, and /usr/lib/python3.10/EXTERNALLY-MANAGED does not exist there.
#
# So detect the flag rather than assuming it either way. The day this image's
# base moves past jammy, the flag becomes both available AND required - and
# that day should not be a build failure either.
#
# When it IS available it is intentional, for the reason it always was: this IS
# the system environment for the FPMS services, which run the system python.
PIP_VER="$(pip3 --version 2>/dev/null | awk '{print $2}' || true)"
PIP_HELP="$(pip3 install --help 2>/dev/null || true)"
# Captured into a variable, not piped into `grep -q`: with `set -o pipefail` a
# `pip3 install --help | grep -q ...` reports pip's SIGPIPE death (141), not
# grep's match, and the test comes out backwards.
PIP_ARGS=(--no-cache-dir --disable-pip-version-check)
case "$PIP_HELP" in
    *--break-system-packages*)
        PIP_ARGS+=(--break-system-packages)
        echo "    pip ${PIP_VER:-?} has --break-system-packages (PEP 668 base); using it"
        # Put it in the config too, so pip installs by later stages, by
        # fpms-selftest's suggested fix, and by the operator on the running
        # rover behave the same way without having to remember the flag.
        printf 'break-system-packages = true\n' >> /etc/pip.conf
        ;;
    *)
        echo "    pip ${PIP_VER:-?} has no --break-system-packages; not needed on jammy (no EXTERNALLY-MANAGED)"
        ;;
esac

# --- paho-mqtt >= 2.0 -------------------------------------------------------
#
# THE SINGLE MOST IMPORTANT LINE IN THIS FILE.
#
# Ubuntu 22.04's python3-paho-mqtt is 1.6.1, which has NO CallbackAPIVersion.
# Five FPMS files do:
#
#     from paho.mqtt.client import Client, CallbackAPIVersion
#
# unguarded, at module import. One of them is fpms_cored.py - the STOP
# authority. On an image built with the apt package, that unit crashes at
# import and restart-loops forever, so the rover has NO emergency stop and NO
# telemetry, and the only evidence is a unit quietly cycling.
#
# pip puts this in /usr/local/lib/python3.10/dist-packages, which precedes
# apt's /usr/lib/python3/dist-packages on sys.path, so it wins even if
# something later drags python3-paho-mqtt in as a dependency.
pip3 install "${PIP_ARGS[@]}" "${PIP_PAHO}" "${PIP_WEBSOCKET}"

# --- the NPU ----------------------------------------------------------------
#
# NOT HERE ANY MORE, and deliberately not verified here either.
#
# librknnrt.so and rknn-toolkit-lite2 belong to stage 25 (25-npu-runtime.sh),
# which downloads to a temp file, proves the .so is really an aarch64 ELF
# rather than a 404 page, reads the version string out of the binary, checks
# RKNNLite() constructs, and records the whole version chain in
# /etc/fpms/npu-versions.json. This stage used to do a bare
# `wget -O /usr/lib/librknnrt.so`, which on a 404 leaves a ZERO-BYTE .so that
# ldconfig indexes and dlopen finds - strictly worse than no file at all - and
# then merely warned about it. Two stages fetching the same two files is how
# they drift apart; only one does.
#
# THE SPLIT: stage 20 owns the Python environment, including /etc/pip.conf and
# the numpy constraint above, which stage 25's wheel install depends on. Stage
# 25 owns the NPU version chain and its much harsher failure policy (it fails
# the build; this stage's warn-and-continue was wrong for a dedicated NPU
# stage). The NPU imports are not checked in the verification below because
# stage 25 runs AFTER this one - build.sh globs scripts/[0-9]*.sh - so the
# wheel does not exist yet, and a check here would fail a perfectly good build.

# --- guard against the shadowing problem ------------------------------------
# Remove any user-local site-packages that could shadow the system ones. This
# exact situation broke tf_transformations on the old Pi.
#
# The glob matters: deleting only `numpy/` while leaving `numpy-2.0.0.dist-info`
# behind leaves pip believing 2.x is installed (so it will not reinstall) while
# imports fall through to the system 1.x. Take the metadata and numpy.libs too.
for d in "${FPMS_HOME}"/.local/lib/python3.*/site-packages/numpy*; do
    [ -e "$d" ] || continue
    echo "    removing user-local $d"
    rm -rf "$d"
done
for d in "${FPMS_HOME}"/.local/lib/python3.*/site-packages; do
    [ -d "$d" ] || continue
    if [ -n "$(ls -A "$d" 2>/dev/null)" ]; then
        echo "    NOTE: ${FPMS_USER} has other user-local packages in $d" >&2
        ls -A "$d" | sed 's/^/      /' >&2
        echo "    Anything here shadows the system copy for every FPMS service." >&2
    fi
done

# --- VERIFY. Do not ship an image whose imports do not resolve. -------------
#
# Every one of these has failed in this project at least once, and every one of
# them fails at import time inside a systemd unit, where the traceback goes to
# the journal and nobody reads it until race day.
#
# Run /usr/bin/python3 explicitly - the system interpreter every unit uses,
# either as `/usr/bin/python3` outright or as `python3` after sourcing
# /opt/ros/humble/setup.bash, which changes PYTHONPATH but not the interpreter.
echo "--- verifying imports"
FAILED=0
NOTES=""
verify() {   # verify <name> <python> <what to do about it>
    local out
    if out="$(/usr/bin/python3 -c "$2" 2>&1)"; then
        echo "    ok    $1"
    else
        echo "    FAIL  $1"
        FAILED=1
        NOTES="${NOTES}
  ${1}
      fix:         ${3}
      reproduce:   /usr/bin/python3 -c \"${2}\"
      python said: $(printf '%s' "$out" | tail -n 1)"
    fi
}
verify "numpy (1.x)" \
    "import numpy,sys; print(numpy.__version__, numpy.__file__); sys.exit(0 if numpy.__version__.startswith('1.') else 1)" \
    "a 2.x breaks tf_transformations. Check /etc/apt/preferences.d/fpms-numpy and /etc/fpms/pip-constraints.txt, and look for a user-local copy under ${FPMS_HOME}/.local"
verify "cv2" \
    "import cv2" \
    "apt install python3-opencv (NOT pip opencv-python - it ABI-clashes with cv_bridge)"
# pyserial vs the impostor: the PyPI package literally named `serial` is a
# DIFFERENT library that also imports as `serial` and has no Serial class. It
# silently breaks the agent and the duty driver. serial.tools.list_ports is a
# subpackage `import serial` does NOT pull in on its own - it has to be
# imported by name, or a correct pyserial fails this check with AttributeError.
verify "pyserial" \
    "import serial; from serial.tools import list_ports; serial.Serial" \
    "apt install python3-serial, and pip uninstall the impostor 'serial' if it is present"
verify "paho CallbackAPI" \
    "from paho.mqtt.client import Client, CallbackAPIVersion" \
    "pip3 install '${PIP_PAHO}' - apt's python3-paho-mqtt is 1.6.1 and has no CallbackAPIVersion; fpms_cored.py, the STOP authority, imports it unguarded"
verify "websocket-client" \
    "import websocket; websocket.WebSocket" \
    "pip3 install ${PIP_WEBSOCKET} - NOT 'websockets', which is a different library with a similar name"
verify "yaml" \
    "import yaml" \
    "apt install python3-yaml"

# Not an import, but every later pip install depends on it: /etc/pip.conf names
# this file, and pip refuses to run at all if a named constraints file is
# missing ("Could not open requirements file"). That would take stage 25's
# wheel with it.
if [ -s /etc/fpms/pip-constraints.txt ]; then
    echo "    ok    pip constraints (/etc/fpms/pip-constraints.txt)"
else
    echo "    FAIL  pip constraints"
    FAILED=1
    NOTES="${NOTES}
  pip constraints
      fix:         /etc/pip.conf points at /etc/fpms/pip-constraints.txt, which is missing or
                   empty. Every pip install on this image fails until it exists."
fi

# The checks above ran as root. The SERVICES run as ${FPMS_USER}, whose
# ~/.local is on sys.path and root's is not - which is exactly the asymmetry
# that hid the shadowing numpy on the old Pi. Best effort: if sudo works in
# this chroot, look through the services' eyes too.
if sudo -n -u "${FPMS_USER}" /usr/bin/python3 -c 'pass' >/dev/null 2>&1; then
    USER_NUMPY="$(sudo -n -u "${FPMS_USER}" /usr/bin/python3 \
        -c 'import numpy; print(numpy.__version__, numpy.__file__)' 2>&1 || true)"
    case "$USER_NUMPY" in
        1.*"/usr/"*) echo "    ok    numpy as ${FPMS_USER}: ${USER_NUMPY}" ;;
        *)  echo "    FAIL  numpy as ${FPMS_USER}"
            FAILED=1
            NOTES="${NOTES}
  numpy as ${FPMS_USER}
      fix:         ${FPMS_USER} imports '${USER_NUMPY}', which is not a system 1.x.
                   Remove ${FPMS_HOME}/.local/lib/python3.*/site-packages/numpy*
      reproduce:   sudo -u ${FPMS_USER} python3 -c 'import numpy; print(numpy.__version__, numpy.__file__)'" ;;
    esac
else
    echo "    ....  skipped the ${FPMS_USER} import check (sudo does not work in this chroot)"
fi

if [ "$FAILED" = 1 ]; then
    echo "" >&2
    echo "FATAL: one or more required Python checks did not pass:" >&2
    printf '%s\n' "$NOTES" >&2
    echo "" >&2
    echo "The image would boot with units crash-looping at import, every one of" >&2
    echo "them reporting only in the journal. Fix the above and re-run:" >&2
    echo "    sudo ./build.sh --stage 20" >&2
    exit 1
fi

echo "--- 20-python-deps OK"
