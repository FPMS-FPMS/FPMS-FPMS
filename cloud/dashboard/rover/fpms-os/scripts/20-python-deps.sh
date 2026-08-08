#!/usr/bin/env bash
# Stage 20 - Python dependencies.
#
# THIS IS THE STAGE MOST LIKELY TO PRODUCE A DEAD IMAGE, and the failures are
# all silent. Read the comments before changing a pin.
set -euo pipefail
echo "--- 20-python-deps"

export DEBIAN_FRONTEND=noninteractive

# --- from apt ---------------------------------------------------------------
#
# python3-opencv from apt, NOT opencv-python from pip. The pip wheel links its
# own FFmpeg/GStreamer and ABI-clashes with ROS's cv_bridge, which is a classic
# and extremely confusing breakage. The apt build also has V4L2 enabled, which
# the agent needs for its MJPG capture (CAP_PROP_FOURCC).
#
# python3-serial from apt is pyserial. See the impostor note below.
apt-get install -y -qq --no-install-recommends \
    python3-opencv python3-serial python3-numpy python3-yaml

# --- numpy must stay on 1.x -------------------------------------------------
#
# ROS Humble's C extensions are built against the NumPy 1.x ABI. A 2.x breaks
# tf_transformations with "np.maximum_sctype was removed in the NumPy 2.0
# release" - and it broke it on the old Pi in the most confusing possible way,
# because the offending copy was a USER-LOCAL install in
# /home/ubuntu/.local/lib/python3.10/site-packages that shadowed the system
# one for every service.
#
# Pin it at the apt level so a stray `pip install -U numpy` cannot walk over it.
cat > /etc/apt/preferences.d/fpms-numpy <<'EOF'
Package: python3-numpy
Pin: version 1.*
Pin-Priority: 1001
EOF

# And at the pip level.
install -d /etc/pip
cat > /etc/pip.conf <<EOF
[global]
# See /etc/apt/preferences.d/fpms-numpy. ROS Humble is a NumPy 1.x world.
constraint = /etc/fpms/pip-constraints.txt
EOF
install -d /etc/fpms
cat > /etc/fpms/pip-constraints.txt <<EOF
${PIP_NUMPY}
EOF

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
# --break-system-packages is required on this base and is intentional: this IS
# the system environment for the FPMS services, which run the system python.
pip3 install --no-cache-dir --break-system-packages \
    "${PIP_PAHO}" "${PIP_WEBSOCKET}"

# --- the NPU ----------------------------------------------------------------
#
# rknn-toolkit-lite2 is a Rockchip aarch64 wheel, NOT on PyPI, and it must be
# version-matched to librknnrt.so, which must in turn match the kernel's rknpu
# driver. (The CONVERTER, rknn-toolkit2, is x86-only and is not installed here
# - the Pi can only run models, not build them.)
#
# FAIL LOUDLY if this cannot be fetched. A silently-blind rover is the worst
# outcome: fpms_rover_agent wraps the whole NPU block in one try/except that
# logs "NPU unavailable (...); streaming without detection" and carries on, so
# systemd reports every unit active while the rover detects nothing at all.
NPU_OK=1
if ! pip3 install --no-cache-dir --break-system-packages "${RKNN_LITE_WHEEL_URL}"; then
    NPU_OK=0
fi
if ! wget -q -O /usr/lib/librknnrt.so "${LIBRKNNRT_URL}"; then
    NPU_OK=0
fi

if [ "$NPU_OK" = 0 ]; then
    cat >&2 <<EOF

########################################################################
 NPU RUNTIME NOT INSTALLED

 rknn-toolkit-lite2 and/or librknnrt.so could not be fetched. The image
 will boot and drive, but it will DETECT NOTHING -- and it will not say
 so: the agent logs "NPU unavailable; streaming without detection" once
 and continues, with every unit reporting active.

 Fix on the running rover, then re-run fpms-selftest:

   pip3 install --break-system-packages ${RKNN_LITE_WHEEL_URL}
   sudo wget -O /usr/lib/librknnrt.so ${LIBRKNNRT_URL}
   cat /sys/kernel/debug/rknpu/version    # must match the runtime

 Refusing to fail the build over this, because a rover that patrols and
 streams is still useful. fpms-selftest reports it as a FAIL every boot.
########################################################################

EOF
else
    ldconfig
fi

# --- guard against the shadowing problem ------------------------------------
# Remove any user-local site-packages that could shadow the system ones. This
# exact situation broke tf_transformations on the old Pi.
rm -rf "${FPMS_HOME}/.local/lib/python3.10/site-packages/numpy" 2>/dev/null || true

# --- VERIFY. Do not ship an image whose imports do not resolve. -------------
#
# Every one of these has failed in this project at least once, and every one of
# them fails at import time inside a systemd unit, where the traceback goes to
# the journal and nobody reads it until race day.
echo "--- verifying imports"
FAILED=0
verify() {
    if python3 -c "$2" >/dev/null 2>&1; then
        echo "    ok    $1"
    else
        echo "    FAIL  $1"; FAILED=1
    fi
}
verify "numpy (1.x)"      "import numpy,sys; sys.exit(0 if numpy.__version__.startswith('1.') else 1)"
verify "cv2"              "import cv2"
verify "pyserial"         "import serial; serial.Serial; serial.tools.list_ports"
verify "paho CallbackAPI" "from paho.mqtt.client import Client, CallbackAPIVersion"
verify "websocket-client" "import websocket; websocket.WebSocket"
verify "yaml"             "import yaml"

# pyserial vs the impostor: the PyPI package literally named `serial` is a
# DIFFERENT library that also imports as `serial` and has no Serial class. It
# silently breaks the agent and the duty driver.
if [ "$FAILED" = 1 ]; then
    echo "FATAL: one or more required Python imports do not resolve." >&2
    echo "The image would boot with units crash-looping at import." >&2
    exit 1
fi

echo "--- 20-python-deps OK"
