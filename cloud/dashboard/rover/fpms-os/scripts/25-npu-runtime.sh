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
#   !! Stage 20 still contains that block. It is now REDUNDANT. This stage runs
#   !! after it (build.sh globs scripts/[0-9]*.sh, so 25 sorts after 20) and
#   !! overwrites both halves, so the image is correct either way - but the
#   !! duplicate download should be deleted from stage 20 by its owner.
#   !! Do NOT fix it from here: scripts/20-python-deps.sh is not this agent's
#   !! file, and two agents editing one script is how this repo gets a third
#   !! near-identical copy of something.
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

WHEEL_URL="${RKNN_LITE_WHEEL_URL:?RKNN_LITE_WHEEL_URL not set - build.sh must export it}"
SO_URL="${LIBRKNNRT_URL:?LIBRKNNRT_URL not set - build.sh must export it}"
TAG="${RKNN_TOOLKIT2_TAG:-unknown}"
SO_DEST=/usr/lib/librknnrt.so
JSON=/etc/fpms/npu-versions.json

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

install -d /etc/fpms

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
# boot time. Agent 7's fpms-selftest is the consumer: it reads
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
    python3 - "$JSON" "$status" <<'PY'
import json, os, subprocess, sys

out, status = sys.argv[1], sys.argv[2]

def env(k, default=None):
    v = os.environ.get(k)
    return v if v else default

doc = {
    "schema": 1,
    "generated_by": "fpms-os scripts/25-npu-runtime.sh",
    "generated_at": subprocess.run(["date", "-u", "+%Y-%m-%dT%H:%M:%SZ"],
                                   capture_output=True, text=True).stdout.strip(),
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
        "url": env("LIBRKNNRT_URL"),
        "read_with": "strings /usr/lib/librknnrt.so | grep -i 'librknnrt version'",
    },

    # --- link 3 of the chain: the Python wheel ----------------------------
    "wheel": {
        "name": "rknn-toolkit-lite2",
        "version": env("WHEEL_VER"),
        "url": env("RKNN_LITE_WHEEL_URL"),
        "python_tag": "cp310",
        "platform_tag": "manylinux_2_17_aarch64",
        "import_ok": status == "INSTALLED",
        "rknnlite_constructible": status == "INSTALLED",
        "read_with": "pip3 show rknn-toolkit-lite2",
        "note": ("import + construct only. load_rknn()/init_runtime() cannot "
                 "run in a chroot: no NPU device, and the .rknn model is not "
                 "in the repository."),
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
            "is therefore deferred to boot: fpms-selftest reads the live value "
            "and checks it against driver.expected."),
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

with open(out, "w") as fh:
    json.dump(doc, fh, indent=2)
    fh.write("\n")
PY
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
# records that it is crippled so it can never be mistaken for a good one:
#
#     # from the HOST, with the chroot still mounted:
#     sudo touch .build/mnt/opt/fpms-os/ALLOW_NPU_MISSING
#     sudo ./build.sh --stage 25
#
# (A marker FILE, not an environment variable, because build.sh's in_chroot()
# runs `env -i` with an explicit variable list - an env var set on the host
# does not reach this script.)
#
# When that marker is present the stage writes npu-versions.json with
# "status": "ABSENT", which agent 7's fpms-selftest reports as a FAIL on every
# single boot. Degraded is allowed. Degraded and quiet is not.
ALLOW_MISSING=0
if [ -e /opt/fpms-os/ALLOW_NPU_MISSING ] || [ "${FPMS_ALLOW_NPU_MISSING:-0}" = 1 ]; then
    ALLOW_MISSING=1
    echo "    ALLOW_NPU_MISSING is set - NPU failures will not fail the build" >&2
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

 Or fix on a running rover, then re-run fpms-selftest:
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
ARCH="$(uname -m)"
PYV="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
[ "$ARCH" = "aarch64" ] || { echo "FATAL: chroot reports arch '$ARCH', expected aarch64." >&2
    echo "The rknn-toolkit-lite2 wheel is aarch64-only. Is qemu-aarch64 binfmt registered?" >&2; exit 1; }
[ "$PYV" = "3.10" ] || { echo "FATAL: chroot python is $PYV, expected 3.10." >&2
    echo "The wheel is cp310-only, and ROS Humble's rclpy extensions are cpython-310." >&2; exit 1; }
echo "    arch=$ARCH python=$PYV toolkit2_tag=$TAG"

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

echo "    fetching librknnrt.so"
# --tries/--timeout so a flaky mirror fails in a minute rather than hanging the
# build. GitHub's /raw/ path redirects to raw.githubusercontent.com; wget
# follows that by default.
if ! wget -q --tries=3 --timeout=30 -O "$TMP/librknnrt.so" "$SO_URL"; then
    npu_fail "could not download librknnrt.so from $SO_URL" \
             "sudo wget -O $SO_DEST $SO_URL && sudo ldconfig"
fi

# Is it actually an aarch64 shared object, or is it an HTML error page with a
# .so name? Checked with python rather than file(1)/readelf, neither of which
# is guaranteed present in the base image.
#   bytes 0-3   \x7fELF
#   byte  4     EI_CLASS, 2 = ELF64
#   bytes 18-19 e_machine, 183 (0xB7) = EM_AARCH64
if ! python3 - "$TMP/librknnrt.so" <<'PY'
import sys
p = sys.argv[1]
b = open(p, "rb").read(20)
import os
sz = os.path.getsize(p)
if b[:4] != b"\x7fELF" or b[4] != 2 or int.from_bytes(b[18:20], "little") != 183:
    sys.stderr.write(
        "    %s is not an aarch64 ELF64 shared object (%d bytes, first bytes %r)\n"
        % (p, sz, b[:16]))
    sys.exit(1)
PY
then
    npu_fail "the downloaded librknnrt.so is not an aarch64 ELF (a 404 page?)" \
             "check $SO_URL resolves - upstream may have moved the path at tag $TAG"
fi

install -m 0644 -o root -g root "$TMP/librknnrt.so" "$SO_DEST"

# ldconfig, so the loader can find it by SONAME rather than by luck. Required
# after every replacement of the file, and harmless to repeat.
ldconfig
if ! ldconfig -p | grep -q 'librknnrt\.so'; then
    # Not fatal on its own - the runtime may dlopen it by absolute path - but
    # if the cache does not have it, something about the install is off and it
    # is worth saying so out loud rather than discovering it at init_runtime().
    echo "    WARNING: librknnrt.so is not in the ldconfig cache after ldconfig" >&2
    echo "    Check /etc/ld.so.conf.d/ includes /usr/lib" >&2
fi

# The version string is EMBEDDED IN THE BINARY. The canonical way to read it is
#     strings librknnrt.so | grep -i 'librknnrt version'
# which yields something of the form
#     librknnrt version: 2.3.0 (<commit>@<build date>)
# strings(1) comes from binutils and is not guaranteed in the base image, so
# the same search is done here over the raw bytes. This is the ONLY place the
# runtime version can be obtained without hardware.
SO_VER="$(python3 - "$SO_DEST" <<'PY'
import re, sys
data = open(sys.argv[1], "rb").read()
m = re.search(rb"librknnrt version:?\s*([0-9][0-9A-Za-z._-]*)", data, re.I)
print(m.group(1).decode("ascii", "replace") if m else "")
PY
)"
SO_VER_FULL="$(python3 - "$SO_DEST" <<'PY'
import re, sys
data = open(sys.argv[1], "rb").read()
m = re.search(rb"librknnrt version[^\x00]{0,160}", data, re.I)
print(m.group(0).decode("ascii", "replace").strip() if m else "")
PY
)"
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

SO_SHA="$(sha256sum "$SO_DEST" | awk '{print $1}')"
SO_SIZE="$(stat -c %s "$SO_DEST")"

# ---------------------------------------------------------------------------
# rknn-toolkit-lite2 (the wheel)
# ---------------------------------------------------------------------------
#
# NOT on PyPI - it is fetched from Rockchip's repo at the pinned tag, which is
# why it comes in as a URL. --break-system-packages is required on this base
# and is intentional, for the same reason as in stage 20: this IS the system
# environment for the FPMS services, which run the system python.
#
# The pip install honours /etc/pip.conf -> /etc/fpms/pip-constraints.txt
# (written by stage 20), which pins numpy<2. That matters here specifically:
# rknn-toolkit-lite2 declares a numpy dependency, and an unconstrained resolve
# would happily pull NumPy 2.x on top of ROS Humble's 1.x-ABI C extensions and
# break tf_transformations. Verified below rather than assumed.
#
# Idempotent: pip reinstalls from a direct URL every run, which is what we
# want - it makes the installed wheel a function of the config file only.
echo "    installing rknn-toolkit-lite2"
if ! pip3 install --no-cache-dir --break-system-packages "$WHEEL_URL"; then
    npu_fail "pip could not install the rknn-toolkit-lite2 wheel" \
             "pip3 install --break-system-packages $WHEEL_URL"
fi

# ---------------------------------------------------------------------------
# VERIFY. Do not ship an image whose NPU imports do not resolve.
# ---------------------------------------------------------------------------
#
# Import + construct only. load_rknn() and init_runtime() CANNOT be exercised
# here: there is no NPU device in the chroot and no model in the image (the
# .rknn lives only on the old Pi - see stage 30). So this proves the wheel is
# installed and its native extension loads; it does not prove inference works.
# That proof is agent 7's fpms-selftest, on hardware, and nothing here should
# be read as a substitute for it.
VERIFY_OUT="$TMP/verify.json"
if ! python3 - "$VERIFY_OUT" <<'PY'
import json, sys
res = {"import_ok": False, "rknnlite_constructible": False,
       "wheel_version": None, "numpy_version": None, "error": None}
try:
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
    # The constructor does not touch the device - it wires up the runtime
    # bindings. If librknnrt.so is missing, corrupt, or the wrong arch, this
    # is where it shows, which is exactly the check we want at build time.
    RKNNLite()
    res["rknnlite_constructible"] = True
    import numpy
    res["numpy_version"] = numpy.__version__
except Exception as exc:
    res["error"] = "%s: %s" % (type(exc).__name__, exc)
open(sys.argv[1], "w").write(json.dumps(res))
sys.exit(0 if res["rknnlite_constructible"] else 1)
PY
then
    echo "    detail: $(cat "$VERIFY_OUT" 2>/dev/null || echo '(no detail)')" >&2
    npu_fail "rknnlite imports or constructs incorrectly after install" \
             "python3 -c 'from rknnlite.api import RKNNLite; RKNNLite()'"
fi

WHEEL_VER="$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["wheel_version"] or "")' "$VERIFY_OUT")"
NUMPY_VER="$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["numpy_version"] or "")' "$VERIFY_OUT")"
echo "    rknn-toolkit-lite2 ${WHEEL_VER:-(version unknown)}  numpy ${NUMPY_VER:-?}"

# numpy must still be 1.x AFTER the wheel's dependency resolve. A 2.x here does
# not break the NPU - it breaks tf_transformations, silently, three stages
# later, and this is the last stage that could have caused it.
case "$NUMPY_VER" in
    1.*) : ;;
    *)   echo "FATAL: numpy is '${NUMPY_VER:-missing}' after installing rknn-toolkit-lite2." >&2
         echo "ROS Humble's C extensions are built against the NumPy 1.x ABI; a 2.x breaks" >&2
         echo "tf_transformations with 'np.maximum_sctype was removed'. Check that" >&2
         echo "/etc/pip.conf points at /etc/fpms/pip-constraints.txt (stage 20)." >&2
         exit 1 ;;
esac

# ---------------------------------------------------------------------------
# Record what was installed. See write_json() near the top of this file.
# ---------------------------------------------------------------------------
#
# Exported so the python heredoc inside write_json() can see them; build.sh's
# in_chroot() runs `env -i`, so nothing is inherited that was not put there
# deliberately, and nothing reaches a child that was not exported here.
export SO_VER SO_VER_FULL SO_SHA SO_SIZE WHEEL_VER NUMPY_VER
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
         fpms-selftest checks it on the running board.
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
