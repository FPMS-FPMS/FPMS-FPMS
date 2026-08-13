#!/usr/bin/env bash
# Rebuild yolo26n-rk3588.rknn from yolo26n.pt.
#
#     ./convert_yolo26.sh [--quantize int8 --dataset calib.txt] [--onnx file]
#
# WHY THIS EXISTS
# ---------------
# The model the rover detects fire with exists on ONE SD card. It is in
# .gitignore (cloud/.gitignore: "dashboard/models/*_rknn_model/") and in no
# repository. cloud/dashboard/HANDOFF.md records it as 7,445,558 bytes at
# cloud/dashboard/models/yolo26n_rknn_model/yolo26n-rk3588.rknn - a path that
# does not exist in a fresh checkout. If that card dies, the rover streams video
# and detects nothing, and there is nothing in git that regenerates it.
#
# This script is the insurance. It supersedes cloud/dashboard/models/convert_yolo26.sh,
# which was a Docker scratch script: it ran Ultralytics' one-shot RKNN export,
# printed "EXPORTED:" and stopped. It never checked the architecture, never
# verified the output, and left no record of what it had produced. Everything it
# established - the pins, end2end off - is carried forward here.
#
# THIS RUNS ON x86_64 LINUX. NEVER ON THE PI.
# rknn-toolkit2 (the converter) has no aarch64 build. The rover has
# rknn-toolkit-lite2, which can only RUN models. The check below is the first
# thing this script does, because the failure you get otherwise is an obscure
# pip resolution error in a venv, which reads as "the script is broken".
#
# STATUS: this exact script has not yet reproduced the deployed model. The board
# has not arrived and the old Pi has not been re-run against it. What it does
# guarantee is that it will not emit a model that fails its own checks.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# Knobs. Defaults are the values that produced the deployed model.
# ---------------------------------------------------------------------------
# The output name is NOT free. fpms_yolo26_npu.py line 47 hardcodes
#     MODEL = "/home/ubuntu/yolo/yolo26n-rk3588.rknn"
# with no override, and that is also the name Ultralytics' RKNN exporter
# produces ("<stem>-<name>.rknn"), which is why the file on the Pi is called
# that. Rename it and the agent logs "NPU unavailable" and streams blind.
MODEL_STEM="${FPMS_MODEL_STEM:-yolo26n}"
RKNN_NAME="${MODEL_STEM}-rk3588.rknn"

# Work outside the repo: a venv here would be ~2 GB of torch inside a tree that
# gets copied verbatim into the image overlay, and .gitignore is owned by
# another agent so it cannot be amended from this file.
WORKDIR="${FPMS_CONVERT_WORKDIR:-${TMPDIR:-/tmp}/fpms-yolo26-convert}"
VENV="$WORKDIR/venv"

QUANTIZE="fp16"
DATASET=""
ONNX_IN=""
EXTRA_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --quantize) QUANTIZE="$2"; shift 2 ;;
        --dataset)  DATASET="$2";  shift 2 ;;
        --onnx)     ONNX_IN="$2";  shift 2 ;;
        --workdir)  WORKDIR="$2"; VENV="$WORKDIR/venv"; shift 2 ;;
        --allow-unverified) EXTRA_ARGS+=(--allow-unverified); shift ;;
        -h|--help)  sed -n '2,40p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
    esac
done

say()  { printf '\n=== %s\n' "$*"; }
die()  { printf '\nFATAL: %s\n\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# 1. Architecture. Refuse early and explain.
# ---------------------------------------------------------------------------
say "host check"
ARCH="$(uname -m)"
OS="$(uname -s)"
printf '    %s %s\n' "$OS" "$ARCH"

if [ "$OS" != "Linux" ] || { [ "$ARCH" != "x86_64" ] && [ "$ARCH" != "amd64" ]; }; then
    cat >&2 <<EOF

########################################################################
 WRONG MACHINE

 This is ${OS}/${ARCH}. The RKNN converter is x86_64 Linux only.

   rknn-toolkit2       x86_64 Linux   CONVERTS models   <- what you need
   rknn-toolkit-lite2  aarch64        RUNS models       <- what the Pi has

 There is no aarch64 build of the converter and there never has been. If
 you are on the rover, stop: the Pi cannot build this model, only run it.

 Run this on an x86_64 Linux box, or in a container on one:

   docker run --rm -it -v "\$PWD:/work" -w /work python:3.10-slim \\
       bash -c 'apt-get update && apt-get install -y -qq git libgl1 \\
                libglib2.0-0 && ./convert_yolo26.sh'

 Then copy the .rknn to the rover.
########################################################################

EOF
    exit 1
fi

# ---------------------------------------------------------------------------
# 2. Which converter version. Track the runtime the image pins.
# ---------------------------------------------------------------------------
#
# A model built by a NEWER toolkit than the runtime on the board fails at
# init_runtime() - and fpms_rover_agent.py wraps that in one try/except that
# logs "NPU unavailable (...); streaming without detection" once and carries on
# forever, with every unit reporting active. That is the failure NPU_SPEC.md
# calls the worst in the project. So the converter version is read from the same
# file that pins the rover's runtime, not typed in here.
CONF="$HERE/../../config/fpms-os.conf"
if [ -f "$CONF" ]; then
    # shellcheck disable=SC1090
    . "$CONF"
    say "converter version, from config/fpms-os.conf"
else
    say "converter version (config/fpms-os.conf not found - using fallback)"
fi
RKNN_TOOLKIT2_TAG="${RKNN_TOOLKIT2_TAG:-v2.3.0}"
TOOLKIT_VER="${RKNN_TOOLKIT2_TAG#v}"
printf '    rknn-toolkit2 %s  (rover runs rknn-toolkit-lite2 %s + the matching librknnrt.so)\n' \
    "$TOOLKIT_VER" "$TOOLKIT_VER"

# ---------------------------------------------------------------------------
# 3. Python. 3.10 to match the rover; anything the wheel supports will convert.
# ---------------------------------------------------------------------------
say "interpreter"
PY=""
for c in python3.10 python3.11 python3.9 python3.8 python3; do
    if command -v "$c" >/dev/null 2>&1; then PY="$c"; break; fi
done
[ -n "$PY" ] || die "no python3 on PATH."
PYV="$("$PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
printf '    %s (%s)\n' "$PY" "$PYV"
case "$PYV" in
    3.8|3.9|3.10|3.11|3.12) : ;;
    *) die "python $PYV: Rockchip publishes rknn-toolkit2 wheels for cp38-cp312 only.
  Install python3.10 (the rover's version) and re-run.
  Check what the pinned release actually ships:
    https://github.com/airockchip/rknn-toolkit2/tree/${RKNN_TOOLKIT2_TAG}/rknn-toolkit2/packages" ;;
esac

# ---------------------------------------------------------------------------
# 4. The venv, with the pins that each prevent a specific failure.
# ---------------------------------------------------------------------------
mkdir -p "$WORKDIR"
if [ ! -x "$VENV/bin/python" ]; then
    say "creating venv at $VENV"
    "$PY" -m venv "$VENV"
else
    say "reusing venv at $VENV"
fi
PIP="$VENV/bin/pip"
PYBIN="$VENV/bin/python"

"$PIP" install -q --upgrade pip setuptools wheel

say "pinned dependencies"
#
#   onnx==1.14.1  rknn-toolkit2 calls onnx.mapping, which was REMOVED in onnx
#                 1.16. Symptom: "AttributeError: module 'onnx' has no attribute
#                 'mapping'", thrown from inside the toolkit part-way through a
#                 build, after several minutes. Established the first time this
#                 conversion was attempted (HANDOFF.md).
#
#   numpy<2       rknn-toolkit2 is built against the NumPy 1.x C API. Also the
#                 rule everywhere else in this project: ROS Humble's C
#                 extensions are 1.x-ABI, and a stray 2.x broke tf_transformations
#                 on the old Pi with "np.maximum_sctype was removed".
#
# Install order matters: ultralytics pulls a recent numpy/onnx of its own, so it
# goes FIRST and the pins go last and win.
"$PIP" install -q --no-cache-dir "ultralytics" \
    || die "ultralytics install failed"
"$PIP" install -q --no-cache-dir "onnx==1.14.1" "numpy<2" \
    || die "could not pin onnx==1.14.1 / numpy<2"

say "rknn-toolkit2 ${TOOLKIT_VER}"
if ! "$PYBIN" -c 'import rknn.api' >/dev/null 2>&1; then
    # rknn-toolkit2 2.x is published on PyPI. 1.x was wheel-only, and PyPI
    # availability for a given point release is NOT guaranteed - hence the
    # fallback below rather than a hard failure.
    if ! "$PIP" install -q --no-cache-dir "rknn-toolkit2==${TOOLKIT_VER}"; then
        cat >&2 <<EOF

  pip could not install rknn-toolkit2==${TOOLKIT_VER} from PyPI.

  Fall back to Rockchip's wheel. The exact filename is NOT guessed here on
  purpose - it carries a build suffix that changes between releases, and a
  wrong URL 404s into a confusing error. List the directory for the tag this
  image pins and use the cp${PYV/./} x86_64 wheel:

    https://github.com/airockchip/rknn-toolkit2/tree/${RKNN_TOOLKIT2_TAG}/rknn-toolkit2/packages

  then:

    RKNN_TOOLKIT2_WHEEL_URL=<that url> $0

EOF
        [ -n "${RKNN_TOOLKIT2_WHEEL_URL:-}" ] \
            || die "set RKNN_TOOLKIT2_WHEEL_URL and re-run."
        "$PIP" install -q --no-cache-dir "$RKNN_TOOLKIT2_WHEEL_URL" \
            || die "wheel install failed: $RKNN_TOOLKIT2_WHEEL_URL"
    fi
fi

say "resolved versions"
"$PYBIN" - <<'PY'
import platform
def v(mod, dist=None):
    try:
        from importlib.metadata import version
        return version(dist or mod)
    except Exception:
        try:
            return getattr(__import__(mod), "__version__", "?")
        except Exception:
            return "MISSING"
print("    python        ", platform.python_version())
print("    numpy         ", v("numpy"))
print("    onnx          ", v("onnx"))
print("    ultralytics   ", v("ultralytics"))
print("    rknn-toolkit2 ", v("rknn", "rknn-toolkit2"))
import onnx
assert hasattr(onnx, "mapping"), "onnx.mapping is gone - the pin did not take"
print("    onnx.mapping   present (rknn-toolkit2 needs it)")
import numpy
assert numpy.__version__.startswith("1."), "numpy is 2.x - the pin did not take"
PY
# Record the resolved set. The whole point of this pipeline is that the NEXT
# person knows what produced a working model; ultralytics is deliberately
# unpinned above (YOLO26 support is recent and moving) so this file is the only
# record of which version was used.
"$VENV/bin/pip" freeze > "$WORKDIR/requirements-resolved.txt"
printf '    frozen -> %s\n' "$WORKDIR/requirements-resolved.txt"

# ---------------------------------------------------------------------------
# 5. .pt -> ONNX, with end2end OFF.
# ---------------------------------------------------------------------------
cd "$WORKDIR"
PT_PATH="$WORKDIR/${MODEL_STEM}.pt"
ONNX_PATH="$WORKDIR/${MODEL_STEM}.onnx"

if [ -n "$ONNX_IN" ]; then
    say "using supplied ONNX: $ONNX_IN"
    ONNX_PATH="$(cd "$(dirname "$ONNX_IN")" && pwd)/$(basename "$ONNX_IN")"
    PT_PATH=""
else
    say "exporting ${MODEL_STEM}.pt -> ONNX (end2end OFF)"
    #
    # end2end=False is MANDATORY. Ultralytics' RKNN docs report end2end=True
    # emits [1,300,6] whose top-k op SEGFAULTS on the RK3588 NPU - the agent
    # process dies, it does not degrade.
    #
    # The kwarg is passed AND the resulting graph is checked (rknn_convert.py
    # refuses any graph containing NonMaxSuppression or TopK), because
    # Ultralytics has moved this flag between `end2end=`, `nms=` and a model
    # attribute across versions and a kwarg a given version silently ignores is
    # not a guarantee. The op histogram is.
    #
    # opset 19 matches the ONNX the deployed model was read off
    # (HANDOFF.md: "Reading models/yolo26n.onnx directly (opset 19)").
    "$PYBIN" - "$MODEL_STEM" <<'PY'
import shutil, sys, traceback
from pathlib import Path
from ultralytics import YOLO

stem = sys.argv[1]
m = YOLO(f"{stem}.pt")          # downloads the stock checkpoint if absent

# Belt and braces: some versions carry end2end as a model attribute rather than
# an export kwarg, and setting it here is a no-op where it does not apply.
try:
    if hasattr(m.model, "end2end"):
        m.model.end2end = False
except Exception:
    pass

common = dict(format="onnx", imgsz=640, opset=19, simplify=True,
              dynamic=False, batch=1, nms=False)
try:
    path = m.export(end2end=False, **common)
except TypeError as e:
    print(f"  (this ultralytics rejects end2end= : {e}; "
          "falling back to nms=False alone - the graph check is the real gate)")
    path = m.export(**common)
except Exception:
    traceback.print_exc()
    raise SystemExit("ONNX export failed")

path = Path(path)
want = Path(f"{stem}.onnx")
if path.resolve() != want.resolve():
    shutil.copy2(path, want)
print(f"  onnx -> {want.resolve()}")
PY
    [ -f "$ONNX_PATH" ] || die "expected $ONNX_PATH after export, not found"
fi

# ---------------------------------------------------------------------------
# 6. ONNX -> RKNN, verified.
# ---------------------------------------------------------------------------
say "converting to RKNN (rk3588, ${QUANTIZE})"
CONV_ARGS=(--onnx "$ONNX_PATH" --out "$WORKDIR/$RKNN_NAME" --quantize "$QUANTIZE")
[ -n "$PT_PATH" ] && CONV_ARGS+=(--pt "$PT_PATH")
[ -n "$DATASET" ] && CONV_ARGS+=(--dataset "$DATASET")
[ ${#EXTRA_ARGS[@]} -gt 0 ] && CONV_ARGS+=("${EXTRA_ARGS[@]}")

"$PYBIN" "$HERE/rknn_convert.py" "${CONV_ARGS[@]}"

# ---------------------------------------------------------------------------
# 7. What to do next. Spelled out - this is where the model has to actually go.
# ---------------------------------------------------------------------------
SHA="$(sha256sum "$WORKDIR/$RKNN_NAME" | cut -d' ' -f1)"

cat <<EOF

########################################################################
 DONE

   $WORKDIR/$RKNN_NAME
   $WORKDIR/$RKNN_NAME.json      <- registry entry
   $WORKDIR/requirements-resolved.txt

   sha256  $SHA

 NEXT - and none of these steps are optional:

 1. Copy to the rover. The path and the filename are hardcoded in
    fpms_yolo26_npu.py; do not rename it.

      scp $WORKDIR/$RKNN_NAME \\
          ubuntu@fpms-rover1.local:/home/ubuntu/yolo/
      scp $WORKDIR/$RKNN_NAME.json \\
          ubuntu@fpms-rover1.local:/home/ubuntu/yolo/

    Use the name, never an IP. This rover's address has changed more than
    seven times and every note that wrote one down was wrong the next day.

 2. Register it, so a wrong model is a declared fault instead of garbage
    boxes. Merge the sidecar into the registry:

      sudo fpms-model-verify --register /home/ubuntu/yolo/$RKNN_NAME.json
      # or splice the object into models[] in /etc/fpms/models.json by hand

 3. Verify ON THE BOARD. The simulator check this pipeline ran proves the
    SHAPE and that DFL/sigmoid are folded in. It cannot prove the runtime
    accepts the model - that depends on librknnrt.so matching the kernel
    rknpu driver, which is a different question entirely.

      fpms-model-verify
      cat /sys/kernel/debug/rknpu/version
      cat /etc/fpms/versions.json

 4. Select it, then look at the video:

      # /etc/fpms/config.env
      FPMS_YOLO_VARIANT=v26
      sudo systemctl restart fpms-rover-agent
      journalctl -u fpms-rover-agent -f | grep -i npu

    "NPU ready with /home/ubuntu/yolo/$RKNN_NAME (variant v26)" is the line
    you need. "NPU unavailable (...); streaming without detection" means the
    rover is blind - and it will keep reporting active forever if you let it.

 5. Commit the sidecar (not the .rknn - it is gitignored) so the next person
    can tell which model a rover is carrying.
########################################################################

EOF
