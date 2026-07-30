#!/usr/bin/env bash
# Convert YOLO26 to RKNN for the RK3588 NPU.
#
# Must run on x86 Linux: rknn-toolkit2 (the converter) is x86-only, while the
# Pi has rknn-toolkit-lite2, which can only *run* models.
#
# Version pins matter here:
#   onnx<1.16  — rknn-toolkit2 calls onnx.mapping, removed in 1.16
#                ("AttributeError: module 'onnx' has no attribute 'mapping'")
#   numpy<2    — rknn-toolkit2 is built against the NumPy 1.x C API
#
# end2end stays OFF: Ultralytics' RKNN docs report end2end=True emits a
# [1,300,6] output whose top-k op segfaults on the NPU.
set -e

cd /out

echo "=== pinning compatible versions ==="
pip install -q --no-cache-dir "onnx==1.14.1" "numpy<2" 2>&1 | tail -3
python -c "import onnx, numpy; print('onnx', onnx.__version__, '| numpy', numpy.__version__)"
python -c "import onnx; print('onnx.mapping present:', hasattr(onnx, 'mapping'))"

echo "=== exporting yolo26n -> rknn (rk3588) ==="
python - <<'PY'
from ultralytics import YOLO
import traceback
try:
    m = YOLO("yolo26n.pt")
    path = m.export(format="rknn", name="rk3588")
    print("EXPORTED:", path)
except Exception as e:
    print("EXPORT FAILED:", type(e).__name__, str(e)[:300])
    traceback.print_exc()
PY

echo "=== artefacts ==="
find /out -name '*.rknn' -exec ls -la {} \;
