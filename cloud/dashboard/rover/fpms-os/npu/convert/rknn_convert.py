#!/usr/bin/env python3
"""ONNX -> RKNN for the RK3588 NPU, with the graph contract enforced.

This is the second half of the pipeline. `convert_yolo26.sh` builds the venv
and produces `yolo26n.onnx`; this file turns that into
`yolo26n-rk3588.rknn`, verifies it, and writes a sidecar JSON so the model can
be registered instead of trusted.

WHY THIS FILE EXISTS
--------------------
`yolo26n-rk3588.rknn` (7,445,558 bytes, per `cloud/dashboard/HANDOFF.md`) lives
on the old Pi and in `.gitignore`. It is in no repository. If that card dies the
rover cannot detect fire and nothing in git regenerates it. Prose describing a
model is not a model.

WHAT THE PRODUCED GRAPH MUST LOOK LIKE, AND WHO DEPENDS ON IT
------------------------------------------------------------
`cloud/dashboard/rover/fpms_yolo26_npu.py` is the decoder and it makes four
assumptions that this converter is responsible for preserving:

  1. ONE output tensor, 84 x 8400.  `_as_anchor_major()` raises if the export
     split the head into per-branch tensors.
  2. Channels 0..3 are xywh CENTRE form in 640-pixel units - DFL already folded
     into the graph and already multiplied by the per-anchor stride.
  3. Channels 4..83 have ALREADY been through Sigmoid.  The decoder does not
     sigmoid again; a graph that emits logits would produce confident garbage
     at every threshold rather than an error.
  4. No NonMaxSuppression and no TopK in the graph.

(2) and (3) are checked here against real simulator output, not assumed. See
`_verify_rknn()`.

NORMALISATION - MEASURED FROM THE AGENT, NOT GUESSED
----------------------------------------------------
`fpms_rover_agent.py`, camera_loop:

    img = cv2.resize(frame, (640, 640))          # STRETCH, not a letterbox
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    outputs = rknn.inference(inputs=[np.expand_dims(img, 0)])

That is a uint8 RGB NHWC array in 0..255. There is no `/255.0` anywhere in the
agent. So the /255 MUST live in the graph:

    mean_values=[[0, 0, 0]]   std_values=[[255, 255, 255]]

which is also exactly what Ultralytics' own RKNN exporter emits, and that
exporter is what produced the model now on the Pi. Get this wrong in either
direction and there is no error - the model sees inputs 255x off, every class
probability collapses, and the rover streams video and detects nothing, which
is the failure mode NPU_SPEC.md exists to make impossible.

The stretch matters to the decoder (independent per-axis rescale, no padding
compensation) but not to this file: the graph only ever sees 640x640.

STATUS
------
Nothing here has run on an RK3588S - the board has not arrived - and this
particular script has not yet reproduced the model on the Pi. What it does do
is refuse to emit a model that fails its own checks. Lines marked MEASURE ME
are the ones a real conversion run will settle.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
import platform
import subprocess
import sys
from collections import Counter

# ---------------------------------------------------------------------------
# The contract. These are not tunables.
# ---------------------------------------------------------------------------
TARGET_PLATFORM = "rk3588"
INPUT_NAME = "images"
INPUT_SHAPE = [1, 3, 640, 640]
OUTPUT_NAME = "output0"
NUM_CH = 84          # 4 box + 80 COCO classes
NUM_ANCHORS = 8400   # 80*80 + 40*40 + 20*20

# The agent feeds uint8 0..255, so the graph does the scaling. See the header.
MEAN_VALUES = [[0, 0, 0]]
STD_VALUES = [[255, 255, 255]]

# Ops whose presence proves end2end leaked back in. Ultralytics' RKNN docs
# report end2end=True emits [1,300,6] whose top-k SEGFAULTS on the NPU - a hard
# crash of the agent process, not a bad detection.
FORBIDDEN_OPS = ("NonMaxSuppression", "TopK", "RoiAlign")


def log(msg: str) -> None:
    print(f"[rknn_convert] {msg}", flush=True)


def die(msg: str) -> "NoReturn":  # noqa: F821
    print(f"\n[rknn_convert] FATAL: {msg}\n", file=sys.stderr, flush=True)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Host guard
# ---------------------------------------------------------------------------
def require_x86_linux() -> None:
    """rknn-toolkit2 is x86_64 Linux only. Refuse anywhere else, loudly.

    The two packages are one letter apart in conversation and completely
    different in capability:

        rknn-toolkit2       x86_64 Linux   converts models   <- this script
        rknn-toolkit-lite2  aarch64        runs models       <- on the rover

    On the Pi this script cannot work, and the failure you would get is an
    ImportError deep in a venv build, which reads as "the script is broken"
    rather than "you are on the wrong machine".
    """
    machine = platform.machine().lower()
    system = platform.system()
    if system != "Linux" or machine not in ("x86_64", "amd64"):
        die(
            f"this converter runs on x86_64 Linux only (here: {system}/{machine}).\n"
            "  rknn-toolkit2 (the converter) has no aarch64 build. The rover has\n"
            "  rknn-toolkit-lite2, which can only RUN models, never build them.\n"
            "  Run this on an x86_64 Linux box (or a container on one), then copy\n"
            "  the .rknn to the rover."
        )


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def sha256_file(path: str) -> str | None:
    if not path or not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def pkg_version(mod: str, dist: str | None = None) -> str | None:
    """Version of an installed package, by distribution name then by module.

    rknn-toolkit2 does not reliably expose `rknn.__version__` - it announces
    itself in a log line at import - so the distribution metadata is the honest
    source. This value goes in the sidecar and is the thing you compare against
    the rover's runtime when a model refuses to init_runtime().
    """
    try:
        from importlib.metadata import version as _dist_version
        return _dist_version(dist or mod)
    except Exception:
        pass
    try:
        return getattr(__import__(mod), "__version__", None)
    except Exception:
        return None


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def git_rev() -> str | None:
    """Which revision of this pipeline produced the model. Best effort."""
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            capture_output=True, text=True, timeout=10)
        return out.stdout.strip() or None
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Stage 1 - read the ONNX and refuse anything the decoder cannot consume
# ---------------------------------------------------------------------------
def inspect_onnx(path: str, strict: bool = True) -> dict:
    """Return facts about the ONNX, and refuse the ones that break the rover.

    Done BEFORE conversion on purpose: an eight-minute build that produces a
    model the decoder cannot read is eight minutes and a wrong artefact.
    """
    try:
        import onnx
    except ImportError:
        die("onnx is not importable. Run this inside the venv that "
            "convert_yolo26.sh builds (it pins onnx==1.14.1).")

    # onnx==1.14.1 is pinned because rknn-toolkit2 calls onnx.mapping, which
    # was removed in onnx 1.16:
    #   AttributeError: module 'onnx' has no attribute 'mapping'
    if not hasattr(onnx, "mapping"):
        die(f"onnx {onnx.__version__} has no `mapping` attribute. rknn-toolkit2 "
            "calls it and will fail mid-build. Pin onnx==1.14.1.")

    model = onnx.load(path)
    graph = model.graph

    opset = None
    for imp in model.opset_import:
        if imp.domain in ("", "ai.onnx"):
            opset = imp.version

    initialisers = {i.name for i in graph.initializer}

    def shape_of(vi):
        dims = []
        for d in vi.type.tensor_type.shape.dim:
            dims.append(d.dim_value if d.HasField("dim_value") else (d.dim_param or "?"))
        return dims

    inputs = [(vi.name, shape_of(vi)) for vi in graph.input if vi.name not in initialisers]
    outputs = [(vi.name, shape_of(vi)) for vi in graph.output]
    ops = Counter(n.op_type for n in graph.node)

    log(f"onnx opset {opset}, {sum(ops.values())} nodes")
    for n, s in inputs:
        log(f"  input   {n}: {s}")
    for n, s in outputs:
        log(f"  output  {n}: {s}")

    problems = []

    # -- end2end enforcement -------------------------------------------------
    # We enforce this by reading the GRAPH, not by trusting that the exporter
    # honoured an `end2end=False` kwarg. Ultralytics has moved that flag around
    # between versions; the op histogram cannot be argued with.
    present = [op for op in FORBIDDEN_OPS if ops.get(op)]
    if present:
        problems.append(
            f"graph contains {present} - this is an end2end export.\n"
            "    Ultralytics' RKNN docs report end2end=True emits [1,300,6] whose\n"
            "    top-k op SEGFAULTS on the RK3588 NPU. Re-export with end2end=False."
        )

    # -- one output tensor ---------------------------------------------------
    if len(outputs) != 1:
        problems.append(
            f"{len(outputs)} output tensors {[n for n, _ in outputs]}; the decoder\n"
            "    (fpms_yolo26_npu._as_anchor_major) requires exactly one. A split\n"
            "    per-branch head needs a different decoder, not a different flag."
        )
    else:
        name, shape = outputs[0]
        if name != OUTPUT_NAME:
            # Not fatal - RKNN addresses outputs by index and the decoder takes
            # outputs[0] - but it means the export differs from the one the
            # deployed model came from, so say so.
            log(f"  NOTE: output is named '{name}', not '{OUTPUT_NAME}'. "
                "Harmless (the decoder indexes, it does not look up by name) "
                "but it means this export is not identical to the recorded one.")
        numeric = [d for d in shape if isinstance(d, int)]
        if NUM_CH not in numeric or NUM_ANCHORS not in numeric:
            problems.append(
                f"output shape {shape} does not contain both {NUM_CH} (4 box + 80\n"
                f"    class) and {NUM_ANCHORS} anchors. The decoder locates the 84 axis\n"
                "    and will raise on anything else."
            )

    # -- input ---------------------------------------------------------------
    if len(inputs) != 1:
        problems.append(f"{len(inputs)} inputs; expected exactly one ({INPUT_NAME}).")
    else:
        name, shape = inputs[0]
        if any(not isinstance(d, int) for d in shape):
            problems.append(
                f"input '{name}' has a dynamic shape {shape}. Export with "
                "dynamic=False, batch=1, imgsz=640 - the NPU needs a static shape."
            )
        elif list(shape) != INPUT_SHAPE:
            problems.append(f"input '{name}' is {shape}, expected {INPUT_SHAPE}.")

    # -- DFL folding: informational, deliberately NOT a gate -----------------
    #
    # HANDOFF.md records two Softmax ops in model.23 in the export that produced
    # the deployed model - that is the folded DFL. Absence is suspicious but a
    # future Ultralytics could fold it a different way, and refusing on op
    # names would be a false gate. The real check is the value range of the box
    # channels in _verify_rknn(): 84 channels rather than 4*16+80=144 already
    # implies the DFL bins were reduced inside the graph.
    softmax = ops.get("Softmax", 0)
    log(f"  Softmax x{softmax} "
        f"({'consistent with folded DFL' if softmax else 'NONE - suspicious, see _verify_rknn'})"
        )
    sigmoid = ops.get("Sigmoid", 0)
    log(f"  Sigmoid x{sigmoid} (class channels must already be probabilities)")
    if sigmoid == 0:
        log("  WARNING: no Sigmoid in the graph. If the class channels are logits, "
            "the decoder will threshold raw scores. The runtime range check below "
            "is the gate that catches this.")

    if problems:
        msg = "the ONNX does not satisfy the decoder contract:\n\n  - " + \
              "\n  - ".join(problems)
        if strict:
            die(msg)
        log("IGNORING (--no-strict-onnx): " + msg)

    return {
        "path": os.path.abspath(path),
        "sha256": sha256_file(path),
        "size_bytes": os.path.getsize(path),
        "opset": opset,
        "inputs": [{"name": n, "shape": s} for n, s in inputs],
        "outputs": [{"name": n, "shape": s} for n, s in outputs],
        "op_histogram": dict(sorted(ops.items(), key=lambda kv: -kv[1])),
        "has_nms_or_topk": bool(present),
    }


# ---------------------------------------------------------------------------
# Stage 2 - convert
# ---------------------------------------------------------------------------
def build_rknn(onnx_path: str, out_path: str, quantize: str,
               dataset: str | None, quant_algorithm: str) -> None:
    """config -> load_onnx -> build -> export_rknn.

    Deliberately NOT set here:

      single_core_mode
          Baking single-core into the model would take the core decision away
          from runtime, and NPU_SPEC.md is explicit that init_runtime() with no
          core_mask already wastes two of three cores. Core selection belongs to
          the daemon (agent 2) and the tuner (agent 4) via
          RKNNLite.NPU_CORE_0_1_2, not to the file on disk.

      optimization_level
          Left at the toolkit default. Lowering it is a debugging step for a
          model that produces wrong numbers, not a shipping setting.

      dynamic_input
          The NPU wants a static shape and the agent always feeds 640x640.
    """
    try:
        from rknn.api import RKNN
    except ImportError:
        die("cannot import rknn.api. rknn-toolkit2 is not installed in this\n"
            "  interpreter. Use convert_yolo26.sh, which builds the venv with the\n"
            "  pinned versions - and check you are on x86_64 Linux.")

    do_quant = (quantize == "int8")
    if do_quant and not dataset:
        die("--quantize int8 requires --dataset pointing at a calibration list.\n"
            "  A quantised model built without calibration data is not 'slightly\n"
            "  worse', it is arbitrary. See README.md, 'If you quantise'.")
    if dataset and not os.path.isfile(dataset):
        die(f"calibration dataset list not found: {dataset}")

    rknn = RKNN(verbose=False)

    log(f"config: target={TARGET_PLATFORM} mean={MEAN_VALUES} std={STD_VALUES}")
    cfg = dict(
        mean_values=MEAN_VALUES,
        std_values=STD_VALUES,
        target_platform=TARGET_PLATFORM,
        # The agent feeds RGB (it does an explicit cv2.COLOR_BGR2RGB before
        # inference). quant_img_RGB2BGR=False keeps the toolkit's calibration
        # loader on RGB too, so calibration statistics match what the rover
        # actually presents. Flipping this silently mis-calibrates the R and B
        # channels - it does not error.
        quant_img_RGB2BGR=False,
    )
    if do_quant:
        # asymmetric_quantized-8 is the RK3588 INT8 mode. quantized_algorithm
        # 'normal' is min/max; 'mmse' searches for a lower-error clip range and
        # is much slower to build. Neither has been evaluated on this model.
        cfg["quantized_dtype"] = "asymmetric_quantized-8"
        cfg["quantized_algorithm"] = quant_algorithm

    try:
        ret = rknn.config(**cfg)
    except TypeError as e:
        die(f"rknn.config() rejected an argument: {e}\n"
            "  The toolkit API changed between versions. Check the installed\n"
            "  rknn-toolkit2 version against the one the rover's librknnrt.so and\n"
            "  rknn-toolkit-lite2 came from (config/fpms-os.conf: "
            "RKNN_TOOLKIT2_TAG).")
    if ret != 0:
        die(f"rknn.config() returned {ret}")

    log(f"load_onnx: {onnx_path}")
    if rknn.load_onnx(model=onnx_path) != 0:
        die("load_onnx failed. The toolkit prints the reason above this line.")

    if do_quant:
        log(f"build: INT8 (asymmetric_quantized-8, {quant_algorithm}), "
            f"calibration list {dataset}")
    else:
        log("build: FP16 (do_quantization=False) - the RK3588 NPU's native float")
    if rknn.build(do_quantization=do_quant, dataset=dataset) != 0:
        die("rknn.build() failed.")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    if rknn.export_rknn(out_path) != 0:
        die(f"export_rknn({out_path}) failed.")

    rknn.release()
    log(f"wrote {out_path} ({os.path.getsize(out_path)} bytes)")


# ---------------------------------------------------------------------------
# Stage 3 - verify the artefact, not the process
# ---------------------------------------------------------------------------
def verify_rknn(path: str) -> dict:
    """Re-load the produced .rknn and prove the decoder contract on real output.

    This runs on the toolkit's x86 SIMULATOR, which is NOT bit-identical to the
    NPU. That is fine for what is being checked here - tensor shape, and whether
    the class channels are probabilities or logits are structural properties of
    the graph, not of the silicon. It is NOT an accuracy check and must not be
    reported as one.

    Returns a dict for the sidecar. Raises nothing - the caller decides.
    """
    import numpy as np
    from rknn.api import RKNN

    result: dict = {"method": "rknn-toolkit2 x86 simulator", "status": "FAIL"}

    rknn = RKNN(verbose=False)
    if rknn.load_rknn(path) != 0:
        result["error"] = "load_rknn failed on the file we just wrote"
        return result

    if rknn.init_runtime() != 0:
        result["error"] = (
            "init_runtime() failed on the simulator. Without it the output shape "
            "cannot be observed, only assumed - and assuming is how a wrong-shaped "
            "model reaches the rover."
        )
        rknn.release()
        return result

    # Feed it EXACTLY the way fpms_rover_agent.py does: uint8 NHWC, 0..255, RGB,
    # no data_format kwarg. If that call shape is wrong, it is wrong on the
    # rover too, and this is where we want to find out.
    rng = np.random.default_rng(20260808)   # seeded: reruns are comparable
    img = rng.integers(0, 256, size=(640, 640, 3), dtype=np.uint8)
    try:
        outputs = rknn.inference(inputs=[np.expand_dims(img, 0)])
    except Exception as e:                      # noqa: BLE001
        result["error"] = f"inference raised {type(e).__name__}: {e}"
        rknn.release()
        return result
    finally:
        pass

    if not outputs:
        result["error"] = "inference returned no outputs"
        rknn.release()
        return result

    result["num_outputs"] = len(outputs)
    result["raw_shape"] = list(np.asarray(outputs[0]).shape)

    if len(outputs) != 1:
        result["error"] = (
            f"{len(outputs)} output tensors; the decoder takes outputs[0] and "
            "requires the whole head in one tensor")
        rknn.release()
        return result

    a = np.squeeze(np.asarray(outputs[0]))
    if a.ndim != 2:
        result["error"] = f"output is {a.ndim}-D after squeeze, expected 2-D"
        rknn.release()
        return result

    # RKNN is not consistent about channel-first vs channel-last across toolkit
    # versions, and the decoder handles both - but which one THIS model emits is
    # a fact worth recording rather than rediscovering on the rover.
    if a.shape[0] == NUM_CH and a.shape[1] == NUM_ANCHORS:
        result["layout"] = "channel_first"          # (84, 8400), the ONNX layout
        pred = a.T
    elif a.shape[1] == NUM_CH and a.shape[0] == NUM_ANCHORS:
        result["layout"] = "channel_last"           # (8400, 84)
        pred = a
    else:
        result["error"] = (
            f"squeezed output {a.shape} is neither ({NUM_CH},{NUM_ANCHORS}) nor "
            f"({NUM_ANCHORS},{NUM_CH}). fpms_yolo26_npu._as_anchor_major raises on this.")
        rknn.release()
        return result

    pred = np.ascontiguousarray(pred, dtype=np.float32)
    box, cls = pred[:, :4], pred[:, 4:]

    if not np.all(np.isfinite(pred)):
        result["error"] = "output contains NaN or Inf"
        rknn.release()
        return result

    # -- classes must already be probabilities -------------------------------
    # THIS is the check that protects the decoder's "do not sigmoid again"
    # contract. Logits routinely reach +-10; a sigmoided tensor cannot leave
    # [0,1]. A small epsilon absorbs fp16 rounding at the endpoints.
    cls_min, cls_max = float(cls.min()), float(cls.max())
    result["class_channel_range"] = [cls_min, cls_max]
    if cls_min < -1e-3 or cls_max > 1.0 + 1e-3:
        result["error"] = (
            f"class channels span [{cls_min:.4f}, {cls_max:.4f}], which is not a "
            "probability range. Sigmoid is NOT folded into this graph. "
            "fpms_yolo26_npu.decode() does not sigmoid - it would threshold raw "
            "logits and emit confident nonsense at every setting of CONF_THRES.")
        rknn.release()
        return result

    # -- boxes must be in 640-pixel units ------------------------------------
    # If DFL were not folded, channels 0..3 would be bin logits or raw
    # distances in stride units - single digits, not hundreds. Centres span the
    # image by construction (they are anchor centres plus a decoded offset), so
    # this holds for any input, including the noise above.
    cx_max = float(box[:, 0].max())
    cy_max = float(box[:, 1].max())
    result["box_channel_max_cx_cy"] = [cx_max, cy_max]
    # 640*1.5 allows for boxes decoded past the image edge, which is normal.
    if not (32.0 <= cx_max <= 960.0 and 32.0 <= cy_max <= 960.0):
        result["error"] = (
            f"box centre channels max out at cx={cx_max:.2f} cy={cy_max:.2f}, "
            "which is not 640-pixel units. The decoder assumes DFL is folded and "
            "the stride multiply has already happened inside the graph.")
        rknn.release()
        return result

    rknn.release()
    result["status"] = "PASS"
    result["checks"] = [
        "reload_ok",
        "single_output",
        f"shape_contains_{NUM_CH}x{NUM_ANCHORS}",
        "finite",
        "classes_in_0_1_sigmoid_folded",
        "boxes_in_640px_units_dfl_folded",
    ]
    return result


# ---------------------------------------------------------------------------
# Stage 4 - the sidecar
# ---------------------------------------------------------------------------
def write_sidecar(sidecar_path: str, model_path: str, onnx_facts: dict,
                  verification: dict, quantize: str, dataset: str | None,
                  pt_path: str | None, install_path: str) -> dict:
    """Emit one registry entry for agent 3's /etc/fpms/models.json.

    Shape: a single object under "schema": "fpms.model.v1", intended to be
    appended to the registry's `models` array verbatim. Everything
    `fpms-model-verify` could reasonably want to assert - sha256, both shapes,
    the normalisation, the decoder module, and what was actually verified
    versus assumed - is here, because the registry's whole job is that a
    wrong-shaped or wrong-version model is a declared fault rather than a
    garbage box.
    """
    entry = {
        "schema": "fpms.model.v1",
        "name": os.path.splitext(os.path.basename(model_path))[0],
        "file": os.path.basename(model_path),
        "install_path": install_path,
        "sha256": sha256_file(model_path),
        "size_bytes": os.path.getsize(model_path),

        # How the agent selects it: FPMS_YOLO_VARIANT=v26 makes
        # fpms_rover_agent.py import fpms_yolo26_npu instead of fpms_yolo_npu.
        "variant": "v26",
        "decoder_module": "fpms_yolo26_npu",
        "target_platform": TARGET_PLATFORM,

        "input": {
            "name": INPUT_NAME,
            "onnx_shape": INPUT_SHAPE,          # NCHW, as the graph declares it
            "feed_shape": [1, 640, 640, 3],     # NHWC, as the agent calls it
            "feed_dtype": "uint8",
            "colour_order": "RGB",
            "mean_values": MEAN_VALUES,
            "std_values": STD_VALUES,
            "normalisation_note": (
                "The /255 lives IN THE GRAPH. fpms_rover_agent.py feeds raw uint8 "
                "0..255 with no scaling. Changing mean/std here without changing "
                "the agent produces no error and no detections."),
            "preprocess": "cv2.resize to 640x640 - a STRETCH, not a letterbox",
        },
        "output": {
            "name": OUTPUT_NAME,
            "onnx_shape": [1, NUM_CH, NUM_ANCHORS],
            "observed_layout": verification.get("layout"),
            "channels": "0..3 xywh centre in 640px units; 4..83 class probabilities",
        },
        "graph_contract": {
            "end2end": False,
            "dfl_folded": True,
            "classes_sigmoided": True,
            "has_nms": False,
            "has_topk": False,
            "note": ("end2end=True emits [1,300,6] whose top-k SEGFAULTS on the "
                     "RK3588 NPU (Ultralytics RKNN docs). dfl_folded and "
                     "classes_sigmoided were verified against simulator output, "
                     "not assumed - see verification.checks."),
        },
        "quantization": {
            "mode": quantize,
            "dataset": os.path.abspath(dataset) if dataset else None,
            "dataset_sha256": sha256_file(dataset) if dataset else None,
        },
        "source": {
            "pt": os.path.basename(pt_path) if pt_path else None,
            "pt_sha256": sha256_file(pt_path) if pt_path else None,
            "onnx": os.path.basename(onnx_facts["path"]),
            "onnx_sha256": onnx_facts["sha256"],
            "onnx_opset": onnx_facts["opset"],
        },
        "converter": {
            "rknn_toolkit2": pkg_version("rknn", "rknn-toolkit2"),
            "onnx": pkg_version("onnx"),
            "numpy": pkg_version("numpy"),
            "ultralytics": pkg_version("ultralytics"),
            "python": platform.python_version(),
            "host": f"{platform.system()}/{platform.machine()}",
            "pipeline_git_rev": git_rev(),
            "note": ("The converter version should match the rknn-toolkit-lite2 "
                     "and librknnrt.so on the rover (config/fpms-os.conf: "
                     "RKNN_TOOLKIT2_TAG). A model built by a newer toolkit than "
                     "the runtime fails at init_runtime(), which the agent "
                     "swallows. Compare against /etc/fpms/versions.json and "
                     "/sys/kernel/debug/rknpu/version on the board."),
        },
        "verification": verification,
        "converted_utc": utc_now(),
    }

    with open(sidecar_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(entry, f, indent=2)
        f.write("\n")
    log(f"wrote {sidecar_path}")
    return entry


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description="Convert a YOLO26 ONNX to a verified RK3588 .rknn.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--onnx", required=True, help="input ONNX (from convert_yolo26.sh)")
    ap.add_argument("--out", default="yolo26n-rk3588.rknn",
                    help="output .rknn. The name is load-bearing: "
                         "fpms_yolo26_npu.MODEL hardcodes "
                         "/home/ubuntu/yolo/yolo26n-rk3588.rknn")
    ap.add_argument("--pt", default=None,
                    help="the source .pt, recorded in the sidecar for provenance")
    ap.add_argument("--quantize", choices=("fp16", "int8"), default="fp16",
                    help="fp16 (default, no calibration, matches the deployed "
                         "model) or int8 (needs --dataset; changes accuracy)")
    ap.add_argument("--dataset", default=None,
                    help="INT8 calibration list: one image path per line")
    ap.add_argument("--quant-algorithm", default="normal",
                    choices=("normal", "mmse", "kl_divergence"),
                    help="INT8 range search. Untested on this model.")
    ap.add_argument("--install-path", default="/home/ubuntu/yolo/yolo26n-rk3588.rknn",
                    help="where this lands on the rover, recorded in the sidecar")
    ap.add_argument("--no-strict-onnx", action="store_true",
                    help="downgrade ONNX contract violations to warnings. "
                         "For investigating a broken export, not for shipping.")
    ap.add_argument("--allow-unverified", action="store_true",
                    help="write the model even if verification fails - under a "
                         ".UNVERIFIED.rknn name so it cannot be deployed by accident")
    args = ap.parse_args()

    require_x86_linux()

    if not os.path.isfile(args.onnx):
        die(f"ONNX not found: {args.onnx}")

    log("--- stage 1: inspect the ONNX")
    onnx_facts = inspect_onnx(args.onnx, strict=not args.no_strict_onnx)

    log("--- stage 2: convert")
    tmp_out = args.out + ".tmp"
    build_rknn(args.onnx, tmp_out, args.quantize, args.dataset, args.quant_algorithm)

    log("--- stage 3: verify the artefact")
    verification = verify_rknn(tmp_out)
    for k, v in verification.items():
        log(f"  {k}: {v}")

    if verification.get("status") != "PASS":
        if not args.allow_unverified:
            os.remove(tmp_out)
            die("verification FAILED and the model has been DELETED.\n\n"
                f"  {verification.get('error', 'no detail')}\n\n"
                "  Refusing to emit it. A model that loads and produces the wrong\n"
                "  tensor is worse than no model: fpms_rover_agent.py catches the\n"
                "  decode exception, logs one line, and streams video forever with\n"
                "  every unit reporting active.\n"
                "  Re-run with --allow-unverified only to keep the artefact for\n"
                "  debugging; it will be named .UNVERIFIED.rknn.")
        final = args.out.replace(".rknn", ".UNVERIFIED.rknn")
        os.replace(tmp_out, final)
        log(f"WROTE UNVERIFIED MODEL: {final} - do NOT put this on a rover.")
    else:
        final = args.out
        os.replace(tmp_out, final)

    log("--- stage 4: sidecar")
    sidecar = final + ".json"
    entry = write_sidecar(sidecar, final, onnx_facts, verification,
                          args.quantize, args.dataset, args.pt, args.install_path)

    print()
    print("=" * 72)
    print(f"  model      {final}")
    print(f"  sha256     {entry['sha256']}")
    print(f"  size       {entry['size_bytes']} bytes")
    print(f"  quantised  {args.quantize}")
    print(f"  verified   {verification.get('status')} "
          f"(layout {verification.get('layout')})")
    print(f"  sidecar    {sidecar}")
    print("=" * 72)
    return 0 if verification.get("status") == "PASS" else 2


if __name__ == "__main__":
    sys.exit(main())
