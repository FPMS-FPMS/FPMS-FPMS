# FPMS-OS NPU layer — the contract

Eight agents build disjoint parts of this. **Where this file and your own
judgement disagree, this file wins.** Read `../SPEC.md` first for the
image-wide rules; this document only covers the NPU.

---

## 0. What we are building and why it is not what exists today

The RK3588S carries a **6 TOPS NPU with three cores**. It is what makes FPMS a
fire-detection rover rather than a camera on wheels: YOLO26 runs on it, on
device, and the whole product thesis depends on that inference being real.

What FPMS-OS ships today is one `pip install` in `scripts/20-python-deps.sh`
and a warning if it fails. That is not an NPU layer. Specifically:

| Gap | Consequence |
|---|---|
| `99-fpms-npu.rules` is named in `SPEC.md` and **was never written** | nothing grants the unprivileged `ubuntu` user access to the NPU device |
| runtime and driver versions are pinned by URL but never **checked against each other** | a mismatch fails at `init_runtime()`, which the agent swallows |
| inference runs **inside the camera loop** | a slow or wedged NPU call stalls the frame pump that also feeds the fire detector |
| `init_runtime()` is called with no core mask | **uses core 0 only — two thirds of the NPU is idle** |
| no thermal or DVFS policy | RK3588 throttles hard and silently under sustained load |
| the `.rknn` model is **not in the repository** | it exists only on the old Pi. If that card dies the rover cannot detect anything, and nothing in git can rebuild it |
| no model verification | a wrong-shaped or wrong-version model loads and produces garbage boxes |
| no bench, no health check | see below |

### The failure this layer exists to make impossible

`fpms_rover_agent.py` wraps the entire NPU block — import, `load_rknn`,
`init_runtime`, labels — in **one try/except** that logs

```
NPU unavailable (...); streaming without detection
```

once, and then carries on forever. Every unit reports `active`. The camera
streams. The dashboard looks alive. **The rover detects nothing.**

That is the single worst failure mode in the project, because it is
indistinguishable from success at every level an operator normally checks. The
NPU layer's job is to make it loud, early, and impossible to miss.

---

## 1. Hard facts — do not re-derive these

### The stack, and its version coupling

```
kernel rknpu driver   (Rockchip BSP only — NO mainline driver exists)
        ↕  MUST MATCH
/usr/lib/librknnrt.so (userspace runtime)
        ↕  MUST MATCH
rknn-toolkit-lite2    (aarch64 Python wheel, NOT on PyPI)
```

A mismatch anywhere in that chain fails at `init_runtime()` with a message the
agent swallows. **Nobody has ever written down which version the old Pi ran** —
`bench_npu.py` reads it at runtime from `/sys/kernel/debug/rknpu/version`. That
is the tell: there was no build record, so a working rover and a broken one
could not be compared.

The **converter** is a different, x86-only package (`rknn-toolkit2`). The Pi can
only *run* models, never build them.

### The model

`yolo26n-rk3588.rknn`, at `${FPMS_YOLO_DIR}/` (default `/home/ubuntu/yolo`).

- input `images` `[1,3,640,640]`
- output `output0` `[1,84,8400]`
- **`end2end=false` is mandatory.** Ultralytics' RKNN docs report `end2end=True`
  emits `[1,300,6]` whose top-k op **segfaults on the NPU**.
- **DFL is already folded into the graph and classes are already sigmoided —
  do not sigmoid again.**
- Despite "NMS-free" being a *training-time* property, the exported graph has no
  NonMaxSuppression/TopK op and still emits all 8400 anchors, so thresholding
  and a light NMS still run on the CPU.
- Preprocessing is **`cv2.resize` to 640×640 — a STRETCH, not a letterbox.** Box
  rescaling is therefore an independent per-axis (x, y) ratio, **not** a single
  uniform scale plus padding offset. If preprocessing ever becomes a letterbox,
  the decoder needs padding compensation added.

Conversion pins that matter: **`onnx==1.14.1`** (rknn-toolkit2 calls
`onnx.mapping`, removed in 1.16) and **`numpy<2`** (built against the NumPy 1.x
C API).

### Cores

Three NPU cores. `init_runtime()` with no `core_mask` **uses core 0 only.**
`RKNNLite.NPU_CORE_0_1_2` exists. There is a real decision here: single-core
per-model with three models in parallel, versus one model across all three.
Measure, do not assume.

### Camera constraints that bound the whole design

- **30 fps hardware cap** at every resolution — the sensor's USB descriptor.
- Ships at **`FPMS_CAMERA_FPS=6`, `FPMS_JPEG_QUALITY=40`, deliberately**: higher
  settings saturated WiFi and starved the LiDAR feed. Do not raise them to feed
  the NPU without re-measuring the LiDAR rate that reaches the dashboard.
- `FPMS_INFER_EVERY=2` — inference on every second frame.
- So the NPU budget is roughly **3 inferences/second**, not 30. Design for
  latency and reliability, not throughput.

### NumPy

`numpy<2`, system-wide, no user-local copy. ROS Humble's C extensions are built
against the 1.x ABI, and a user-local 2.x on the old Pi shadowed the system one
and broke `tf_transformations`. `rknn-toolkit-lite2` is also a 1.x consumer.

---

## 2. Design rules for this layer

1. **Silence is never success.** Every NPU failure must be loud, named, and on a
   topic. A rover that streams video and detects nothing must announce that
   within seconds, not never.
2. **Inference must not share a thread with the camera pump or the LiDAR path.**
   A wedged NPU call must not be able to stall telemetry.
3. **Never claim a detection you cannot substantiate.** If the model is wrong,
   the runtime is mismatched, or the output shape is unexpected, refuse and say
   so — do not emit boxes.
4. **Version-match by measurement, not by hope.** Record driver, runtime and
   wheel versions at build time; verify they agree at boot.
5. **Do not guess a model.** If the `.rknn` is absent or its checksum is
   unknown, that is a declared fault, not a silent fallback.
6. **Thermal reality is not optional.** RK3588 throttles. Measure sustained
   latency, not a first-inference number.

---

## 3. File ownership — strictly disjoint

Everything lives under `cloud/dashboard/rover/fpms-os/`. **Do not create or edit
a file outside your list.** If you need something from another area, assume it
exists at the path named here.

| # | Agent | Owns |
|---|---|---|
| 1 | Driver & runtime | `overlay/etc/udev/rules.d/99-fpms-npu.rules`, `scripts/25-npu-runtime.sh`, `npu/versions/README.md` |
| 2 | Inference daemon | `overlay/usr/local/bin/fpms-npud`, `overlay/etc/systemd/system/fpms-npud.service` |
| 3 | Model registry | `overlay/usr/local/bin/fpms-model-verify`, `overlay/etc/fpms/models.json`, `npu/models/README.md` |
| 4 | Cores & thermal | `overlay/usr/local/bin/fpms-npu-tune`, `overlay/etc/systemd/system/fpms-npu-tune.service` |
| 5 | Conversion pipeline | `npu/convert/convert_yolo26.sh`, `npu/convert/rknn_convert.py`, `npu/convert/README.md` |
| 6 | Decode correctness | `npu/tests/test_decode.py`, `npu/tests/README.md` |
| 7 | Health & benchmark | `overlay/usr/local/bin/fpms-npu-selftest`, `npu/bench/fpms_npu_bench.py` |
| 8 | Docs | `docs/NPU.md`, `npu/README.md` |

Shared contract every agent codes against:

```
/dev/rknpu                          device (verify the real node on hardware)
/sys/kernel/debug/rknpu/version     driver version
/sys/kernel/debug/rknpu/load        per-core utilisation
/usr/lib/librknnrt.so               runtime
/etc/fpms/npu-versions.json         written at build time by stage 25
/etc/fpms/models.json               the model registry
/home/ubuntu/yolo/yolo26n-rk3588.rknn
/var/lib/fpms/npu-selftest.json     selftest output
/run/fpms/npud.sock                 daemon IPC (unix socket)
MQTT  fpms/<thing>/telemetry/npu    health, per-core load, latency percentiles
MQTT  fpms/<thing>/events/npu_fault QoS 1, edge-triggered
```

Env vars (add to `config.env` **only via agent 8's doc note** — do not edit
`config.env` yourself, it is owned elsewhere):
`FPMS_NPU_CORE_MASK`, `FPMS_NPU_REQUIRED`, `FPMS_NPU_MAX_LATENCY_MS`,
`FPMS_NPU_SOCKET`, `FPMS_YOLO_DIR`, `FPMS_YOLO_VARIANT`.

---

## 4. Honesty requirements

- Nothing here has run on an RK3588S. **The board has not arrived.** Every file
  must be explicit about what is verified and what is inferred.
- Where a device path, sysfs node, or version string cannot be confirmed
  without hardware, **say so in the file** and provide the exact command that
  confirms it. Do not invent a plausible value and present it as fact.
- Match the repository's voice: comment *why*, cite the measurement, and mark
  unmeasured things `MEASURE ME`.
