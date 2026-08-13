# Rebuilding `yolo26n-rk3588.rknn`

**The model this rover detects fire with is not in this repository, and until
now nothing could regenerate it.**

`cloud/dashboard/HANDOFF.md` records it as
`cloud/dashboard/models/yolo26n_rknn_model/yolo26n-rk3588.rknn`, 7,445,558
bytes. That path does not exist in a fresh checkout — `cloud/.gitignore`
excludes `dashboard/models/*_rknn_model/`, `*.onnx` and `*.pt`, with the
comment that the derived files need "a real re-conversion, not a download". So
the only copy is on the old Pi, in `/home/ubuntu/yolo/`, on one SD card.

That is the same shape of failure `PI_FILE_INVENTORY.md` records for
`fpms-wait-net`, `fpms-uros-release-reset` and `fpms-wifi-ps-hold` — files the
whole stack depends on, present on exactly one machine, in no repo. FPMS-OS
exists to close that class of hole. This directory closes it for the model.

Prose describing a model is not a model. These three files are.

```
convert_yolo26.sh    operator entry point: host check, venv, .pt -> ONNX
rknn_convert.py      ONNX -> .rknn, verification, sidecar JSON
README.md            this file
```

---

## Run it

**On x86_64 Linux. Never on the Pi.**

```sh
cd cloud/dashboard/rover/fpms-os/npu/convert
./convert_yolo26.sh
```

Roughly 10–20 minutes, most of it `pip install torch`. Output lands in
`${TMPDIR:-/tmp}/fpms-yolo26-convert/` (override with `--workdir`), deliberately
outside the repo: the venv is a couple of gigabytes and this tree gets copied
verbatim into the image.

| flag | |
|---|---|
| `--onnx FILE` | skip the Ultralytics export, convert an ONNX you already have |
| `--quantize int8 --dataset LIST` | INT8 instead of FP16 — read §"If you quantise" first |
| `--workdir DIR` | where the venv and artefacts go |
| `--allow-unverified` | keep a model that failed verification, named `.UNVERIFIED.rknn` |

### Why it refuses to run on aarch64

Two packages, one letter apart in conversation, completely different in what
they can do:

| | arch | can |
|---|---|---|
| `rknn-toolkit2` | **x86_64 Linux only** | convert models |
| `rknn-toolkit-lite2` | aarch64 | *run* models, nothing else |

There is no aarch64 build of the converter and there never has been. The Pi
physically cannot build this model. `convert_yolo26.sh` checks `uname -m`
before it does anything else, and `rknn_convert.py` checks again in
`require_x86_linux()`, because the failure you get otherwise is a pip
resolution error deep inside a venv, which reads as "the script is broken"
rather than "you are on the wrong machine".

No x86 Linux box to hand? A container on one works — the script prints a
`docker run` line when it refuses.

---

## The pins, and the failure each one prevents

| pin | prevents |
|---|---|
| **`onnx==1.14.1`** | `rknn-toolkit2` calls `onnx.mapping`, **removed in onnx 1.16**. Symptom: `AttributeError: module 'onnx' has no attribute 'mapping'`, thrown from inside the toolkit several minutes into a build. This is the pin that cost the original conversion attempt its first afternoon (`HANDOFF.md`). |
| **`numpy<2`** | `rknn-toolkit2` is built against the NumPy 1.x C API. It is also the rule everywhere else in this project: ROS Humble's C extensions are 1.x-ABI, and a stray user-local 2.x on the old Pi broke `tf_transformations` with `np.maximum_sctype was removed`. |
| **`rknn-toolkit2` == the rover's runtime version** | read from `config/fpms-os.conf` (`RKNN_TOOLKIT2_TAG`, currently `v2.3.0`) rather than typed here, so the converter tracks the `librknnrt.so` and `rknn-toolkit-lite2` the image installs. A model built by a newer toolkit than the runtime fails at `init_runtime()` — and the agent **swallows that**, logging `NPU unavailable (...); streaming without detection` once and running forever with every unit `active`. |
| **Python 3.8–3.12** | the range Rockchip publishes wheels for. 3.10 preferred, matching the rover. |
| **`ultralytics` unpinned** | *deliberate, and a gap.* YOLO26 support is recent and moving, and nobody wrote down which version produced the model on the Pi. The script therefore records the **resolved** version in `requirements-resolved.txt` and in the sidecar JSON, so the next conversion can be pinned to whatever actually worked. **`MEASURE ME`: pin this once a conversion has been validated on the board.** |

Install order in the script matters: `ultralytics` first (it drags in its own
recent `numpy`/`onnx`), the pins last so they win.

---

## The graph contract — what the decoder depends on

`cloud/dashboard/rover/fpms_yolo26_npu.py` is the decoder. It was written
against the ONNX the deployed model came from, and it makes four assumptions.
**Conversion is responsible for preserving all four**, so this pipeline checks
each one rather than trusting a flag.

### 1. `end2end=false`, mandatory

Ultralytics' RKNN docs report `end2end=True` emits `[1,300,6]` whose top-k op
**segfaults on the NPU** — the agent process dies, it does not degrade to a bad
detection.

Enforced in two places. `convert_yolo26.sh` passes `end2end=False` (and
`nms=False`, and clears the model attribute where it exists) — but Ultralytics
has moved this flag between a kwarg, a different kwarg, and a model attribute
across versions, and **a kwarg a version silently ignores is not a guarantee**.
So `rknn_convert.py::inspect_onnx()` reads the op histogram and refuses any
graph containing `NonMaxSuppression`, `TopK` or `RoiAlign`. The histogram
cannot be argued with.

### 2. One output tensor, `[1, 84, 8400]`

84 = 4 box + 80 COCO classes. 8400 = 80×80 + 40×40 + 20×20 anchors.
`_as_anchor_major()` in the decoder raises if the head was split into per-branch
tensors — that needs a different decoder, not a different flag.

Note that "NMS-free" describes how YOLO26 was **trained**, not the tensor you
get back. With `end2end=false` the head still emits all 8400 anchors, and
thresholding plus a light NMS still run on the CPU.

### 3. DFL already folded in — box channels are 640-pixel units

Channels 0..3 arrive as xywh **centre** form already multiplied by the
per-anchor stride (`[8]*6400 + [16]*1600 + [32]*400`). The decoder does no DFL
reconstruction.

Verified by running the produced model and checking that the centre channels
max out in the hundreds, not single digits. (A graph with 84 channels rather
than 4×16+80 = 144 already implies the DFL bins were reduced inside the graph;
the range check is the confirmation.)

### 4. Classes already sigmoided — **do not sigmoid again**

Channels 4..83 are probabilities when they arrive. The decoder thresholds them
directly.

This is the assumption with the nastiest failure mode: a graph emitting logits
produces *no error at all*. `decode()` would threshold raw scores and emit
confident nonsense at every setting of `CONF_THRES`. So `verify_rknn()` runs
the model and refuses if the class channels leave `[0, 1]` — logits routinely
reach ±10; a sigmoided tensor cannot.

> **This is a contract, not a preference.** If you change the export such that
> the sigmoid or the DFL leaves the graph, `fpms_yolo26_npu.py` must change in
> the same commit. The verification here will stop you shipping the mismatch,
> but only if you run it.

---

## Normalisation — measured from the agent, not chosen

```python
rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], ...)
```

**Checked against what the rover actually feeds the model.**
`fpms_rover_agent.py`, `camera_loop()`:

```python
img = cv2.resize(frame, (640, 640))          # a STRETCH, not a letterbox
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
outputs = rknn.inference(inputs=[np.expand_dims(img, 0)])
```

That is a **uint8 RGB NHWC array in 0..255**. There is no `/255.0` anywhere in
the agent, and no `astype(np.float32)`. So the scaling has to live in the
graph, which is what `std_values=[[255,255,255]]` does — and it is also exactly
what Ultralytics' own RKNN exporter emits, which matters because that exporter
is what produced the model now on the Pi (the filename `yolo26n-rk3588.rknn` is
its `<stem>-<name>.rknn` convention).

Get this wrong in either direction and **there is no error**: the model sees
inputs 255× off, every class probability collapses below threshold, and the
rover streams perfectly good video while detecting nothing.

`quant_img_RGB2BGR=False` for the same reason — the agent hands over RGB, so
INT8 calibration statistics must be gathered on RGB too. Flipping it
mis-calibrates the R and B channels silently.

**The stretch does not affect this file.** The graph only ever sees 640×640.
The stretch matters to the *decoder*, which rescales with an independent per-axis
ratio and no padding compensation. If preprocessing ever becomes a letterbox,
`fpms_yolo26_npu.py` needs padding compensation — the converter does not change.

---

## Quantisation: FP16 by default

`rknn_convert.py` defaults to `--quantize fp16`, i.e. `build(do_quantization=False)`.
FP16 is the RK3588 NPU's native float, so this is not a fallback.

**Why FP16 is the default:**

1. **It is what is deployed.** Ultralytics' RKNN exporter calls
   `rknn.build(do_quantization=False)` unconditionally — quantisation is a
   `TODO` in that exporter — and that exporter produced the model on the Pi.
   The recorded size, 7,445,558 bytes, is consistent with ~2 bytes per weight
   for a yolo26n and roughly twice what INT8 would produce. **A rebuild's first
   job is to reproduce what works**, not to improve it.
2. **INT8 needs a calibration dataset that does not exist in this repo.** There
   are no fire images, no rover-camera images, nothing.
3. **The decoder's thresholds were tuned against the FP16 model.**
   `fpms_yolo26_npu.py` ships `CONF_THRES = 0.35`, `IOU_THRES = 0.55`. INT8
   shifts the score distribution; those numbers would need re-deriving, and
   nobody can do that without the board and real footage.
4. **The NPU budget is not the constraint.** `FPMS_INFER_EVERY=2` at
   `FPMS_CAMERA_FPS=6` is roughly **3 inferences per second** — the camera caps
   at 30 fps in hardware and the FPS is held at 6 on purpose because higher
   settings saturated WiFi and starved the LiDAR. Trading accuracy for latency
   we are not short of is a bad trade.

**What INT8 would buy:** roughly half the model size, less NPU memory
bandwidth, and lower per-inference latency — the RK3588 NPU's 6 TOPS figure is
an INT8 number. Worth revisiting **only** if benchmarking on the actual board
shows inference latency is a real constraint, and only with a dataset. Do not
quantise speculatively.

### If you quantise

```sh
./convert_yolo26.sh --quantize int8 --dataset /path/to/calib.txt
```

`calib.txt` is one image path per line. Requirements, in order of how quietly
they hurt you:

- **Representative of what the rover sees.** Frames from *this* camera, at the
  rover's mounting height, including flame and smoke. COCO val images calibrate
  for the wrong distribution and the damage shows up as missed fires, not as an
  error.
- **Pre-resize to 640×640 with the same `cv2.resize` stretch the agent uses**,
  so calibration sees the aspect distortion the model will actually be fed.
- **RGB.** Left as `quant_img_RGB2BGR=False`; see above.
- 100–500 images is the usual guidance. `MEASURE ME` — untested here.
- Re-tune `CONF_THRES`/`IOU_THRES` in `fpms_yolo26_npu.py` afterwards, against
  real footage, and say in the commit that you did.

`--quant-algorithm mmse` searches for a lower-error clip range and builds much
more slowly. Neither algorithm has been evaluated on this model.

**A quantised model built without a calibration dataset is not "slightly
worse", it is arbitrary.** `rknn_convert.py` refuses `--quantize int8` without
`--dataset`.

---

## What "verified" means here, and what it does not

After `export_rknn()`, `rknn_convert.py` **re-loads the file it just wrote** and
runs it on the toolkit's x86 simulator with a seeded random uint8 image, fed
exactly the way the agent feeds it. It refuses to emit the model unless all of:

```
reload_ok                        the file loads as an RKNN model
single_output                    one tensor, not a split head
shape_contains_84x8400           either layout; which one is recorded
finite                           no NaN, no Inf
classes_in_0_1_sigmoid_folded    class channels are probabilities
boxes_in_640px_units_dfl_folded  centre channels are pixels, not bins
```

Failure **deletes the artefact** and exits non-zero. `--allow-unverified` keeps
it, but renames it `*.UNVERIFIED.rknn` so it cannot reach a rover by a careless
`scp`. That asymmetry is on purpose: a model that loads and returns the wrong
tensor is worse than no model, because `fpms_rover_agent.py` catches the decode
exception, logs one line, and streams video forever with every unit reporting
`active`.

**What this does not prove:**

- **Not accuracy.** The simulator is not bit-identical to the NPU, and a
  random-noise input says nothing about detection quality. Nobody should read
  `verified: PASS` as "it detects fire".
- **Not runtime compatibility.** Whether `librknnrt.so` and the kernel `rknpu`
  driver on the board accept this model is a separate question, answerable only
  on the board:

  ```sh
  cat /sys/kernel/debug/rknpu/version
  cat /etc/fpms/versions.json
  fpms-model-verify
  ```

- **Nothing here has run on an RK3588S.** The board has not arrived. This
  pipeline has not yet reproduced the model that is on the old Pi, and until it
  has, "the model is rebuildable" is a claim, not a measurement. The way to
  settle it is to run this and compare against the Pi's copy:

  ```sh
  ssh ubuntu@fpms-rover1.local sha256sum /home/ubuntu/yolo/yolo26n-rk3588.rknn
  ```

  A byte-identical result is not expected (build timestamps, toolkit
  non-determinism); matching output shape, layout and detections on the same
  frame is the standard that matters.

---

## The sidecar, and the model registry

Every successful conversion writes `<model>.rknn.json` next to the model — one
object, `"schema": "fpms.model.v1"`, shaped to be dropped straight into the
`models` array of `/etc/fpms/models.json` (agent 3's registry). It carries:

- `sha256`, `size_bytes`, `install_path`
- `input`: both shapes (`[1,3,640,640]` NCHW as the graph declares, `[1,640,640,3]`
  NHWC as the agent calls it), dtype, colour order, `mean_values`/`std_values`,
  and the preprocessing note
- `output`: declared shape and the layout actually observed
- `graph_contract`: `end2end`, `dfl_folded`, `classes_sigmoided`, `has_nms`,
  `has_topk`
- `quantization`: mode, dataset path and dataset hash
- `source`: `.pt` and `.onnx` hashes and the ONNX opset
- `converter`: resolved `rknn-toolkit2`, `onnx`, `numpy`, `ultralytics`, Python,
  host, and the git rev of this pipeline
- `verification`: the checks that ran and what they observed
- `variant: "v26"` and `decoder_module: "fpms_yolo26_npu"` — which decoder this
  model requires

That last pair is the point. `fpms_rover_agent.py` picks its decoder from
`FPMS_YOLO_VARIANT` alone: `v26` imports `fpms_yolo26_npu`, anything else
imports `fpms_yolo_npu`. **Nothing checks that the loaded `.rknn` matches the
selected decoder.** Pointing the v8 decoder at this model would double-apply
DFL and sigmoid and produce boxes that are wrong rather than absent. The sidecar
is what lets `fpms-model-verify` catch that.

**Commit the sidecar. Do not commit the `.rknn`** — it is gitignored, and a
7 MB binary blob in git history is not the fix for this problem. The sidecar
plus this pipeline is: it says exactly which model a rover carries, and the
pipeline regenerates it.

---

## Getting it onto the rover

```sh
scp yolo26n-rk3588.rknn      ubuntu@fpms-rover1.local:/home/ubuntu/yolo/
scp yolo26n-rk3588.rknn.json ubuntu@fpms-rover1.local:/home/ubuntu/yolo/
```

**Do not rename the file.** `fpms_yolo26_npu.py` hardcodes
`MODEL = "/home/ubuntu/yolo/yolo26n-rk3588.rknn"` with no override. Use the
hostname, never an IP — this rover's address has changed more than seven times
in this project and every note that wrote one down was wrong the next day.

Then, on the rover:

```sh
sudo fpms-model-verify --register /home/ubuntu/yolo/yolo26n-rk3588.rknn.json
fpms-model-verify

# /etc/fpms/config.env
FPMS_YOLO_VARIANT=v26

sudo systemctl restart fpms-rover-agent
journalctl -u fpms-rover-agent -f | grep -i npu
```

The line you need is:

```
NPU ready with /home/ubuntu/yolo/yolo26n-rk3588.rknn (variant v26)
```

The line that means the rover is blind is:

```
NPU unavailable (...); streaming without detection
```

It is logged **once**, and then the agent runs forever with `systemctl` reporting
`active`, the camera streaming, and the dashboard looking alive. That is the
single worst failure mode in this project, because it is indistinguishable from
success at every level an operator normally checks. Look for the first line.
Do not assume it because the video came up.

---

## Relationship to `cloud/dashboard/models/convert_yolo26.sh`

That file is the original Docker scratch script and everything it established is
carried forward here: the `onnx==1.14.1` and `numpy<2` pins, `end2end` off, and
that this is an x86-only job. What it did not do — and what made the model
unrebuildable in practice — is check the host architecture, verify the artefact,
record what produced it, or say where the output goes. It ran Ultralytics'
one-shot RKNN export, printed `EXPORTED:`, and stopped.

Treat this directory as the successor. If the old script is ever deleted,
nothing is lost.
