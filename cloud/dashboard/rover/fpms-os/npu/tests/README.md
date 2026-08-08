# NPU decode tests

Offline correctness checks for the YOLO26 decode path. No Pi, no NPU, no
`.rknn`, no `rknnlite`, no `pytest`. numpy only.

```sh
python3 npu/tests/test_decode.py      # exit 0 = pass, 1 = fail, 2 = could not load the module
```

95 assertions across 19 test groups against
`cloud/dashboard/rover/fpms_yolo26_npu.py`. Run it before every change to that
file and before every model re-export.

---

## Why a decode bug is worse than a crash here

A crash is cheap. `fpms_rover_agent.py` wraps the inference block in a
try/except, logs `inference failed`, and the dashboard shows no boxes. Somebody
notices within a minute.

A decode bug costs everything, because **it produces plausible-looking boxes
rather than an error.** The rectangles are the right count, the right dtype,
the right colour, drawn on the right video. They are simply in the wrong place,
or at the wrong threshold, or labelled with the wrong class. Every layer above
the decode — the drawn frame, the MQTT detection record, the dashboard, the
operator — reports success.

`NPU_SPEC.md` names this as the shape of the worst failure in the project: a
rover that streams video and detects nothing is *"indistinguishable from
success at every level an operator normally checks."* A rover that detects the
**wrong thing** is worse still, because it also passes the check where a human
watches the video and says "yes, there are boxes."

That is why this file is not organised around coverage. It is organised around
the specific ways a correct-looking decode can be silently wrong.

Every check names the real failure it guards against. **If a check fails, do
not edit the check.** Read its docstring first — each one exists because the
alternative behaviour is invisible from the outside.

---

## What is protected

### 1. The output shape contract — `[1, 84, 8400]`

4 box coordinates + 80 classes over 80×80 + 40×40 + 20×20 anchors. Tensors that
are not this must **raise**, not decode. A permissive decoder is how a
wrong-version model "loads and produces garbage boxes."

Refusals pinned: an 85-channel v8 export, a v5 `[1,25200,85]`, a 64-channel
model, a split-head export, a flat 1-D tensor.

### 2. `[1,300,6]` from `end2end=True` is refused, not misparsed

The single most dangerous plausible-looking tensor in the stack.
`end2end=True` emits 300 rows of `x1,y1,x2,y2,score,class` — **already
decoded, already NMS'd, and in corner format, not centre format.** If this
decoder ever accepted it, all 300 rows would be re-read as `xywh` centres and
re-scaled, producing 300 boxes of the right count and dtype and complete
geometric nonsense.

`NPU_SPEC.md`: *"`end2end=false` is mandatory. Ultralytics' RKNN docs report
`end2end=True` emits `[1,300,6]` whose top-k op segfaults on the NPU."*

### 3. No double sigmoid — the highest-stakes invariant

Channels 4..83 arrive **already sigmoided**; the DFL reconstruction is already
folded into the graph by the two Softmax ops in `model.23`. Sigmoiding again
does not error. It maps [0,1] onto [0.5, 0.731]:

| stored | double-sigmoided | effect against `CONF_THRES=0.35` |
|---|---|---|
| 0.90 (a real detection) | 0.71 | still reported, just weaker — no alarm |
| 0.00 (a real *non*-detection) | **0.50** | **now above threshold** |

So the symptom is not "detections vanish", which somebody would investigate. It
is *"everything is a detection at middling confidence"* — and after NMS
collapses them, a handful of confident-looking phantom boxes on a
fire-detection rover. A false-alarm generator that never crashes.

Verified against a double-sigmoid mutant: an all-zero output tensor produced
**50 phantom detections, every one at exactly 0.500**. Four independent
discriminators cover this: the all-zero tensor, exact score passthrough, a
raw-logit value that must not be squashed, and a just-below-threshold value
that a double sigmoid can never drop.

### 4. Stretch rescaling, not letterbox — the highest-value geometry test

`fpms_rover_agent.py` line 601 is literally `cv2.resize(frame, (640, 640))`. A
plain stretch. No aspect preservation, no padding. The inverse is therefore an
**independent per-axis ratio**, `x * (iw/640)` and `y * (ih/640)`.

Nearly every YOLO decoder in the wild does the *letterbox* inverse instead,
because nearly every YOLO preprocessor letterboxes. Pasting one in produces
boxes that are offset and squashed — close enough that the video reads as
"working", wrong enough that the box centre is metres off in the world frame
once anything downstream uses it.

Two geometry traps this test is deliberately built to avoid:

- **At the image centre, the stretch and letterbox inverses agree exactly.** A
  centred box proves nothing. This test uses a 16:9 frame and an off-centre box
  — the only geometry that discriminates.
- **Clipping can mask the difference.** If the letterbox answer falls off the
  frame it is clipped to 0, and a naive "these differ" assertion passes for the
  wrong reason. The box position is chosen so *both* interpretations land
  inside the frame, and the letterbox reference is clipped the same way
  `decode()` clips.

Also covered: `ih`/`iw` argument-order swaps, tested in both landscape and
portrait because a square frame cannot catch them.

### 5. NMS still runs on the CPU

"NMS-free" is a **training-time** property of YOLO26. The exported graph
contains no `NonMaxSuppression` and no `TopK` (verified by op histogram) and
still emits all 8400 anchors. Thresholding and a light NMS happen in Python.

Guarded: duplicate stacks collapse; disjoint boxes survive; IoU just above /
just below / **exactly at** `IOU_THRES` (the comparison is `iou <= thres`, so
equality keeps both); containment alone does not suppress; the per-class offset
trick isolates all 80 classes including the 78/79 boundary; the offset never
leaks into the returned coordinates.

### 6. Numerical edges

All-zero, NaN in class channels, NaN in box coordinates, ±Inf, an all-NaN
tensor, a single detection (which skips NMS via a separate code path), all 8400
anchors above threshold, and class ids at 0 and 79.

Class 0 is `person` and lives in channel 4 — the first channel after the box.
A `pred[:, 5:]` slice (the v5/v8 objectness layout) shifts every class id down
by one, so person becomes bicycle and the agent's `WILDLIFE_IDS` /
`COCO_TREE_IDS` lookups all point at the wrong things. Nothing errors; the
labels are just wrong.

---

## Warnings this run currently emits

A warning is not a failure. These are properties of `fpms_yolo26_npu.py` as it
stands, recorded so they are decisions rather than accidents.

| Warning | What it means |
|---|---|
| `saturate.timing` | Worst-case NMS on a fully-saturated frame took **~0.7–1.8 s on an x86 dev box**. `MAX_DETECTIONS=50` is applied *after* the NMS loop, so it bounds the output but not the work. `decode()` runs inline in the camera thread, so this stalls the frame pump — which `NPU_SPEC.md` design rule 2 forbids. A `if len(keep) >= MAX_DETECTIONS: break` inside `_nms` would bound it. **MEASURE ME on the RK3588S.** |
| `anchors.wrong_count` | `NUM_ANCHORS` is defined and then **never used**. A 320×320 export emits `[1,84,2100]`, which is accepted, and every box comes back half-size and mispositioned. Exactly the plausible-looking failure this file exists to catch, and the decoder cannot currently see it. |
| `nan.class_eats_detection` | One NaN in an anchor's class row silently *drops* that anchor's real detection (`argmax` picks the NaN, then `NaN >= thresh` is False). Corrupt output lowers the detection rate with no error anywhere. |
| `nan.box_reaches_agent` | A NaN box coordinate is returned to the caller. The agent does `int(v)` on it; `int(nan)` raises inside the inference try/except, so the agent logs `inference failed` and keeps drawing the **previous** frame's boxes indefinitely. |
| `inf.score_reported` | An infinite confidence is returned as a detection and rounded into telemetry as `Infinity`, which is not valid JSON. |
| `nan.all_nan_silent` | An all-NaN tensor decodes to zero detections and no error — indistinguishable from a quiet forest. |

The last four all point the same way: `decode()` has no `np.isfinite()` guard.
Per design rule 3 (*"never claim a detection you cannot substantiate"*), a
non-finite tensor should probably be a named fault rather than a quiet zero.
That change belongs to whoever owns `fpms_yolo26_npu.py`; this file only
records the current behaviour so a change to it is visible.

---

## Relationship to `rover/test_yolo26_decode.py`

That file exists and holds 22 checks: the centred round-trip, both tensor
orientations, threshold, duplicate NMS, per-class NMS, clipping, dtypes. Those
are still the right checks and are **not repeated here.**

This file covers what that one does not: the double-sigmoid trap tested as a
*discriminating* case, stretch-vs-letterbox proved on a non-square frame with
an off-centre box, the `[1,300,6]` refusal, numerical edges, saturation, and
the unvalidated anchor count.

**Run both.** Neither supersedes the other.

---

## What a green run does NOT prove

This is the important section. A green run means *the decode is self-consistent
against a synthesised tensor.* It does not mean detection works.

Not testable offline, and not tested here:

- **That the real `.rknn` emits `[1,84,8400]` at all.** The model is not in the
  repository — it exists only on the old Pi. Nothing here reads it. If it was
  exported with `end2end=True`, or at 320×320, or with a different class count,
  this suite still passes and the decode still fails on hardware.
- **That DFL is folded and classes are pre-sigmoided in the shipped model.**
  That fact comes from an op histogram of `models/yolo26n.onnx`, read once by a
  human. These tests assert the decoder *behaves as if* it is true. They cannot
  confirm it *is* true for a given `.rknn`. **Re-verify on every re-export** —
  agent 5 owns the conversion pipeline and agent 3 owns model verification.
- **That the NPU runtime loads.** Driver / `librknnrt.so` /
  `rknn-toolkit-lite2` version matching fails at `init_runtime()` and is
  entirely outside this file. See `npu/versions/README.md` and
  `fpms-npu-selftest`.
- **Real inference latency, thermal behaviour, or core utilisation.** The
  timing number in `saturate.timing` is from a Windows x86 dev box under
  numpy 2.x and says nothing about an RK3588S Cortex-A76 under sustained load.
  See `npu/bench/fpms_npu_bench.py`.
- **Detection quality.** Nothing here has ever seen a real image, a real fire,
  or a real frame from the camera. Every tensor is synthesised. Boxes being
  decoded correctly is not the same as the model finding the right things.
- **The numpy version the Pi actually runs.** `NPU_SPEC.md` pins `numpy<2`
  system-wide (ROS Humble's C extensions are built against the 1.x ABI). This
  suite runs on either and prints which one it used — that note is not
  decorative, read it.

**Nothing in this directory has run on an RK3588S. The board has not arrived.**

---

## How the tests were validated

Tests that pass against broken code are worse than no tests. This suite was
checked against 14 mutants of `fpms_yolo26_npu.py` in a sandboxed copy (the
real file was never modified). Every mutant was caught; the unmutated control
passed clean.

| Mutation | Assertions that fired |
|---|---|
| double sigmoid on class channels | 35 |
| letterbox rescale instead of stretch | 6 |
| NMS deleted | 3 |
| `ih`/`iw` swapped | 9 |
| `pred[:, 5:]` class slice | 28 |
| permissive shape handling | 5 |
| `iou < thres` instead of `<=` | 1 |
| per-class NMS offset shrunk | 2 |
| `MAX_DETECTIONS` cap removed | 1 |
| frame clipping removed | 6 |
| `NUM_CLASSES` changed (admits `[1,300,6]`) | 11 |
| `>` instead of `>=` at the threshold | 1 |
| `conf_thres` kwarg ignored | 1 |
| output tensor written through to the caller | 1 |
| **control (unmutated)** | **0** |

Re-run that exercise after adding checks. A check that no mutation can trip is
decoration.
