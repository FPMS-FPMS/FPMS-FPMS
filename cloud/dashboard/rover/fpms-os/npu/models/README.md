# NPU models — the registry, and the one file that is not in it

This directory documents the models the rover may load. It contains **no
model**. That is the first thing to understand about it.

Owned here: `overlay/etc/fpms/models.json` (the registry) and
`overlay/usr/local/bin/fpms-model-verify` (the checker). Rebuilding a model
from ONNX is `npu/convert/`; running one is `fpms-npud`.

---

## THE MODEL IS NOT IN GIT

`yolo26n-rk3588.rknn` **exists on exactly one SD card, in one Pi.** It is not in
this repository, not in any release, not in any backup this project can point
at. Nobody has ever recorded its sha256, its size, which `rknn-toolkit2` built
it, or which `yolo26n.pt` it started life as.

`scripts/30-fpms-payload.sh` says so at build time, in a banner, deliberately:

```
 NOT IN THE REPOSITORY, AND THEREFORE NOT IN THIS IMAGE:
   ~/yolo/yolo26n-rk3588.rknn   the detection model
```

### What is lost if that card dies

The rover streams video and detects nothing — while every unit reports
`active`, the dashboard fills with frames, and the operator has no signal that
anything is wrong. `fpms_rover_agent.py` logs

```
NPU unavailable (...); streaming without detection
```

**once**, and then runs blind forever. That is the worst failure mode in the
project because it is indistinguishable from success at every level an operator
normally checks.

And unlike the three `/usr/local/bin` scripts that `PI_FILE_INVENTORY.md`
records as existing only on that Pi — which were reconstructed, painfully, from
prose and behaviour — **a model cannot be reconstructed from prose.** No amount
of documentation regenerates a set of weights. The best this repository can
offer is `npu/convert/`, which rebuilds a *different* file from a *different*
ONNX with a *different* converter version, and which nobody has yet run.

FPMS is a fire-detection rover rather than a camera on wheels because of that
one file.

### Get it off the card. Today.

From the operator laptop, with the old Pi powered up:

```bash
# 1. Copy everything in the yolo directory, not just the .rknn -- fpms_yolo_npu.py
#    (the v8 decode path) and custom_labels.json are missing from git too.
scp -r ubuntu@<old-pi>:~/yolo ./yolo-rescue/

# 2. Hash it IMMEDIATELY, before it is copied anywhere else. A hash taken after
#    a chain of copies proves the last copy, not the original.
sha256sum ./yolo-rescue/yolo26n-rk3588.rknn
ls -l   ./yolo-rescue/yolo26n-rk3588.rknn

# 3. Verify the copy against the source, over the wire, before you trust it.
#    A truncated scp produces a file that looks fine and fails at load_rknn.
ssh ubuntu@<old-pi> sha256sum '~/yolo/yolo26n-rk3588.rknn'
```

Then put it somewhere that is not one SD card. Options, in order of how much
this project would benefit:

| Where | Why |
|---|---|
| **git-lfs in this repo** | `.gitattributes` already carries `*.rknn binary`, so the repo was written expecting a model to land here one day. A nano model is small. This is the only option that makes the image self-contained. |
| A release asset / object store, with the sha256 in `models.json` | Keeps the repo light; the hash in the registry is what makes it trustworthy. |
| Two USB sticks in two places | Better than one card. Not better than either row above. |

Whichever you choose, **record the sha256 in `models.json`** (see below). A
backup nobody can prove is the right file is a backup you will not dare restore
from under time pressure.

### Also copy the ONNX

`fpms_yolo26_npu.py`'s header cites `models/yolo26n.onnx` as "the source the
`.rknn` was converted from". Whether that file still exists anywhere is
**UNKNOWN**. If it does, it is what makes `npu/convert/` a real rebuild path
rather than a hopeful one. Look for it on the old Pi and on whatever x86 box did
the conversion — the Pi can only *run* models, never build them, so the
conversion happened somewhere else.

---

## The registry: `/etc/fpms/models.json`

Every model the rover may load is described there before it is loaded: logical
name, path, sha256, expected input and output names and shapes, class count,
labels, provenance, and a `verified` flag.

### `sha256` is `null`, and that is a feature

We do not have the hash. So the registry says `null`, `sha256_status` says why,
and `verified` is `false`.

This follows `calibration.json.example`'s precedent exactly: **a missing value
is visibly missing, never quietly defaulted.** A fabricated hash would make
`fpms-model-verify` FAIL on the one genuinely correct file the project owns, and
the obvious way an operator "fixes" that is to stop checking hashes at all.

`null` is not a shrug. It is a declared, checkable state:
`fpms-model-verify` reports `UNVERIFIED`, prints the observed hash, and **exits
non-zero**.

### Adding a model

1. Copy the entry for `yolo26n` and change `logical_name`, `file`, the shapes,
   `num_classes` and `provenance`.
2. Leave `sha256: null` and `verified: false`. Do not fill them in by hand —
   `--register` exists so the recorded hash is one somebody actually observed.
3. Set `enabled: true` only for models the rover may really load. A named model
   is verified even when `enabled: false`, so keeping an entry around costs
   nothing and documents a decision.
4. Fill in what you know; mark what you do not know as `UNKNOWN` **with the
   reason**. Every `null` in the shipped file has a `_status` string next to it
   saying why. Match that.
5. Run `fpms-model-verify <name>` on hardware. It must pass before anything
   trusts it.

Documentation lives in keys beginning with `_` because JSON has no comments —
and because `--register` does a `json.load`/`json.dump` round-trip, so `_` keys
survive and real comments would not.

---

## Verifying: `fpms-model-verify`

```bash
fpms-model-verify                 # every enabled model
fpms-model-verify yolo26n         # one, by logical name
fpms-model-verify --offline       # hash + registry only; never opens the NPU
fpms-model-verify --json          # machine-readable
fpms-model-verify --list          # what the registry currently claims
sudo fpms-model-verify yolo26n --register    # record the observed hash
```

Exit: `0` verified, `1` unproven (WARN/SKIP), `2` failed.

**`1` and `0` are different on purpose.** "We could not check" must never share
an exit code with "we checked and it was fine", or every boot script and CI job
quietly accepts an unverified model.

### What it checks, and why each one is there

| Check | The failure it catches |
|---|---|
| file exists | The model is not in git. This is the common case, not the edge case. |
| sha256 vs registry | A partial `scp`, a reconversion nobody mentioned, a corrupt card. |
| end2end pre-filter (byte scan) | An `end2end=True` export, **before** the NPU is asked to run it. |
| `load_rknn` returns 0 | A truncated or wrong-format container. |
| `init_runtime` returns 0 | The driver/runtime/wheel version mismatch — the one the agent swallows. |
| **output shape is `[1,84,8400]`** | **The one that matters.** A wrong-shaped model does not error; it produces garbage boxes that look like detections. |
| outputs finite | A broken graph or quantisation. Non-finite scores make every comparison silently `False`. |
| class channels in `[0,1]` | Classes are **already sigmoided** in this graph. Out of range means the Sigmoid was dropped; all-`>= 0.5` means it was applied **twice**. |
| box channels in pixel units | DFL and the stride multiply are folded into the graph. Values under 1.0 mean they are not, and every box decodes to a dot at the origin. |
| anchor count 8400 | `80² + 40² + 20²` — 640×640 only. A different count means the agent's `cv2.resize(frame,(640,640))` is feeding the wrong size. |

### `[1,300,6]` is refused, loudly

An `end2end=True` export emits `[1,300,6]`, and Ultralytics report that its
**top-k op segfaults on the RK3588 NPU**.

A segfault is not an exception. The agent's one big `try/except` cannot catch
`SIGSEGV` — the process dies, systemd restarts it, and it dies again. So
`fpms-model-verify` runs the inference probe **in a child process**, where a
`SIGSEGV` arrives as return code `-11` and gets reported as a named diagnosis
instead of taking the verifier down with it.

It also refuses the shape on sight, which is cheaper and safer than surviving
the crash.

### Running it while `fpms-npud` holds the NPU

- It **never** stops the daemon. Taking detection offline to prove detection
  works is not a trade a verification tool gets to make on a live rover.
- It asks for the smallest slice it can: core 0 only. (No `core_mask` means
  core 0 — a *bug* for the daemon, which should use all three, and exactly
  right for a probe.)
- Whether the RKNPU driver admits a second context at all is **UNMEASURED**.
  If `init_runtime()` fails while `fpms-npud` is active, the tool reports
  `SKIP`, names the daemon as the likely holder, and prints the command to
  re-run in a maintenance window. A `SKIP` is not a pass and does not exit `0`.

Confirm the contention behaviour the day you have hardware:

```bash
systemctl is-active fpms-npud && fpms-model-verify yolo26n     # contended
sudo systemctl stop fpms-npud && fpms-model-verify yolo26n; sudo systemctl start fpms-npud
```

---

## Making the registry trustworthy — the hardware checklist

Nothing below can be done from a workstation. **The board has not arrived**, and
nothing in this directory has run on an RK3588S.

1. **Rescue the model from the old Pi** (above). Nothing else on this list
   matters if that card dies first.
2. Get it onto the rover at `${FPMS_YOLO_DIR}/yolo26n-rk3588.rknn` (default
   `/home/ubuntu/yolo/`). The path is also hardcoded as `MODEL` in
   `fpms_yolo26_npu.py`; if you move it, change both.
3. Run the offline half anywhere, first — it needs no NPU:
   ```bash
   fpms-model-verify --offline
   ```
   Expect `sha256 on record: WARN` and the observed hash. That is correct
   behaviour for a registry with `null`.
4. Run the full check on the rover:
   ```bash
   fpms-model-verify yolo26n
   ```
   Read the output shape line. It is the reason this tool exists.
5. **Only if it passes**, bless it:
   ```bash
   sudo fpms-model-verify yolo26n --register
   ```
   That writes the sha256, sets `verified: true`, and records who, when and on
   which host. `--register` refuses to run after a failed shape check — a
   registry that adopts whatever it finds verifies nothing.
6. Commit the updated `models.json` back into
   `overlay/etc/fpms/models.json` so the **next image ships knowing the hash.**
   This is the step that converts one lucky rover into a reproducible one, and
   it is the step most likely to be skipped.
7. Record what could not be recovered. `converter_version`,
   `quantization`, `upstream_weights` and `runtime_built_against` are all
   `UNKNOWN` in the shipped registry. If the old Pi or the conversion box can
   still answer any of them, fill them in — and if they cannot, **leave them
   `UNKNOWN`**. That is not a gap to be tidied away; it is why a working rover
   and a broken one could never be compared.

### Confirm on hardware, do not assume

Written down here so the assumptions are visible rather than buried:

- `/dev/rknpu` — the real device node name is **unconfirmed**.
  `ls -l /dev | grep -i rknpu`
- `/sys/kernel/debug/rknpu/version` — the driver version nobody ever recorded.
  `cat /sys/kernel/debug/rknpu/version`
- the end2end byte-scan pre-filter is a **heuristic that has never been tested
  against a real bad export**. The day you have one:
  `strings model.rknn | grep -Ei 'topk|nonmaxsuppression'` on both a known-good
  and a known-bad file. If it does not discriminate, delete the check rather
  than leaving a comforting one in place.

---

## Rebuilding from ONNX

`npu/convert/` — `convert_yolo26.sh` and `rknn_convert.py`.

Two things that will bite, both non-obvious:

- the converter is **`rknn-toolkit2`, x86-64 only.** The Pi runs models; it
  cannot build them. Conversion happens on a workstation and the `.rknn` is
  copied over.
- `onnx==1.14.1` and `numpy<2`. `rknn-toolkit2` calls `onnx.mapping`, removed in
  onnx 1.16, and is built against the NumPy 1.x C API.

And the one that matters most: **export with `end2end=false`.** See above for
what happens otherwise.

A rebuilt model is a *different file* with a different hash. Register it as
such — do not reuse the old entry's `verified: true`.
