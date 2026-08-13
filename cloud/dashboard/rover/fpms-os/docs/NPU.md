# The NPU: active, streaming, blind

This document exists so nobody has to re-derive this in six months.

## The symptom

Everything says healthy. The rover detects nothing.

```
$ systemctl status fpms-rover-agent
   Active: active (running)
$ journalctl -u fpms-rover-agent
   camera found on /dev/video1 (640x480)
   camera negotiated: 6.0 fps, 640x480
   NPU unavailable (...); streaming without detection
$ mosquitto_sub -t 'fpms/rover1/telemetry/camera'
   {"jpeg":"/9j/4AAQ...", "detections":[], "fps":5.9}
```

The video is live. The dashboard renders it. Every unit is `active`. The
selftest's `NPU runtime` check can even pass. And there is not one fire, one
tree or one person in `detections`, ever, for the entire life of the process.

There is no error after that single line. It is logged **once**, at startup, at
the same severity as "camera found". It is above every line an operator scrolls
back through during a run, and `systemctl status` shows the tail.

## Why this is the worst failure mode in the project

`fpms_rover_agent.py`'s `camera_loop()` wraps the **entire** NPU block — the
`import fpms_yolo26_npu`, the `from rknnlite.api import RKNNLite`, the
`load_rknn()`, the `init_runtime()`, and the labels file — in **one
`try/except Exception`**. The handler sets `rknn = None`, logs

```
NPU unavailable ({e}); streaming without detection
```

and falls through into the capture loop, which then runs forever with
`if run_inference and rknn is not None` never true.

The reasoning in the code is not stupid, and it is written down there: *"a video
feed with no boxes still lets an operator see the fire."* That is a defensible
trade when a human is watching the stream. It is indefensible as the *only*
signal, because it is **indistinguishable from success at every level an
operator normally checks**:

| What an operator checks | What it says on a blind rover |
|---|---|
| `systemctl status fpms-rover-agent` | `active (running)` |
| the dashboard camera panel | live video, correct frame rate |
| `telemetry/camera` on MQTT | arriving, at the configured rate |
| `telemetry/health` unit list | all eleven units up |
| `events/fire` | still fires — the HSV colour screen is CPU-side and unaffected |
| the journal | one line, at startup, that scrolled away |

That last row is the trap inside the trap. `detect_fire()` is a **colour screen,
not a detector** — an HSV band on a quarter-scale frame, described in the source
as *"deliberately a screen, not a verdict — a hit raises an event that the
thermal sensor and the cloud VLM then corroborate."* With the NPU down, the
screen keeps raising `events/fire` on any red-orange object and the
corroborating class labels are simply gone. The rover does not go quiet. It gets
**less discriminating while looking exactly as busy.**

There is a second, subtler version of the same shape. If the NPU initialises and
then starts *failing per-frame*, the handler is:

```python
except Exception as e:
    log(f"inference failed: {e}")
```

and the very next non-inference frame runs `detections = list(LAST_DETECTIONS)`
and redraws the carried-over boxes onto the live video. So a wedged NPU produces
a stream that **still has boxes drawn on it** — the last ones it ever managed —
flickering on and off at the `FPMS_INFER_EVERY` cadence. A stale box is worse
than no box, because it is evidence.

This is the same family as `docs/DDS.md`'s "connected, subscribed, silent" and
`FAILURE_MODES.md` D1's "a dead LiDAR read as a completely clear 360°". The
project's recurring defect is not that things break. It is that they break while
continuing to look correct.

## Why the NPU matters here at all

FPMS is a wildfire-detection rover, built for WRO by two Grade 8 students. The
entry is not "a robot that drives to waypoints". It is a robot that finds fire
and reports what it saw. **On-device detection is the product thesis, not an
optimisation.**

That has a consequence for how the layer is designed: a rover that streams video
without detection has not degraded gracefully, it has stopped being the thing it
is. So `FPMS_NPU_REQUIRED=1` is the default, and the failure is a declared fault
on `events/npu_fault` within seconds, not a log line at startup.

---

## The stack, and the version coupling that breaks it

```
kernel rknpu driver        Rockchip BSP kernel ONLY — no mainline driver exists
        ↕  MUST MATCH      (not pinned by our build; it comes with the vendor image)
/usr/lib/librknnrt.so      userspace runtime
        ↕  MUST MATCH      (pinned: config/fpms-os.conf → LIBRKNNRT_URL)
rknn-toolkit-lite2         aarch64 cp310 wheel, NOT on PyPI
                           (pinned: config/fpms-os.conf → RKNN_LITE_WHEEL_URL)
```

`config/fpms-os.conf` pins `RKNN_TOOLKIT2_TAG="v2.3.0"` and derives **both** the
wheel URL and the `librknnrt.so` URL from that one tag. So the bottom two rungs
agree by construction, and that is worth knowing: **the leg that can actually
mismatch is the kernel driver**, which arrives inside the vendor Ubuntu image
and is not pinned by anything we control.

A mismatch there does not fail at import. It fails at **`init_runtime()`** —
which is the third statement inside the one `try` block. So:

> `fpms-selftest`'s existing `check_npu()` runs
> `python3 -c "from rknnlite.api import RKNNLite; print('ok')"` and reports
> **PASS**. It also checks that the `.rknn` file exists and reports **PASS**.
> Both of those pass on a rover whose runtime cannot talk to its driver.

An import check proves the wheel is installed. A stat proves the model is on
disk. Neither proves a single tensor has ever moved. That gap is the whole
reason this layer has a `fpms-npu-selftest` of its own that **runs a real
inference on a known frame and asserts the output shape**.

### Nobody ever wrote down what the old Pi ran

There is no build record for the working rover's NPU stack. The evidence is
`bench_npu.py`: it SSHes into the Pi and, alongside the timing run, executes

```sh
pip3 list | grep -iE 'ultralytics|rknn|onnx'
cat /sys/kernel/debug/rknpu/version
```

**A benchmark that has to ask the machine what it is running is a benchmark for
a machine nobody described.** So when detection broke, there was no way to diff
a working stack against a broken one — the only source of truth was the SD card
itself. That is precisely the condition FPMS-OS exists to end.

Hence `/etc/fpms/npu-versions.json`, written at build time by
`scripts/25-npu-runtime.sh`, carrying the wheel version, the `librknnrt.so`
build string and the tag they came from; and a boot-time check that reads
`/sys/kernel/debug/rknpu/version` and compares. When they disagree the rover
says so on `events/npu_fault` **before** anyone asks it to find a fire.

The **converter** is a different package entirely: `rknn-toolkit2`, x86-only.
The Pi can *run* models and can never *build* one. Nothing on the rover can
regenerate the `.rknn` — which brings us to the thing that should worry you
most.

---

## The model is not in git, and it cannot be reconstructed

`yolo26n-rk3588.rknn` exists **only on the old Pi**, at
`/home/ubuntu/yolo/`. It is in no repository. `scripts/30-fpms-payload.sh`
prints a banner saying so. `FAILURE_MODES.md` §G lists it. `fpms-selftest`
reports it FAIL until it is present.

Everyone has written it down. Nobody has copied it.

This is the same class of loss as the three `/usr/local/bin` scripts in
`FAILURE_MODES.md` A1 — `fpms-wait-net`, `fpms-wifi-ps-hold`,
`fpms-uros-release-reset` — which the whole stack depended on, lived on one SD
card, and were in no repo. **Except those three were reconstructed from prose,
and a model cannot be.** You cannot rebuild a set of quantised weights from a
description of what they do. If that card dies, the rover cannot detect fire,
and there is nothing in this repository that can bring it back.

Note the exposure is worse than one file. The old Pi's `~/yolo/` also holds
`yolov8n.rknn` (the v8 path's model, per `rescued/fpms_yolo_npu.py`) and
`custom_labels.json`, the operator's label overrides, which the agent loads with
a bare `except: labels = {}` — lose it and every detection silently reverts to
raw COCO names.

### What to do about it today

Not "before the competition". Today, while the card still spins:

```sh
# 1. Get it off the card. Any laptop, any directory. Do this first.
scp -r ubuntu@<old-pi>:~/yolo ./yolo-backup-$(date +%F)/

# 2. Record what you actually have, so it can be verified later.
sha256sum ./yolo-backup-*/*.rknn ./yolo-backup-*/*.json | tee MODEL_SHA256.txt
ls -l  ./yolo-backup-*/

# 3. And what produced it, if the .pt / .onnx are still there.
ssh ubuntu@<old-pi> 'ls -l ~/yolo/*.pt ~/yolo/*.onnx; \
                     pip3 list | grep -iE "ultralytics|rknn|onnx|numpy"'
```

Then put the checksums — not the weights — in `/etc/fpms/models.json`, so the
model registry refuses to load anything that is not the file you measured. A
binary blob is a poor fit for git; a checksum of it is not.

The upstream source is `yolo26n.onnx`, and `npu/convert/` exists so the model
*can* be rebuilt from it on an x86 machine. But that path has never been run
either, and it carries pins of its own: **`onnx==1.14.1`** (rknn-toolkit2 calls
`onnx.mapping`, removed in 1.16) and **`numpy<2`** (built against the NumPy 1.x
C API). Treat the conversion pipeline as a recovery plan under test, not as a
substitute for having the file.

---

## The model contract

Read off `models/yolo26n.onnx`, the source the `.rknn` was converted from, and
documented in `fpms_yolo26_npu.py`:

| | |
|---|---|
| input | `images` `[1, 3, 640, 640]` |
| output | `output0` `[1, 84, 8400]` |
| 8400 | 80·80 + 40·40 + 20·20 anchors |
| 84 | 4 box + 80 class |

Five properties, each of which has a specific way of going wrong:

**1. `end2end=false` is mandatory.** Ultralytics' RKNN documentation reports
that `end2end=True` emits `[1, 300, 6]`, and that its top-k op **segfaults on
the RK3588 NPU**. Not "runs slowly", not "returns garbage" — takes the process
down. This is a property of the *export*, so it is decided on the x86 converter
box, months before anything runs on the rover. `npu/convert/` must not offer it
as an option and `fpms-model-verify` must reject a `[1,300,6]` output shape by
name.

**2. DFL is already folded into the graph, and classes are already sigmoided.**
The two Softmax ops in `model.23` are baked in, so channels 0..3 arrive as xywh
**centre form already multiplied by the per-anchor stride** — `[8]*6400 +
[16]*1600 + [32]*400` — i.e. in 640×640 input-pixel units. Channels 4..83 have
already been through Sigmoid; they are probabilities.

> **Do not sigmoid them again.** A second sigmoid is the nastiest possible bug
> here because it does not crash and does not empty the output: σ is monotonic,
> so the *argmax class is unchanged* and boxes still appear in the right places.
> Only the confidences move — σ(0.9) ≈ 0.71, σ(0.35) ≈ 0.59 — compressing
> everything toward 0.5. Against `CONF_THRES = 0.35`, that turns every
> low-confidence background anchor into a detection and makes a genuine 0.95
> detection look marginal. You get a plausible-looking, uniformly wrong
> confidence scale, and nothing anywhere says so.

**3. "NMS-free" is a training-time property, not a description of the tensor.**
YOLO26 is trained NMS-free; the exported graph, verified by op histogram,
contains **no NonMaxSuppression and no TopK**, and still emits all 8400 anchors.
So thresholding and a light NMS still run **on the CPU**, after the NPU is done.
That CPU tail — argmax over 8400×80, the mask, the per-class NMS — is real work
on every inferred frame and belongs in the latency budget. Duplicate suppression
is much weaker than v8's, but it is not identically zero, so the NMS stays.

**4. Preprocessing is a stretch, not a letterbox.** The agent does a plain
`cv2.resize(frame, (640, 640))` on a 640×480 capture. There is no padding and no
aspect-ratio preservation, so rescaling boxes back to the source frame is an
**independent per-axis ratio** — `iw/640` on x, `ih/640` on y — *not* a single
uniform scale plus an offset. On 640×480 those ratios are 1.0 and 0.75, which
means a uniform-scale decoder would be wrong **only in y**, by 25%: boxes that
look almost right, consistently too tall, in a way that reads as "the model
isn't great" rather than as a bug. If preprocessing ever becomes a letterbox,
the decoder needs padding compensation added *in the same commit*.

**5. The output layout is not guaranteed.** RKNN is not consistent about
returning the ONNX `(1,84,8400)` or a channel-last `(1,8400,84)`, and may or may
not keep the batch axis. `fpms_yolo26_npu._as_anchor_major()` locates the 84
axis rather than trusting a shape, and raises with a specific message if the
export split the head into separate branch tensors. Keep that behaviour: a
transposed decode does not error, it produces boxes from class probabilities.

---

## The three cores

The RK3588S NPU has **three cores, 6 TOPS combined**. `init_runtime()` called
with no `core_mask` **uses core 0 only.**

`fpms_rover_agent.py` calls `rknn.init_runtime()` — no mask. So does
`bench_npu.py`. **Every number this project has ever measured for NPU inference
was measured on one third of the NPU**, and the one place in the codebase that
does pass a mask, `rescued/fpms_yolo_npu.py`, passes
`core_mask=RKNNLite.NPU_CORE_0` — explicitly pinning the same single core. There
is no evidence anyone ever ran a multi-core configuration on this hardware.

`RKNNLite.NPU_CORE_0_1_2` exists. But **do not assume it is a 3× win** — it is
not obviously even a win at all:

- One model spread across three cores lowers *per-inference latency*, which is
  what a 3 inferences/second duty cycle actually needs.
- Three cores each running their own model instance raises *throughput*, which
  this rover does not need, and lets the LiDAR-adjacent work proceed while a
  frame is in flight.
- A 640×640 nano model may not have enough parallel work to keep three cores
  busy, in which case the mask buys scheduling overhead and heat.

So `FPMS_NPU_CORE_MASK` is a variable, `fpms-npu-tune` sets it, and
`npu/bench/fpms_npu_bench.py` exists to answer the question with numbers rather
than with this paragraph. **MEASURE ME.** What is *not* in doubt is that leaving
it unset means two thirds of the part is idle, which is not a defensible
default.

## Thermal

RK3588 throttles hard, and it does it silently — the governor drops the NPU
clock and nothing in `rknnlite` mentions it. There is one consequence and it is
methodological:

> **A first-inference number is meaningless.** So is a thirty-frame run.

`bench_npu.py` averages 30 frames after a 3-frame warm-up. That is a fine
measurement of *cold* performance and tells you nothing about a rover that has
been patrolling for eleven minutes in a gym under stage lights. The benchmark in
this layer must report **sustained** latency — a percentile distribution over
minutes, with the SoC temperature sampled alongside it — and the health topic
must publish latency percentiles, not a mean, for the same reason.

`fpms-npu-tune` owns the DVFS and governor policy. The honest position today is
that nobody knows how much this board throttles, because nobody has run a
sustained load on one.

---

## The architecture FPMS-OS adopts

Two structural changes, and the rest follows from them.

**Inference moves out of the camera loop, into `fpms-npud`.** Today
`rknn.inference()` is called inline in `camera_loop()`, on the same thread that
reads the camera. A slow or wedged NPU call therefore stalls the frame pump —
and that same agent process also owns the LiDAR serial port and republishes
scans to MQTT. `FAILURE_MODES.md` B3 records what happens when the camera path
delays the LiDAR path: nothing is dropped, scans just arrive *late*, and a
guard that thinks it is looking at the present is looking at the past. A daemon
behind a unix socket makes an NPU stall cost detections and nothing else.

**Nothing loads without being verified first.** A model registry that refuses an
unverified `.rknn`; a version manifest checked at boot; a selftest that runs a
real inference rather than an import.

```
  USB UVC camera   30 fps hardware cap (USB descriptor) — shipped at 6
         │
         │  BGR 640x480
         ▼
  fpms-rover-agent ──────────────────────────────► MQTT telemetry/camera
    camera_loop                                     JPEG q40, every frame
         │
         │  every FPMS_INFER_EVERY-th frame (=2)  → ~3/s
         │  cv2.resize 640x640  ** STRETCH **  +  BGR→RGB
         ▼
  /run/fpms/npud.sock                    ← the boundary that did not exist
         │
         ▼
  fpms-npud.service                        own process, own thread
    ├─ /etc/fpms/npu-versions.json    manifest ↔ /sys/kernel/debug/rknpu/version
    ├─ /etc/fpms/models.json          registry — refuses an unverified model
    ├─ load_rknn(/home/ubuntu/yolo/yolo26n-rk3588.rknn)
    └─ init_runtime(core_mask=FPMS_NPU_CORE_MASK)   ← not the default core 0
         │
         │  [1,84,8400] float32
         ▼
    decode  (fpms_yolo26_npu.decode)              ON THE CPU:
         │    threshold @0.35 · per-class NMS @0.55 · per-axis rescale
         ▼
    boxes / scores / classes
         │
         ▼
  fpms-rover-agent   overlay boxes on the JPEG
         ├──► MQTT telemetry/camera      detections[]
         ├──► MQTT events/fire           QoS 1   (corroborated by the HSV screen)
         ├──► MQTT events/wildlife       QoS 1
         ├──► MQTT telemetry/npu         per-core load, latency percentiles
         └──► MQTT events/npu_fault      QoS 1, EDGE-TRIGGERED

  fpms-npu-tune.service ──► DVFS / governor / core mask, once at boot
  fpms-npu-selftest     ──► real inference, real shape assert
                            → /var/lib/fpms/npu-selftest.json
```

The paths in that diagram are the shared contract from `NPU_SPEC.md` §3. Every
one of them is a promise between agents, not an implementation detail:

```
/dev/rknpu                          device node — VERIFY ON HARDWARE
/sys/kernel/debug/rknpu/version     driver version
/sys/kernel/debug/rknpu/load        per-core utilisation
/usr/lib/librknnrt.so               runtime
/etc/fpms/npu-versions.json         written at build time by stage 25
/etc/fpms/models.json               the model registry
/home/ubuntu/yolo/yolo26n-rk3588.rknn
/var/lib/fpms/npu-selftest.json     selftest output
/var/lib/fpms/model-provision.json  boot-time model state + the fault edge
/run/fpms/npud.sock                 daemon IPC
MQTT  fpms/<thing>/telemetry/npu
MQTT  fpms/<thing>/events/npu_fault
```

Two notes on that list. `/dev/rknpu` is the *expected* node name and has not
been confirmed on an RK3588S — `99-fpms-npu.rules` must grant the unprivileged
`ubuntu` user access to whatever the node actually is, which is why the rule is
written against a verified `ls -l /dev/rknpu*` and not against this document.
And `events/npu_fault` is **edge-triggered at QoS 1** deliberately: the bridge
to the operator laptop is `topic # both 0`, so QoS-1 guarantees hold only within
the rover, and a fault that publishes once per frame on a saturated link is a
fault that gets dropped like everything else.

---

## The boot sequence

The model is not in the image and cannot be, so **the moment a rover is most
likely to be blind is the moment it comes up**. That is where this layer has to
be loudest, and it is what `fpms-model-provision.service` is for.

```
fpms-firstboot.service          grows the rootfs, WiFi, hostname, the broker
        │                       password, and creates ${FPMS_YOLO_DIR}
        ▼
fpms-model-provision.service    ONESHOT, ROOT, RemainAfterExit
        │                         1. find the vfat boot partition (statfs, not
        │                            findmnt | grep -q)
        │                         2. is ${FPMS_YOLO_DIR}/yolo26n-rk3588.rknn there?
        │                         3. is a copy on the boot partition?
        │                         4. install / verify / register / DECLARE
        │                       -> /var/lib/fpms/model-provision.json
        │                       -> MQTT events/npu_fault   QoS 1, edge-triggered
        ▼
fpms-npu-tune.service           core mask and thermal policy
        ▼
fpms-npud.service               loads the model, probes the shape, serves
        │                       /run/fpms/npud.sock
        ▼
fpms-rover-agent.service        camera, MQTT, overlay
```

The `Before=fpms-npud.service` edge is **ordering only**, and it is an
optimisation rather than a correctness requirement: the daemon retries its load
every `FPMS_NPU_RELOAD_BACKOFF_S` (30 s) forever, so a model copied across at
runtime is picked up without anyone restarting anything. What the edge does buy
is an **uncontended device** for the verification inference, so an
`init_runtime()` failure at boot means a version mismatch rather than "the
daemon already had it" — which `fpms-model-verify` can only report as a `SKIP`.

Nothing declares `Requires=` on this unit. A rover with no detection model must
still boot, drive, stream and answer the console.

### At boot, with a model

```
fpms-model-provision: NPU model provisioning
fpms-model-provision: boot partition: /boot/firmware
fpms-model-provision: model:          /home/ubuntu/yolo/yolo26n-rk3588.rknn
fpms-model-provision: DETECTION MODEL VERIFIED
fpms-model-provision:   sha256 58b38aee2dec4b1f...
fpms-model-provision:   output shape OBSERVED as [1,84,8400] on this boot
```

It runs the **full** `fpms-model-verify` — not `--offline` — on every boot where
a model is present, and that is deliberate. The thing it is watching for is the
three-way version coupling (kernel `rknpu` ↔ `librknnrt.so` ↔ the wheel), which
fails at `init_runtime()`, which the agent swallows, and which can break
*between* boots: an apt upgrade, a different card, a vendor kernel. A hash check
sees none of that.

If the shape is observed and the registry does not already record it, the file
is **blessed** — `fpms-model-verify --register` writes the sha256,
`verified: true`, and who/when/where. It will not overwrite a *different*
recorded hash without `--force` (a registry that quietly adopts whatever it
finds verifies nothing), and it does not re-register on a rover that is already
blessed, so a healthy boot does not churn an `/etc` file with a new timestamp
every time.

> **A pass here does not prove the permission.** The unit runs as root and
> `fpms-model-verify` does not drop privileges, so a pass proves the model and
> the version chain. It does **not** prove that the unprivileged `ubuntu` user —
> which is what actually runs `fpms-npud` and the agent — can open the device.
> That gap is what `99-fpms-npu.rules` closes and what `fpms-npu-selftest`
> detects, by dropping to `ubuntu` on purpose. Run it before believing a run.

### At boot, without one

This is the case the unit exists for, and it produces **four** signals, because
every single-signal design this project has tried has been missed:

| Signal | Where it lands |
|---|---|
| a console banner: `*** NO USABLE DETECTION MODEL ON THIS ROVER ***` | `StandardOutput=journal+console` — readable on a monitor with no laptop |
| `events/npu_fault`, QoS 1, edge-triggered, cooled down | MQTT → dashboard → the operator alert path |
| `/var/lib/fpms/model-provision.json` | a marker `fpms-selftest` / `fpms-npu-selftest` can read without loading anything or opening the NPU |
| the unit **FAILS** (exit 3) | `systemctl --failed`, `systemctl is-system-running` → `degraded`, and the health topic's unit list |

The exit code is the interface, and the `0`/`1` split matters as much as the
`3`:

| Exit | Meaning |
|---|---|
| `0` | installed **and** the output shape was OBSERVED this boot |
| `1` | installed and IDENTIFIED, shape **UNPROVEN** — no `rknnlite`, a contended device, a check that could not run. `SuccessExitStatus=0 1`, so this is not a unit failure; it is the honest day-one state of a rover whose NPU stack has never been proven. |
| `3` | **no usable model.** The unit fails, on purpose. |
| `4` | the script could not do its own job (unwritable state dir). |

"We could not check" never shares an exit code with "we checked and it was
fine" — the same contract as `fpms-model-verify` and `fpms-selftest`, for the
same reason: otherwise every boot script quietly accepts an unverified model.

The faults are a fixed vocabulary, never free text:

| Fault | What it means |
|---|---|
| `model_missing` | nothing installed, and nothing on the boot partition |
| `model_conflict` | an installed model and a *different* one on the boot partition, with no registry hash to arbitrate. **Refuses to choose**; nothing is overwritten. |
| `model_end2end_suspect` | the container carries `TopK` / `NonMaxSuppression` strings. Copied off the card as `…​.rknn.end2end-suspect`, kept **out of the load path** |
| `model_install_failed` | the copy did not match the source — a truncated read off a tired card, not a model problem |
| `model_unverified` | present and identified; the shape has never been observed here |
| `model_verify_failed` | `fpms-model-verify` FAILED. Read `/var/lib/fpms/model-provision-verify.txt`. |
| `model_verify_timeout` | a wedged NPU call, bounded so it could not stall the boot |

### Why the fault event has a cooldown

`events/npu_fault` is published on the **rising edge of a named fault**, then at
most once per `FPMS_MODEL_FAULT_REPEAT_S` (default **3600 s**) while it
persists, and `events/npu_recovered` once on the falling edge. A change of fault
*name* counts as a new edge; the same name repeating does not.

This is not theoretical tidiness. A past FPMS bug emailed 330 obstacle events
and exhausted the Resend daily quota, so **a real fire alert could not send**
(`CLAUDE.md`, "Hard-won constraints"; `cloud/HANDOFF.md` item 10). An alert path
that fires per occurrence does not merely spam — it destroys the channel for the
one event the product exists to report.

A once-per-boot publisher looks safe until you picture the afternoon before a
competition: a bench, a loose barrel jack, a card being reflashed — dozens of
boots an hour, every one with no model, every one an event. So the fault state
**persists across reboots** in `model-provision.json`, and the cooldown is
twelve times `fpms-npud`'s 300 s precisely because a per-boot publisher has a
much worse worst case than a daemon.

`last_sent` advances **only on a publish that actually left the machine**. This
unit runs seconds after `mosquitto.service` starts, and "started" is not
"accepting connections"; marking an edge as spent on a publish that went nowhere
would swallow the most important alert this rover can send for a whole cooldown.
Same reasoning, verbatim, as `fpms-npud`'s `set_fault()`.

### Getting a model onto a rover from Windows

This is the realistic delivery path for a binary that cannot live in git, and it
is the same one `fpms-wifi.conf` already uses.

1. **Rescue it from the old Pi first.** Nothing below matters if that card dies —
   `npu/models/README.md` has the `scp` + `sha256sum` recipe.
2. Put the rover's SD card in a Windows machine. It mounts the **FAT boot
   partition** — the one with `README-FPMS.txt` and `fpms-wifi.conf.example` on
   it. (Windows cannot see the Linux rootfs at all, which is exactly why this
   partition is the delivery mechanism.)
3. Copy the model there, named **exactly** `yolo26n-rk3588.rknn`. Case does not
   matter, vfat folds it. The `yolo\` and `fpms\` subdirectories of that
   partition are searched too, so dropping the whole rescued `yolo` folder in
   works.
4. Eject, boot the rover, and watch for the banner.

```sh
ssh ubuntu@fpms-rover1.local
fpms-model-provision --status              # what the last boot decided
journalctl -u fpms-model-provision -b --no-pager
```

Three things it deliberately will **not** do:

- **It does not delete the file from the boot partition.** `fpms-wifi.conf` is
  renamed to `.applied` because a WiFi password on a card that any computer will
  mount is a hazard. A model is the opposite: it is the only copy in the world,
  and leaving it there means a re-flash of this card still has it. Do not "tidy"
  this.
- **It does not adopt a differently-named `.rknn`.** yolov8n's exported head is
  *also* `[1,84,8400]`, so no shape check anywhere in this stack can tell a stray
  `yolov8n.rknn` from the right file. Installing one under the canonical name
  would produce a rover that verifies clean, detects confidently, and is wrong.
  It reports what it found and refuses.
- **It does not put a suspected `end2end=True` export into the load path.** That
  model's top-k op segfaults on the NPU, and a SIGSEGV is not an exception:
  `fpms-npud` dies, systemd restarts it, and it dies again — a crash loop that
  publishes *nothing*, which is the same silence by another route. The file is
  still copied off the card (as `…​.rknn.end2end-suspect`, because the card
  is the thing that might die tonight), just not into the path anything loads.
  The byte scan is **an unconfirmed heuristic**, so there is one documented
  override:

  ```sh
  sudo fpms-model-provision --force
  ```

  If it ever misfires on the project's one good model, override it and then
  **delete the check** rather than leaving a comforting one in place — the same
  instruction `npu/models/README.md` gives.

### Two smaller things, written down so they are not re-derived

It also installs `custom_labels.json` from the boot partition if one is there
and none is installed. That is deliberately not a fault path: the agent loads
that file inside a bare `try/except` and falls back to the compiled
`COCO_NAMES`, so a missing one costs display names, not detection.

It does not touch `fpms_yolo26_npu.py`. That module — the v26 decode path — **is**
in the repository, and `scripts/30-fpms-payload.sh` installs it to
`${FPMS_YOLO_DIR}`. The `.rknn` really is the only piece of the v26 detection
path that is missing, which is why one file on a FAT partition is enough to fix
it.

---

## How to verify it is really working

**"The unit is active" proves nothing here.** Neither does live video, nor a
green dashboard, nor `rknnlite` importing. Every one of those is true on a rover
that has never run a single inference. The check has to exercise the thing that
actually fails.

```sh
# 0. Is there a model at all, and did anything check it? This is the cheapest
#    question and the one most often skipped. It loads nothing and opens
#    nothing - it reads what the boot already decided.
fpms-model-provision --status
systemctl status fpms-model-provision      # FAILED here means: no usable model
journalctl -u fpms-model-provision -b --no-pager

# 1. Did the agent say the words? This is the one line that matters, and it
#    appears ONCE at startup, so grep the whole journal, not the tail.
journalctl -u fpms-rover-agent --no-pager | grep -E 'NPU ready|NPU unavailable'
#    want:  NPU ready with /home/ubuntu/yolo/yolo26n-rk3588.rknn (variant v26)
#    fear:  NPU unavailable (...); streaming without detection

# 2. Does the stack agree with itself? Manifest vs. the running kernel.
cat /etc/fpms/npu-versions.json
sudo cat /sys/kernel/debug/rknpu/version

# 3. Run a REAL inference and assert the shape. This is the check that an
#    import test and a stat cannot fake.
sudo fpms-npu-selftest -v
cat /var/lib/fpms/npu-selftest.json

# 4. Is the model the model you measured?
fpms-model-verify --all

# 5. Are the cores you paid for actually being used? Watch during a patrol.
watch -n1 sudo cat /sys/kernel/debug/rknpu/load

# 6. Sustained latency, not a first-inference number.
python3 /opt/fpms/npu/bench/fpms_npu_bench.py --seconds 180

# 7. The end-to-end proof: point the camera at a person and watch the wire.
mosquitto_sub -h 127.0.0.1 -u fpms -P "$FPMS_MQTT_PASS" \
  -t 'fpms/rover1/telemetry/camera' -C 20 | python3 -c '
import sys, json
for line in sys.stdin:
    d = json.loads(line).get("detections", [])
    print([(x["label"], x["conf"]) for x in d])'
```

Step 7 is the one that cannot lie. A non-empty `detections` array containing a
COCO label, from a frame you controlled, is the only evidence that the whole
chain — driver, runtime, model, core mask, socket, decode, rescale — is intact.
Everything above it is a narrowing of where the fault is.

Run step 7 again after ten minutes of patrolling. Thermal throttling and a
per-frame `inference failed:` loop both look fine at second zero.

---

## The duty cycle: this is a latency problem, not a throughput one

| | |
|---|---|
| camera hardware cap | **30 fps**, every resolution — the sensor's USB descriptor, not a software limit |
| shipped | **`FPMS_CAMERA_FPS=6`, `FPMS_JPEG_QUALITY=40`** |
| inference cadence | **`FPMS_INFER_EVERY=2`** |
| → NPU budget | **≈ 3 inferences per second** |

The 6 fps is **deliberate and load-bearing**, not a placeholder. `config.env`
records why: at higher settings the camera stream saturated WiFi and starved the
LiDAR feed. `FAILURE_MODES.md` B3 has the mechanism — paho serialises all
publishes from one client onto one thread, so a 40 kB base64 frame queued ahead
of a 2.5 kB scan delayed the scan by however long the frame took to drain.
Nothing was dropped; scans arrived *late*.

So **do not raise the frame rate to feed the NPU.** If you do, re-measure the
LiDAR rate that actually reaches the dashboard, not the rate the agent logs.

The design consequence is the whole point of this section: at three inferences
per second, a 6 TOPS NPU is not remotely the bottleneck, and optimising
throughput is optimising nothing. What matters is:

- **latency** — a single inference must complete well inside its ~333 ms slot,
  and must still do so at minute eleven under thermal load;
- **reliability** — three inferences per second means a fault takes seconds, not
  frames, to become visible, so it has to be *announced* rather than inferred
  from a rate;
- **never stalling the neighbours** — which is why `fpms-npud` is a separate
  process and why the socket has a timeout.

A core mask that halves latency is worth having. A core mask that triples
throughput is worth nothing here.

---

## What is NOT fixed, and what is not verified

In the register of `FAILURE_MODES.md`: entries here are not fixed. Read them
before a competition.

| | Status |
|---|---|
| **Nothing in this layer has run on an RK3588S** | **The board has not arrived.** Every latency figure, core-mask recommendation, device path and sysfs node in this document is derived from documentation and from the old Pi, not measured on the target. |
| **The model is still not in git** | `yolo26n-rk3588.rknn` exists only on the old Pi's SD card. `yolov8n.rknn` and `custom_labels.json` with it. Nothing in this repository can rebuild them, and the conversion pipeline that theoretically could has never been run. `fpms-model-provision` gives it a delivery path and makes its absence loud; **it does not make a copy exist.** This is still the single highest-value thing anyone can fix this week, and it needs `scp`, not code. |
| **`fpms-model-provision` has never seen a real vfat boot partition** | its decision table, fault vocabulary, cooldown and exit codes were exercised against stubs on a workstation, including the found / absent / conflicting / truncated / suspected-end2end paths. The `statfs` probe for `/boot/firmware`, the install into `${FPMS_YOLO_DIR}` on a real rootfs, and the `mosquitto_pub` at boot have not run on hardware. Confirm the first with `stat -f -c %T /boot/firmware`. |
| **The end2end byte scan has still never met a bad export** | `fpms-model-provision` withholds a flagged file from the load path where `fpms-model-verify` only WARNs, because a SIGSEGV here is a crash loop rather than a wrong number. Both are the same unconfirmed heuristic. The day a real `end2end=True` export exists, run `strings model.rknn \| grep -Ei 'topk\|nonmaxsuppression'` against a known-good and a known-bad file, and **delete the check** rather than leave a comforting one if it does not discriminate. |
| **`/dev/rknpu` is unverified** | the node name is inferred. Confirm with `ls -l /dev/rknpu*` on the real board before trusting `99-fpms-npu.rules`. |
| **No multi-core configuration has ever run** | every NPU number this project owns was measured with `init_runtime()` on the default core 0, or explicitly pinned to `NPU_CORE_0`. The 3× is an assumption. |
| **Thermal behaviour is completely unmeasured** | no sustained-load run exists, on this board or the old Pi. `bench_npu.py` averages 30 frames after a 3-frame warm-up. |
| **The kernel `rknpu` version is unknown and unpinnable** | it ships inside the vendor image. The build pins the wheel and `librknnrt.so` to tag `v2.3.0`; the driver is whatever the base image contains. The manifest check detects the mismatch — it cannot prevent it. |
| **The one try/except in `fpms_rover_agent.py` is still there** | this layer routes around it; it does not remove it. `fpms_rover_agent.py` is application code owned outside FPMS-OS. Until it is changed, the startup log line remains the only in-process signal, and `events/npu_fault` is the thing to trust. |
| **`fpms-selftest`'s existing NPU checks pass on a blind rover** | `check_npu()` tests `import rknnlite` and `os.path.exists(model)`. A driver/runtime mismatch fails at `init_runtime()`, downstream of both. `fpms-npu-selftest` exists to close this and has itself never run on hardware. |
| **The stale-box behaviour is unaddressed** | if inference starts throwing mid-run, carried-over boxes keep being drawn on the video at the `INFER_EVERY` cadence. An operator watching the stream sees detections that are minutes old. |
| **`FPMS_INFER_EVERY=2` against `FPMS_CAMERA_FPS=6` has never been profiled end to end** | the ~3/s figure is arithmetic. The agent's own comment cites a ~31 ms YOLO pass, measured on the **v8** model, on **one core**, cold. |

---

## Configuration reference

These four are **new**. They do not exist in `/etc/fpms/config.env` today.

| Variable | Default | Meaning |
|---|---|---|
| `FPMS_NPU_CORE_MASK` | `0_1_2` | Which NPU cores `init_runtime()` binds. `0`, `0_1`, `0_1_2`, or `auto`. Unset in the RKNN API means **core 0 only**, which is why this must be set explicitly rather than left to the library. `0_1_2` is the *starting* value, not a measured one — **MEASURE ME** with `npu/bench/fpms_npu_bench.py`. |
| `FPMS_NPU_REQUIRED` | `1` | `1` — a rover that cannot infer is a **declared fault**: `fpms-npud` publishes `events/npu_fault` at QoS 1 and the selftest FAILs. `0` — permit streaming-only operation, for bench work on a board with no model. Default `1` because on this rover detection is the product, not a feature. |
| `FPMS_NPU_MAX_LATENCY_MS` | `250` | Per-inference deadline. Exceeding it is a fault event, not a warning, because the failure it is watching for is thermal throttling, which is gradual and silent. Derived from the ~333 ms budget implied by 3 inferences/second — **MEASURE ME**, this number is arithmetic, not observation. |
| `FPMS_NPU_SOCKET` | `/run/fpms/npud.sock` | The `fpms-npud` unix socket. Under `/run` deliberately: tmpfs, cleared on every boot, so a stale socket from a previous process era cannot be connected to. (`docs/DDS.md` is a long story about exactly that mistake in another transport.) |
| `FPMS_MODEL_FAULT_REPEAT_S` | `3600` | How long `fpms-model-provision` waits before re-publishing an **unchanged** `events/npu_fault`. Twelve times `fpms-npud`'s `FPMS_NPU_FAULT_REPEAT_S`, because a once-per-boot publisher on a bench being reflashed has a far worse worst case than a daemon. A change of fault *name* ignores it. Set to `0` only when testing the publish path. |

Two that already exist and are covered by `config.env` today:

| Variable | Shipped | Meaning |
|---|---|---|
| `FPMS_YOLO_VARIANT` | `v26` | **Must be set explicitly.** The code default in `fpms_rover_agent.py` is `"v8"`, which imports `fpms_yolo_npu` from `FPMS_YOLO_DIR` — a module the image does not contain. Left at the default, the agent takes the one try/except and runs blind while every unit reports active. |
| `FPMS_YOLO_DIR` | `/home/ubuntu/yolo` | Where the model, the decode module and `custom_labels.json` live. |

### Follow-up: `config.env` has not been edited

**This is a flagged action item, not a completed change.** The four new variables
above are documented here and consumed by `fpms-npud`, `fpms-npu-tune` and
`fpms-npu-selftest`, but `overlay/etc/fpms/config.env` is owned by another agent
(`SPEC.md` §9, agent E) and has deliberately not been touched by this document.

Someone must add this block to `config.env`, with the provenance markers that
file requires:

```sh
# ============================================================================
# NPU  (see docs/NPU.md)
# ============================================================================
#
# Unset, the RKNN runtime binds core 0 ONLY - two thirds of a 6 TOPS NPU idle.
# 0_1_2 is a starting value, NOT a measurement. UNMEASURED: no multi-core
# configuration has ever run on this hardware.
FPMS_NPU_CORE_MASK=0_1_2

# 1 = a rover that cannot infer is a DECLARED FAULT. On this rover detection is
# the product; streaming without it is not graceful degradation.
FPMS_NPU_REQUIRED=1

# ASSUMED. Arithmetic from the ~333 ms budget implied by 3 inferences/second,
# not an observation. Re-derive from fpms_npu_bench.py on the real board.
FPMS_NPU_MAX_LATENCY_MS=250

# Under /run deliberately: tmpfs, cleared on boot, so no stale socket survives
# a process era.
FPMS_NPU_SOCKET=/run/fpms/npud.sock
```

Until that lands, every consumer must apply the defaults in the table above when
the variable is absent — and say in its log which value it used and where it
came from. A default that is applied silently is how this project got
`FPMS_YOLO_VARIANT=v8` on a rover with no v8 module.

### Follow-up: `fpms-model-provision.service` is not enabled yet

**This is a flagged action item, not a completed change.** Units are enabled at
build time by `scripts/60-enable-units.sh`, from the `BOOT_UNITS` array near the
top of that file — it runs `systemctl enable` in the chroot, then verifies (and
if necessary creates) the `multi-user.target.wants` symlink by hand. That script
belongs to agent A, so this document names the change rather than making it:

```sh
# scripts/60-enable-units.sh, in BOOT_UNITS
    fpms-model-provision.service
```

Until that one line lands, `fpms-model-provision.service` ships **installed but
disabled**, which is the worst of both worlds: the file is present, `systemctl
cat` shows it, and it never runs. `fpms-missions` was once found in exactly that
state. There is no runtime symptom to notice, because the whole point of the
unit is to speak up when nothing else does.

Confirm on a flashed card with:

```sh
systemctl is-enabled fpms-model-provision     # want: enabled
```

Two smaller follow-ups in the same category, both outside this layer's files:

- `fpms-selftest`'s `check_npu()` and `fpms-npu-selftest`'s `check_model()` both
  answer "is the model there" by `os.path.exists`. Neither reads
  `/var/lib/fpms/model-provision.json`, so neither can currently distinguish
  *absent* from *present but rejected as an end2end suspect*, nor report which
  boot last proved the shape. Folding that marker in is a few lines in each and
  removes a whole class of "the file is there so it must be fine".
- `overlay/boot/README-FPMS.txt` — the operator-facing card README — documents
  `fpms-wifi.conf`, `fpms-hostname`, `fpms-broker-host` and
  `fpms-mqtt-password`, and says nothing about the model. The one place an
  operator with a card in a Windows reader will actually look should carry the
  four lines from "Getting a model onto a rover from Windows" above. It is owned
  by agent B.

---

## If none of this works

Then the fault is somewhere this document does not describe, and this document
should be corrected rather than worked around.

The order to narrow it, cheapest first:

1. `sudo cat /sys/kernel/debug/rknpu/version` — if the file is absent, the BSP
   kernel's `rknpu` is not loaded and nothing above it can work. That is a
   kernel/base-image problem, not an FPMS one.
2. `python3 -c "from rknnlite.api import RKNNLite; RKNNLite()"` — isolates the
   wheel.
3. `fpms-npu-selftest -v` — isolates `init_runtime()`, the model, and the shape,
   which is where a version mismatch actually lands.
4. `fpms-model-verify --all` — isolates the file itself.

And if the model is simply gone, there is no diagnostic step that helps. Copy it
off the old Pi. That instruction has been in three files in this repository for
some time and copying it takes thirty seconds.
