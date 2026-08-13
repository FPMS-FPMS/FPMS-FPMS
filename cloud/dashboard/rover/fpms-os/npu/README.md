# `npu/` — the FPMS-OS NPU layer

The RK3588S carries a **6 TOPS NPU with three cores**. It is what makes FPMS a
fire-detection rover rather than a camera on wheels: YOLO26 runs on it, on
device, and the product thesis depends on that inference being real.

This directory is the build-side half of that layer — conversion, tests,
benchmarks, version records. The runtime half lands on the rootfs from
`../overlay/`.

**Read `../docs/NPU.md` first if you have five minutes.** It is the deep
document: the failure this layer exists to make impossible, the version
coupling, the model contract, and the honest register of what has and has not
been verified.

## The failure this layer exists to make impossible

`fpms_rover_agent.py` wraps the whole NPU block — import, `load_rknn`,
`init_runtime`, labels — in **one try/except** that logs

```
NPU unavailable (...); streaming without detection
```

once, at startup, and then carries on forever. Every unit reports `active`. The
camera streams. The dashboard looks alive. **The rover detects nothing.**

It is the worst failure mode in the project because it is indistinguishable from
success at every level an operator normally checks. Everything below exists to
make it loud, early, and impossible to miss.

## File map

Ownership per `NPU_SPEC.md` §3. Everything is under `fpms-os/`.

| Path | Purpose |
|---|---|
| `npu/NPU_SPEC.md` | **the contract.** Where it and your judgement disagree, it wins |
| `npu/versions/README.md` | what runtime/driver/wheel versions were pinned, and how they were confirmed |
| `npu/models/README.md` | the model registry's format, and what "verified" means |
| `npu/convert/convert_yolo26.sh` | x86-only: `.pt` → `.onnx` → `.rknn`, with the pins that matter |
| `npu/convert/rknn_convert.py` | the conversion itself (`end2end=false`, always) |
| `npu/convert/README.md` | why the Pi can never build a model, only run one |
| `npu/tests/test_decode.py` | decode correctness — offline, no hardware |
| `npu/tests/README.md` | what the decode tests assert and why each one exists |
| `npu/bench/fpms_npu_bench.py` | **sustained** latency percentiles, per-core load, temperature |
| `npu/README.md` | this file |
| `docs/NPU.md` | the deep document |

Landing on the rootfs, from `../overlay/` and `../scripts/`:

| Path | Purpose |
|---|---|
| `scripts/25-npu-runtime.sh` | installs the runtime; writes `/etc/fpms/npu-versions.json` at build time |
| `overlay/etc/udev/rules.d/99-fpms-npu.rules` | gives the unprivileged `ubuntu` user access to the NPU device |
| `overlay/usr/local/bin/fpms-npud` | the inference daemon — keeps a stalled NPU off the camera and LiDAR threads |
| `overlay/etc/systemd/system/fpms-npud.service` | its unit |
| `overlay/usr/local/bin/fpms-model-verify` | refuses to load a model that is not the one you measured |
| `overlay/etc/fpms/models.json` | the model registry |
| `overlay/usr/local/bin/fpms-npu-tune` | core mask, DVFS, governor |
| `overlay/etc/systemd/system/fpms-npu-tune.service` | applies it once at boot |
| `overlay/usr/local/bin/fpms-npu-selftest` | runs a **real inference** and asserts the output shape |

## The order an operator uses them

Each step assumes the one above it passed. Do not skip ahead — a benchmark of a
wrong model is a number about nothing.

```
1.  fpms-model-verify --all           is the model the one we measured?
2.  cat /etc/fpms/npu-versions.json   do driver, runtime and wheel agree?
    sudo cat /sys/kernel/debug/rknpu/version
3.  sudo fpms-npu-selftest -v         does a real inference return [1,84,8400]?
4.  fpms_npu_bench.py --seconds 180   what is SUSTAINED latency, not first-shot?
5.  fpms-npu-tune                     set the core mask the benchmark chose
```

Then the only check that cannot lie — point the camera at a person and watch the
wire:

```sh
mosquitto_sub -h 127.0.0.1 -u fpms -P "$FPMS_MQTT_PASS" \
  -t 'fpms/rover1/telemetry/camera' -C 20
```

A non-empty `detections` array, from a frame you controlled, is the only
evidence the whole chain is intact. **`systemctl status` proves nothing here**,
and neither does live video: both are green on a rover that has never run a
single inference.

Run it again after ten minutes of patrolling. Thermal throttling and a
per-frame `inference failed:` loop both look fine at second zero.

## Three things to know before you touch anything here

- **`end2end=false` is mandatory.** `end2end=True` emits `[1,300,6]` whose top-k
  op **segfaults on the RK3588 NPU**. This is decided at conversion time, on an
  x86 box, long before the rover sees it.
- **Classes are already sigmoided and DFL is already folded in — do not sigmoid
  again.** It does not crash and does not empty the output. It quietly
  compresses every confidence toward 0.5.
- **The model is not in git.** `yolo26n-rk3588.rknn` exists only on the old Pi.
  Nothing here can rebuild it, and unlike the three `/usr/local/bin` scripts this
  project already lost and reconstructed, **a model cannot be reconstructed from
  prose.** Copy it off that card today: `docs/NPU.md` has the four commands.

## Status

**Nothing in this layer has run on an RK3588S. The board has not arrived.**
Every latency figure, core-mask recommendation, device path and sysfs node here
is derived from documentation and from the old Pi — never measured on the
target. `docs/NPU.md` carries the full NOT FIXED register, in the register of
`docs/FAILURE_MODES.md`. Read it before a competition.
