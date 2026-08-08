# The NPU version chain

This directory exists to hold something the project has never had: **a written
record of which NPU versions were running when the rover worked.**

`bench_npu.py` reads the driver version at runtime, off the live board, with
`cat /sys/kernel/debug/rknpu/version`. That line is the tell. It means there was
no build record — so when a rover detected nothing, there was nothing to compare
it against, and "the NPU is broken" and "the NPU is fine, the model is wrong"
looked identical.

`scripts/25-npu-runtime.sh` now writes `/etc/fpms/npu-versions.json` at build
time. This file explains what is in it, how to read the one value that cannot go
in it, and what to do when the numbers disagree.

> **Nothing in this document has been verified on an RK3588S.** The board has
> not arrived. Every number below is either read out of this repository's own
> config, or marked `MEASURE ME`. There are no invented versions here, and the
> known-good table is empty on purpose — see §6.

---

## 1. The chain

```
  [1]  kernel rknpu driver          in the Rockchip BSP kernel
       /sys/kernel/debug/rknpu/version        NO mainline driver exists
              ↕  MUST MATCH
  [2]  /usr/lib/librknnrt.so        the userspace runtime
              ↕  MUST MATCH
  [3]  rknn-toolkit-lite2           the aarch64 Python wheel (not on PyPI)
              ↕  MUST MATCH
  [4]  yolo26n-rk3588.rknn          the model container version
```

Link 4 is the one NPU_SPEC.md's three-link diagram leaves out, and it bites just
as hard. A `.rknn` file carries the container version of the **rknn-toolkit2**
that exported it, and a runtime older than that container cannot load it —
`Invalid RKNN model version`. The FPMS model was built on an unrecorded machine
with an unrecorded toolkit2, and lives only on the old Pi, so link 4 is currently
**unknown in both directions**. Agent 3's model registry owns that half; it is
named here so the coupling is not forgotten when links 1–3 are made to agree.

FPMS-OS controls links 2 and 3. It does **not** control link 1 (that is a
property of the base image's kernel) and does not yet control link 4.

---

## 2. Why a mismatch is invisible — the actual mechanism

Three separate things have to go wrong for a version mismatch to reach the
operator as "the rover is fine", and in this codebase all three are true today:

**a. `init_runtime()` returns an int. It does not raise.**

```python
r = RKNNLite()
r.load_rknn(y.MODEL); r.init_runtime()      # bench_npu.py, verbatim
```

Neither return value is checked. A `-1` from `init_runtime()` — which is what a
driver/runtime mismatch produces — flows straight past. Code that "handles
errors" with `try/except` around this does not handle this error at all, because
there is no exception. **Anywhere in this layer that calls `init_runtime()`, the
return value must be compared to `0`.**

**b. The runtime's complaint goes to stderr, not to Python.**

librknnrt prints its diagnostics (`E RKNN: ...`) from C, to the process's
stderr. Under systemd that lands in the journal for a unit nobody is tailing,
interleaved with the RKNN banner that prints on every single run — so it does not
even look unusual.

**c. The agent swallows what is left.**

`fpms_rover_agent.py` wraps import, `load_rknn`, `init_runtime` and label
loading in **one** `try/except` that logs

```
NPU unavailable (...); streaming without detection
```

once, then runs forever. Every unit reports `active`. The camera streams. The
dashboard is alive. The rover detects nothing.

So the observable difference between "correctly version-matched" and "off by one
patch release" is: **nothing at all, until a fire is missed.** That is why this
layer records versions instead of trusting them.

---

## 3. How to read each version on a running board

### [1] Kernel driver

```bash
sudo cat /sys/kernel/debug/rknpu/version
```

`sudo` is not optional: systemd mounts `/sys/kernel/debug` as `0700 root:root`,
so `ubuntu` cannot even traverse into it. See the debugfs section at the bottom
of `overlay/etc/udev/rules.d/99-fpms-npu.rules` for the three ways to fix that
and which one to prefer.

Corroborate it — the probe line is written at module init and survives in the
ring buffer:

```bash
sudo dmesg | grep -i rknpu
```

`VERIFY ON HARDWARE:` the exact text of both is unconfirmed. Record what they
actually print, verbatim, in §6.

### [2] librknnrt.so

The version is a string compiled into the binary:

```bash
strings /usr/lib/librknnrt.so | grep -i 'librknnrt version'
```

Expected shape: `librknnrt version: <x.y.z> (<commit>@<build timestamp>)`.
Stage 25 extracts exactly this at build time (without `strings`, which is not
guaranteed present) into `librknnrt.version` and `librknnrt.version_string`.

If that grep comes back empty on a file that is a valid ELF, **do not assume the
version** — record `MEASURE ME` and say so.

### [3] The wheel

```bash
pip3 show rknn-toolkit-lite2
python3 -c "import importlib.metadata as m; print(m.version('rknn-toolkit-lite2'))"
```

### [1] and [2] together, without root

This is the useful one, and it needs no debugfs:

```bash
python3 -c "
from rknnlite.api import RKNNLite
r = RKNNLite()
print('load_rknn    ->', r.load_rknn('/home/ubuntu/yolo/yolo26n-rk3588.rknn'))
print('init_runtime ->', r.init_runtime())
print(r.get_sdk_version())"
```

`get_sdk_version()` reports an **API** line (that is link 2, librknnrt) and a
**DRV** line (that is link 1, the kernel module). Both zeros above mean success;
any non-zero is the failure the agent would have swallowed.

Its limitation is exactly where you need it most: it requires a loaded model and
a successful `init_runtime()`, so it tells you nothing when the NPU is the thing
that is broken. Use debugfs for that case.
`VERIFY ON HARDWARE:` that `get_sdk_version()` reports DRV on the pinned
runtime version.

### The build record

```bash
cat /etc/fpms/npu-versions.json
```

Written by stage 25. `driver.expected` is `null` until somebody measures it —
see §7.

---

## 4. Finding a matching set

Everything comes out of **`airockchip/rknn-toolkit2`**, and the tag *is* the
version. `config/fpms-os.conf` pins one tag and derives both URLs from it:

```
RKNN_TOOLKIT2_TAG="v2.3.0"
  rknn-toolkit-lite2/packages/rknn_toolkit_lite2-2.3.0-cp310-cp310-manylinux_2_17_aarch64...whl
  rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so
```

That single-tag derivation is deliberate and is the most valuable property of
the config: **links 2 and 3 cannot drift apart by accident.** If you ever change
one URL without the other, you have hand-built the mismatch this whole document
is about. Change `RKNN_TOOLKIT2_TAG` and nothing else.

Two constraints on the wheel filename, both hard:

- `cp310` — Ubuntu 22.04 system Python is 3.10, pinned by ROS Humble's
  `cpython-310-aarch64` rclpy extensions (SPEC.md §1). A `cp311` wheel will not
  install and, if forced, will not import.
- `aarch64` — obviously; stage 25 checks `uname -m` before it tries, so a
  mis-configured chroot fails with a sentence instead of a pip traceback.

For link 1, the authoritative statement of "which driver versions does this SDK
require" is in the repo **at the tag you are pinning** — read it there, not from
memory and not from a blog:

```
https://github.com/airockchip/rknn-toolkit2/tree/<TAG>/rknpu2/         README + doc/
https://github.com/airockchip/rknn-toolkit2/blob/<TAG>/CHANGELOG.md
```

The general shape of the rule is that a given librknnrt states a **minimum**
driver version and is expected to work with that driver or newer.
`VERIFY:` this direction, and the specific minimum for the pinned tag, against
the document above before relying on it. This document deliberately does not
quote a number for it.

**You cannot pip your way out of a link-1 mismatch.** The `rknpu` module is part
of the BSP kernel; changing it means changing the kernel package
(`dpkg -l | grep linux-image`), which changes the Mali driver and `bcmdhd` with
it, on an image whose whole premise is that the vendor kernel is not ours to
rebuild. On a fixed BSP the cheap direction is always the other one: move
**links 2 and 3 down** to a tag the shipped driver supports.

---

## 5. When they disagree

| What you see | Which link | What to do |
|---|---|---|
| `init_runtime()` returns non-zero, `load_rknn()` returned 0 | 1 ↔ 2 | Read driver + API versions (§3). If the driver is older than the runtime requires, **downgrade `RKNN_TOOLKIT2_TAG`** and rebuild. Do not upgrade the kernel to chase it. |
| `load_rknn()` returns non-zero, or `Invalid RKNN model version` | 4 | The model was exported by a *newer* toolkit2 than this runtime. Either re-export with the pinned toolkit2 (agent 5's `npu/convert/`) or raise the tag. Never both at once. |
| `from rknnlite.api import RKNNLite` raises `OSError`/`cannot open shared object` | 2 | `/usr/lib/librknnrt.so` is missing, zero-byte, or the wrong architecture. `ls -l` it, then `sudo ldconfig`. Stage 25's ELF check exists because a 404'd download used to leave a zero-byte file here that `ldconfig` happily indexed. |
| Import works as `root`, fails as `ubuntu` | not a version problem | Device permissions. See `99-fpms-npu.rules`. |
| Everything returns 0, boxes are nonsense | not a version problem | Decode, not versions — NPU_SPEC.md §1: DFL is folded in and classes are already sigmoided; preprocessing is a **stretch**, so box rescaling is a per-axis ratio, not a uniform scale plus padding. Agent 6 owns this. |
| Versions agree, latency climbs over minutes | not a version problem | RK3588 thermal throttling. Agent 4. |

The ordering matters: **establish links 1–3 before believing anything about
link 4 or about decode.** A mismatched runtime can produce output that decodes
into plausible-looking garbage, and chasing that as a decode bug has cost this
project time before in other subsystems.

---

## 6. Known-good combinations

**There are none that have been verified on hardware, and this table will not
pretend otherwise.**

| Driver [1] | librknnrt [2] | wheel [3] | model [4] | Status | Evidence |
|---|---|---|---|---|---|
| `MEASURE ME` | 2.3.0 *(expected)* | 2.3.0 *(expected)* | unknown | **NOT TESTED** | Derived from `RKNN_TOOLKIT2_TAG="v2.3.0"` in `config/fpms-os.conf`. Links 2 and 3 are matched *by construction* — same tag — but nothing here has been run on an RK3588S. The `2.3.0` values are what the pinned URLs claim; stage 25 records what was actually installed. |
| unknown | unknown | unknown | `yolo26n-rk3588.rknn` | **the old Pi, which worked** | No record exists. This is the entire reason this directory exists. If that card is still readable, its values are recoverable — see below. |

### Recovering the one set we know worked

The old Pi ran a rover that detected fires. Nobody wrote down what was on it. If
that card still boots, this is a ten-minute job that retires the largest unknown
in this layer:

```bash
ssh ubuntu@<old-pi>
sudo cat /sys/kernel/debug/rknpu/version
strings /usr/lib/librknnrt.so | grep -i 'librknnrt version'
pip3 show rknn-toolkit-lite2
sha256sum ~/yolo/yolo26n-rk3588.rknn
uname -r
```

Paste the output into the table above as a new row, with the date and who ran
it. A row with real output beats every inference in this document.

---

## 7. Recording a measurement

When link 1 has been read off real hardware:

1. Add the row to §6, verbatim, with a date.
2. Pin the expectation so the image can check it at boot. In
   `config/fpms-os.conf` (agent A's file):
   ```sh
   # MEASURED <date> on <board>: sudo cat /sys/kernel/debug/rknpu/version
   RKNN_EXPECTED_DRIVER_VERSION="..."
   ```
   and add `RKNN_EXPECTED_DRIVER_VERSION="$RKNN_EXPECTED_DRIVER_VERSION"` to
   `build.sh`'s `in_chroot()` environment list — it runs `env -i`, so a variable
   that is not in that list does not reach stage 25.
3. Rebuild. Stage 25 puts it in `/etc/fpms/npu-versions.json` as
   `driver.expected`; agent 7's `fpms-npu-selftest` compares it against the live
   value on every boot and reports the mismatch as a FAIL.

Until step 2 happens, `driver.expected` is `null` and the boot-time check has
nothing to compare against. That is the honest state, and it is recorded as such
in the JSON rather than defaulted to something plausible — a wrong expectation
that passes is worse than no expectation that says so.

---

## 8. What lives where

| Path | Owner | What it is |
|---|---|---|
| `config/fpms-os.conf` | agent A | the pins: `RKNN_TOOLKIT2_TAG` and the two URLs derived from it |
| `scripts/25-npu-runtime.sh` | agent 1 | installs links 2 and 3, writes the build record |
| `overlay/etc/udev/rules.d/99-fpms-npu.rules` | agent 1 | device access for `ubuntu` |
| `/etc/fpms/npu-versions.json` | written by stage 25 | the build record |
| `overlay/usr/local/bin/fpms-npu-selftest` | agent 7 | reads the record, compares against the live board |
| `overlay/etc/fpms/models.json` | agent 3 | link 4, the model registry |
| `npu/convert/` | agent 5 | the x86-only exporter that determines link 4 |
