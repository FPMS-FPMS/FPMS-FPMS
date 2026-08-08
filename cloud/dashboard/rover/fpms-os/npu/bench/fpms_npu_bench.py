#!/usr/bin/env python3
"""fpms_npu_bench.py - what the NPU actually sustains, and on how many cores.

Usage (on the rover, as ubuntu):
    python3 fpms_npu_bench.py                     full sweep, ~6 minutes
    python3 fpms_npu_bench.py --seconds 30        quicker, less thermally honest
    python3 fpms_npu_bench.py --masks 0,0_1_2     only the comparison you want
    python3 fpms_npu_bench.py --json out.json     machine-readable
    python3 fpms_npu_bench.py --quiet --json -    JSON on stdout, nothing else

Exit: 0 the rover's duty cycle is met, 1 met only marginally or with
      warnings, 2 not met or the bench could not run.


WHY THIS EXISTS, AND WHAT IT MEASURES THAT bench_npu.py DID NOT
==============================================================
`cloud/dashboard/rover/bench_npu.py` established the shape of the question:
capture, preprocess, infer, decode, encode, timed per stage over 30 frames on
the old Pi. Its NPU number - about 20 ms per inference - is the only NPU
measurement this project owns, and everything below is calibrated against it.

Three things it could not tell you, all of which decide the design:

1. SUSTAINED latency, not first-inference latency. RK3588 throttles hard and
   silently: the SoC pulls its devfreq ceiling down as it heats, and a warm
   number taken 30 frames in is a number from before that happened. Thirty
   frames at 6 fps is five seconds. Nothing thermal is visible in five
   seconds. This runs long enough to see the knee, and reports p50/p90/p99/
   max plus the drift between the first and last fifth of the run.

2. CORE MASKS. init_runtime() with no core_mask uses CORE 0 ONLY - two thirds
   of a 6 TOPS NPU idle, and no log line anywhere mentions it. Whether
   0_1_2 is actually faster than 0 for a single YOLO graph is a MEASUREMENT,
   not a fact: the three cores can also be used as three independent
   single-core contexts, which is better for three models and worse for one.
   Agent 4's tuning decision should rest on this output, not on assumption.

3. WHETHER THE ROVER'S ACTUAL DUTY CYCLE IS MET. This is the part that makes
   the number mean something. The camera caps at 30 fps IN HARDWARE (the
   sensor's USB descriptor), ships at FPMS_CAMERA_FPS=6 deliberately -
   higher settings saturated WiFi and starved the LiDAR feed - and
   FPMS_INFER_EVERY=2. So the requirement is about THREE INFERENCES PER
   SECOND, a 333 ms period. A benchmark that reports "142 fps!" without
   saying whether 3/s is met under sustained thermal load has answered a
   question nobody asked. This one says it in one line, plainly.


WHAT IT WILL NEVER DO
=====================
  - move the rover. It touches no drive topic, opens no serial port, and
    imports nothing that does.
  - stop or reconfigure any service. If fpms-npud is running it will contend
    with this bench for the device; the bench SAYS SO and prints the command,
    and leaves the decision to a human. (bench_npu.py's driver stopped
    fpms-rover-agent automatically. That is fine for a laptop-driven
    profiling run and wrong for a tool that may be run on a live rover.)


HONESTY
=======
NONE OF THIS HAS RUN ON AN RK3588S. The board has not arrived. The thermal
and devfreq paths below are written against Rockchip's documented sysfs and
are marked MEASURE ME where the exact node name could not be confirmed
without hardware. The one number here that is real is the ~20 ms from the old
Pi. Every threshold is derived from the 333 ms duty period, which IS known,
rather than from a plausible-looking guess.
"""

import argparse
import glob
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time

CONFIG = "/etc/fpms/config.env"
DRV_LOAD = "/sys/kernel/debug/rknpu/load"
DRV_VERSION = "/sys/kernel/debug/rknpu/version"

DEFAULT_MASKS = ["0", "0_1", "0_1_2"]
MASK_ATTR = {"0": "NPU_CORE_0", "0_1": "NPU_CORE_0_1",
             "0_1_2": "NPU_CORE_0_1_2", "auto": "NPU_CORE_AUTO"}

# The knee we are looking for. 15% is not arbitrary: below ~10% the run-to-run
# noise on a shared SoC swamps it, and above ~20% the frame budget is already
# in trouble, so a threshold there would only ever confirm a problem already
# visible elsewhere. MEASURE ME once there is real hardware to calibrate on.
DRIFT_WARN_PCT = 15.0

# Fraction of the run used for the "start" and "end" thermal windows.
WINDOW = 0.2


# ---------------------------------------------------------------- helpers ---
def read_text(path):
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return None


def load_config():
    cfg = {}
    try:
        with open(CONFIG) as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                cfg[k.strip()] = v.strip().strip('"').strip("'")
    except OSError:
        pass
    cfg.update({k: v for k, v in os.environ.items() if k.startswith("FPMS_")})
    return cfg


def pct(sorted_vals, q):
    """Nearest-rank percentile. No interpolation: with a few thousand samples
    the difference is noise, and an integer index is one less thing to be
    subtly wrong about in a number people will quote."""
    if not sorted_vals:
        return float("nan")
    k = max(0, min(len(sorted_vals) - 1,
                   int(math.ceil(q / 100.0 * len(sorted_vals))) - 1))
    return sorted_vals[k]


def pearson(xs, ys):
    """Correlation, in pure Python - numpy is present but this is clearer
    about what it does and about returning nan rather than raising when one
    series is flat (a board that never warmed up)."""
    n = len(xs)
    if n < 3:
        return float("nan")
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 0 or syy <= 0:
        return float("nan")
    return sxy / math.sqrt(sxx * syy)


def npu_temp_c():
    """Hottest of the NPU/SoC zones, in Celsius, or None.

    We take the MAX rather than a single named zone: the RK3588 vendor DT
    declares soc/gpu/npu zones, throttling is driven by whichever trips
    first, and MEASURE ME - the exact set is BSP-dependent. Confirm with
    `grep . /sys/class/thermal/thermal_zone*/type`.
    """
    best = None
    for zpath in glob.glob("/sys/class/thermal/thermal_zone*"):
        ztype = read_text(os.path.join(zpath, "type")) or ""
        if not any(k in ztype.lower() for k in ("npu", "soc", "cpu", "center")):
            continue
        raw = read_text(os.path.join(zpath, "temp"))
        try:
            c = int(raw) / 1000.0
        except (TypeError, ValueError):
            continue
        best = c if best is None else max(best, c)
    return best


def core_load():
    """Per-core utilisation, or None if debugfs is unreadable (it is root-only
    by default, and this bench is meant to run as ubuntu). Vendor format:
        NPU load:  Core0: 12%, Core1:  0%, Core2:  0%,
    """
    text = read_text(DRV_LOAD)
    if text is None:
        return None
    found = re.findall(r"Core(\d+)\s*:\s*(\d+)\s*%", text)
    if found:
        return {int(c): int(v) for c, v in found}
    m = re.search(r"(\d+)\s*%", text)
    return {0: int(m.group(1))} if m else None


def devfreq_ceiling():
    """(current ceiling Hz, top OPP Hz) for the NPU, or None.

    Thermal throttling shows up as a LOWERED CEILING, not as a low current
    frequency - an idle NPU sits at its minimum OPP quite legitimately, and
    reading cur_freq would call that throttling every time.
    MEASURE ME: node name unconfirmed (expected ~fdab0000.npu).
    """
    for dev in glob.glob("/sys/class/devfreq/*npu*"):
        mx = read_text(os.path.join(dev, "max_freq"))
        avail = read_text(os.path.join(dev, "available_frequencies"))
        if not (mx and avail):
            continue
        try:
            return int(mx), max(int(x) for x in avail.split())
        except ValueError:
            continue
    return None


def daemon_active():
    try:
        p = subprocess.run(["systemctl", "is-active", "fpms-npud.service"],
                           capture_output=True, text=True, timeout=10)
        return p.stdout.strip() == "active"
    except Exception:  # noqa: BLE001
        return False


# ------------------------------------------------------------- the phases ---
def run_phase(RKNNLite, np, model, mask, seconds, label, quiet):
    """Flat out on one core mask for `seconds`, sampling temperature and load.

    Flat out, not paced: the point is to provoke the thermal behaviour that a
    paced run hides. The duty-cycle question is answered separately, by
    run_duty(), and the two must not be conflated - a rover that meets 3/s
    while thermally saturated is a different claim from one that meets it cold.
    """
    r = RKNNLite()
    if r.load_rknn(model) != 0:
        raise RuntimeError("load_rknn(%s) failed" % model)
    attr = MASK_ATTR.get(mask)
    cm = getattr(RKNNLite, attr, None) if attr else None
    rc = r.init_runtime() if cm is None else r.init_runtime(core_mask=cm)
    if rc != 0:
        raise RuntimeError("init_runtime(core_mask=%s) returned %r" % (attr, rc))

    rng = np.random.default_rng(20260808)
    img = rng.integers(0, 255, (1, 640, 640, 3), dtype=np.uint8)

    # Warm-up, discarded. The first inference includes lazy allocation and a
    # cold cache and is never representative - reporting it is how a "12 ms
    # NPU" turns into a 40 ms one in the field.
    t0 = time.perf_counter()
    r.inference(inputs=[img])
    warmup_ms = (time.perf_counter() - t0) * 1000.0

    samples, temps, loads = [], [], {}
    t_end = time.time() + seconds
    last_probe = 0.0
    shape = None
    while time.time() < t_end:
        a = time.perf_counter()
        out = r.inference(inputs=[img])
        dt = (time.perf_counter() - a) * 1000.0
        now = time.time()
        if shape is None:
            shape = list(np.asarray(out[0]).shape)
        temp = None
        if now - last_probe > 0.5:      # sysfs reads are not free; 2 Hz is ample
            last_probe = now
            temp = npu_temp_c()
            cl = core_load()
            if cl:
                for c, v in cl.items():
                    loads[c] = max(loads.get(c, 0), v)
            if temp is not None:
                temps.append((now, temp))
        samples.append((now, dt))
        if not quiet and len(samples) % 200 == 0:
            sys.stderr.write("\r    %s: %d inferences, %.1f ms last, %s C   "
                             % (label, len(samples), dt,
                                ("%.1f" % temps[-1][1]) if temps else "?"))
            sys.stderr.flush()
    if not quiet:
        sys.stderr.write("\r" + " " * 72 + "\r")
    try:
        r.release()
    except Exception:  # noqa: BLE001
        pass

    lat = sorted(d for _, d in samples)
    n = len(samples)
    w = max(1, int(n * WINDOW))
    head = statistics.median([d for _, d in samples[:w]])
    tail = statistics.median([d for _, d in samples[-w:]])
    drift = ((tail - head) / head * 100.0) if head else float("nan")

    # Correlate latency against the temperature at the time it was measured.
    # This is what separates "it got slower" from "it got slower BECAUSE it
    # got hotter" - the second has a fix that is not a software fix.
    corr = float("nan")
    if len(temps) >= 3:
        xs, ys = [], []
        ti = 0
        for ts, dt in samples:
            while ti + 1 < len(temps) and temps[ti + 1][0] <= ts:
                ti += 1
            xs.append(temps[ti][1])
            ys.append(dt)
        corr = pearson(xs, ys)

    return {
        "mask": mask, "attr": attr, "seconds": seconds,
        "n": n, "shape": shape, "warmup_ms": warmup_ms,
        "p50": pct(lat, 50), "p90": pct(lat, 90), "p99": pct(lat, 99),
        "max": lat[-1] if lat else float("nan"),
        "min": lat[0] if lat else float("nan"),
        "mean": statistics.fmean([d for _, d in samples]) if samples else float("nan"),
        "throughput_per_s": n / float(seconds) if seconds else float("nan"),
        "head_median": head, "tail_median": tail, "drift_pct": drift,
        "temp_start": temps[0][1] if temps else None,
        "temp_end": temps[-1][1] if temps else None,
        "temp_max": max(t for _, t in temps) if temps else None,
        "temp_latency_corr": corr,
        "core_load_peak": loads or None,
        "throttled": (drift == drift and drift > DRIFT_WARN_PCT),
    }


def run_duty(RKNNLite, np, model, mask, seconds, target_hz, quiet):
    """The question that actually matters: can it hold the rover's cadence?

    Paced at the real rate, from a board already warmed by the sweep above -
    which is the honest starting condition, not a cold one. A deadline miss
    here is a dropped detection on a moving rover.
    """
    r = RKNNLite()
    if r.load_rknn(model) != 0:
        raise RuntimeError("load_rknn(%s) failed" % model)
    attr = MASK_ATTR.get(mask)
    cm = getattr(RKNNLite, attr, None) if attr else None
    rc = r.init_runtime() if cm is None else r.init_runtime(core_mask=cm)
    if rc != 0:
        raise RuntimeError("init_runtime(core_mask=%s) returned %r" % (attr, rc))

    rng = np.random.default_rng(20260808)
    img = rng.integers(0, 255, (1, 640, 640, 3), dtype=np.uint8)
    r.inference(inputs=[img])

    period = 1.0 / target_hz
    lat, misses = [], 0
    n = max(1, int(seconds * target_hz))
    next_at = time.perf_counter()
    for _ in range(n):
        now = time.perf_counter()
        if now < next_at:
            time.sleep(next_at - now)
        a = time.perf_counter()
        r.inference(inputs=[img])
        dt = (time.perf_counter() - a) * 1000.0
        lat.append(dt)
        if dt > period * 1000.0:
            misses += 1
        next_at += period
    try:
        r.release()
    except Exception:  # noqa: BLE001
        pass

    s = sorted(lat)
    return {
        "mask": mask, "target_hz": target_hz, "period_ms": period * 1000.0,
        "n": len(lat), "p50": pct(s, 50), "p99": pct(s, 99),
        "max": s[-1] if s else float("nan"),
        "misses": misses,
        "miss_pct": 100.0 * misses / len(lat) if lat else float("nan"),
        "headroom_pct": (100.0 * (1.0 - pct(s, 99) / (period * 1000.0))
                         if s else float("nan")),
        "temp_end": npu_temp_c(),
    }


# ------------------------------------------------------------------ report ---
def render(res, quiet):
    if quiet:
        return
    print("=" * 72)
    print(" FPMS-OS NPU benchmark    %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 72)
    print("  model     %s" % res["model"])
    print("  driver    %s" % (res["driver_version"] or "unreadable (root-only debugfs)"))
    print("  shape     %s" % (res["phases"][0]["shape"] if res["phases"] else "?"))
    if res["contention"]:
        print("  WARNING   fpms-npud is ACTIVE and competing for the NPU.")
        print("            Every number below includes that contention. To")
        print("            measure the device alone, on a rover that is not")
        print("            on a mission:")
        print("            sudo systemctl stop fpms-npud   # remember to start it")
    print("-" * 72)
    print("  sustained latency, flat out (ms)")
    print("  %-8s %6s %7s %7s %7s %7s %8s %7s %6s"
          % ("mask", "n", "p50", "p90", "p99", "max", "inf/s", "drift%", "degC"))
    for p in res["phases"]:
        print("  %-8s %6d %7.1f %7.1f %7.1f %7.1f %8.1f %+7.1f %6s"
              % (p["mask"], p["n"], p["p50"], p["p90"], p["p99"], p["max"],
                 p["throughput_per_s"], p["drift_pct"],
                 ("%.0f" % p["temp_end"]) if p["temp_end"] is not None else "?"))
        if p["core_load_peak"]:
            print("           cores: %s" % ", ".join(
                "Core%d peak %d%%" % (c, v)
                for c, v in sorted(p["core_load_peak"].items())))
    print("")
    for p in res["phases"]:
        if p["throttled"]:
            corr = p["temp_latency_corr"]
            print("  THROTTLING on mask %s: median rose %.1f%% from the first"
                  % (p["mask"], p["drift_pct"]))
            print("    fifth of the run to the last, %s to %s degC."
                  % (("%.0f" % p["temp_start"]) if p["temp_start"] else "?",
                     ("%.0f" % p["temp_end"]) if p["temp_end"] else "?"))
            if corr == corr:
                print("    Latency/temperature correlation %+.2f - %s"
                      % (corr, "the heat is the cause" if corr > 0.5
                         else "weakly correlated; look for another cause"))
            print("    A first-inference number would have missed this "
                  "entirely.")
    if res["best_mask"]:
        b = res["by_mask"][res["best_mask"]]
        base = res["by_mask"].get("0")
        line = "  BEST MASK: %s at p99 %.1f ms" % (res["best_mask"], b["p99"])
        if base and res["best_mask"] != "0" and base["p99"] > 0:
            line += " (%.2fx better than core 0 only)" % (base["p99"] / b["p99"])
        print(line)
        if base and res["best_mask"] == "0":
            print("    Core 0 alone won. That is a real result and it means")
            print("    the three cores are better spent on three parallel")
            print("    models than on one - but confirm it did not simply")
            print("    thermally outlast the multi-core runs by re-running")
            print("    with --masks 0_1_2,0_1,0 to reverse the order.")
    print("-" * 72)

    d = res["duty"]
    if d:
        print("  THE QUESTION THAT MATTERS")
        print("  The rover needs %.1f inferences/s (FPMS_CAMERA_FPS=%s /"
              % (d["target_hz"], res["camera_fps"]))
        print("  FPMS_INFER_EVERY=%s), a %.0f ms period. Camera caps at 30 fps"
              % (res["infer_every"], d["period_ms"]))
        print("  in hardware and ships at 6 deliberately - higher settings")
        print("  saturated WiFi and starved the LiDAR feed.")
        print("")
        print("  Paced run on mask %s, from a warm board: p50 %.1f ms, p99 "
              "%.1f ms," % (d["mask"], d["p50"], d["p99"]))
        print("  %d/%d deadline misses (%.1f%%), %.0f%% headroom at p99."
              % (d["misses"], d["n"], d["miss_pct"], d["headroom_pct"]))
        print("")
        print("  VERDICT: %s" % res["verdict"])
    print("=" * 72)
    print("  Nothing here has run on an RK3588S - the board has not arrived.")
    print("  The only prior measurement is ~20 ms/inference on the old Pi")
    print("  (bench_npu.py). Treat the first real run as the calibration.")
    print("=" * 72)


def main():
    ap = argparse.ArgumentParser(
        description="Sustained NPU latency, core-mask comparison, and whether "
                    "the rover's real 3 inferences/second is met.")
    ap.add_argument("--seconds", type=float, default=90.0,
                    help="flat-out seconds PER MASK (default 90; below ~60 "
                         "the thermal knee may not appear at all)")
    ap.add_argument("--duty-seconds", type=float, default=60.0,
                    help="seconds for the paced duty-cycle phase (default 60)")
    ap.add_argument("--masks", default=",".join(DEFAULT_MASKS),
                    help="comma-separated: 0,0_1,0_1_2,auto")
    ap.add_argument("--model", default=None,
                    help="path to the .rknn (default: "
                         "$FPMS_YOLO_DIR/yolo26n-rk3588.rknn)")
    ap.add_argument("--json", default=None,
                    help="write JSON here ('-' for stdout)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    cfg = load_config()
    model = args.model or os.path.join(
        cfg.get("FPMS_YOLO_DIR", "/home/ubuntu/yolo"), "yolo26n-rk3588.rknn")
    if not os.path.exists(model):
        print("no model at %s\n"
              "The .rknn is not in the git repository - it exists only on the "
              "old Pi. Copy it across, or pass --model." % model,
              file=sys.stderr)
        return 2

    try:
        import numpy as np
        from rknnlite.api import RKNNLite
    except Exception as exc:  # noqa: BLE001
        print("cannot import the RKNN runtime: %s\n"
              "This bench only runs on the rover: the wheel is aarch64, "
              "Rockchip's, and not on PyPI. Nothing here is simulatable on a "
              "build machine." % exc, file=sys.stderr)
        return 2

    masks = [m.strip() for m in args.masks.split(",") if m.strip()]
    bad = [m for m in masks if m not in MASK_ATTR]
    if bad:
        print("unknown mask(s): %s (use 0, 0_1, 0_1_2, auto)" % ", ".join(bad),
              file=sys.stderr)
        return 2

    contention = daemon_active()
    if contention and not args.quiet:
        print("fpms-npud is active; results include contention. Continuing "
              "anyway - this tool does not stop services.", file=sys.stderr)

    phases = []
    for m in masks:
        if not args.quiet:
            print("  running mask %s for %.0f s ..." % (m, args.seconds),
                  file=sys.stderr)
        try:
            phases.append(run_phase(RKNNLite, np, model, m, args.seconds,
                                    "mask " + m, args.quiet))
        except Exception as exc:  # noqa: BLE001
            phases.append({"mask": m, "attr": MASK_ATTR.get(m),
                           "error": "%s: %s" % (type(exc).__name__, exc),
                           "n": 0, "shape": None, "throttled": False,
                           "p50": float("nan"), "p90": float("nan"),
                           "p99": float("nan"), "max": float("nan"),
                           "min": float("nan"), "mean": float("nan"),
                           "drift_pct": float("nan"), "warmup_ms": float("nan"),
                           "throughput_per_s": float("nan"),
                           "head_median": float("nan"),
                           "tail_median": float("nan"),
                           "temp_start": None, "temp_end": None,
                           "temp_max": None,
                           "temp_latency_corr": float("nan"),
                           "core_load_peak": None, "seconds": args.seconds})

    by_mask = {p["mask"]: p for p in phases}
    ok_phases = [p for p in phases if p.get("n")]
    # Ranked on p99, not on the mean. The mean hides exactly the tail that
    # drops a frame, and a dropped frame on a moving rover is a missed fire.
    best = min(ok_phases, key=lambda p: p["p99"])["mask"] if ok_phases else None

    try:
        camera_fps = float(cfg.get("FPMS_CAMERA_FPS", 6))
        infer_every = float(cfg.get("FPMS_INFER_EVERY", 2))
    except ValueError:
        camera_fps, infer_every = 6.0, 2.0
    target_hz = camera_fps / max(infer_every, 1.0)

    duty, verdict = None, "NOT MEASURED"
    if best:
        if not args.quiet:
            print("  duty-cycle phase on mask %s at %.1f Hz for %.0f s ..."
                  % (best, target_hz, args.duty_seconds), file=sys.stderr)
        try:
            duty = run_duty(RKNNLite, np, model, best, args.duty_seconds,
                            target_hz, args.quiet)
        except Exception as exc:  # noqa: BLE001
            duty = {"error": "%s: %s" % (type(exc).__name__, exc),
                    "mask": best, "target_hz": target_hz,
                    "period_ms": 1000.0 / target_hz}

    if duty and "error" not in duty:
        if duty["misses"] == 0 and duty["headroom_pct"] >= 50:
            verdict = ("MET, comfortably - %.1f inferences/s with %.0f%% of "
                       "the %.0f ms period still free at p99."
                       % (target_hz, duty["headroom_pct"], duty["period_ms"]))
            code = 0
        elif duty["miss_pct"] <= 1.0 and duty["headroom_pct"] > 0:
            verdict = ("MET, but with only %.0f%% headroom at p99 (%d/%d "
                       "deadline misses). One more thermal soak, one more "
                       "process on the SoC, or a hotter day eats this."
                       % (duty["headroom_pct"], duty["misses"], duty["n"]))
            code = 1
        else:
            verdict = ("NOT MET - %.1f%% of inferences overran the %.0f ms "
                       "period. The rover will silently skip detections, "
                       "which looks exactly like an empty scene. Try a wider "
                       "core mask, improve cooling, or lower "
                       "FPMS_CAMERA_FPS/raise FPMS_INFER_EVERY - and "
                       "re-measure the LiDAR rate at the dashboard if you "
                       "touch the camera."
                       % (duty["miss_pct"], duty["period_ms"]))
            code = 2
    else:
        code = 2

    if any(p.get("throttled") for p in phases) and code == 0:
        code = 1

    res = {
        "ts": time.time(),
        "source": "fpms_npu_bench",
        "model": model,
        "driver_version": read_text(DRV_VERSION),
        "devfreq": devfreq_ceiling(),
        "contention": contention,
        "camera_fps": cfg.get("FPMS_CAMERA_FPS", "6"),
        "infer_every": cfg.get("FPMS_INFER_EVERY", "2"),
        "target_hz": target_hz,
        "phases": phases, "by_mask": by_mask,
        "best_mask": best, "duty": duty, "verdict": verdict,
        "exit": code,
    }

    render(res, args.quiet or args.json == "-")
    if args.json == "-":
        print(json.dumps(res, indent=2, default=str))
    elif args.json:
        with open(args.json, "w") as fh:
            json.dump(res, fh, indent=2, default=str)
    return code


if __name__ == "__main__":
    sys.exit(main())
