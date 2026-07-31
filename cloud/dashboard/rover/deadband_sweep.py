#!/usr/bin/env python3
"""Motor deadband sweep for the Yahboom MicroROS Board V2.0 (ESP32-S3) rover.

WHY THIS EXISTS
  fpms_teleop.py needs a MIN_CMD floor: a commanded speed below which it
  should never bother asking the board to move, because the board can't
  actually do anything useful with it. This script measures that floor by
  stepping /cmd_vel up from zero and watching /odom_raw for the point where
  the response becomes real, proportional motion.

THE HAZARD THIS SCRIPT IS BUILT AROUND
  MEASURED 2026-07-31 (see fpms_teleop.py CALIBRATION comments): commanding
  linear.x = 0.0041 produced NO motion for ~3s and then a 290mm BACKWARD
  lurch plus unintended rotation. The board firmware appears to run a
  closed-loop wheel controller whose integrator winds up while the commanded
  velocity sits under the motor deadband, then releases all at once in a
  direction that is not reliably the commanded one. A naive "ramp vx up
  until the wheels turn" sweep is exactly the procedure that produces this:
  it holds sub-deadband commands for as long as it takes to notice motion,
  which is precisely long enough to build a dangerous release.

  Every design choice below exists to defuse that:
    * Each step's hold is short (default 1.5s) -- shorter than the ~3s it
      took the known lurch to build, so windup has less time to accumulate
      before the step ends regardless of what the operator sees.
    * Every hold is followed by >=1.0s of published zero Twist before the
      next (larger) step starts, so a windup that DID accumulate discharges
      instead of carrying forward and appearing as a bigger, more confusing
      lurch on a later, larger step.
    * The observed odom is checked EVERY control tick, not just at the end
      of a step: a reading opposite the commanded sign beyond tolerance is
      treated as the windup/lurch hazard happening live and aborts the
      whole sweep immediately, publishing zero and stopping.
    * Two thresholds are reported, not one: "first motion at all" can be a
      lurch caught mid-release; "first SMOOTH PROPORTIONAL motion" requires
      motion to appear promptly (not after a stall) and to hold steady
      rather than spike, which is what actually distinguishes a real
      deadband breakaway from a windup release landing in the sample window.
    * --on-blocks is a hard, unbypassable interlock on ever publishing a
      real command: this sweep exists BECAUSE sub-deadband commands can move
      the rover unpredictably, so the wheels must be free before any of it
      is allowed to run for real.
    * --dry-run exercises 100% of the same decision logic (step sequencing,
      resume/state-file handling, the motion/smooth classifier, the summary
      table) against a synthetic model, with no rclpy import, no ROS_DOMAIN_ID
      requirement, and no possibility of ever touching /cmd_vel. It is the
      way to review this script.
    * On every exit path -- normal completion, an abort, an uncaught
      exception, or SIGINT/SIGTERM -- zero Twist is published repeatedly
      before the process ends.

RESUMABILITY
  Progress is written to a JSON state file after every completed step
  (never mid-step, so a resume never has to reconstruct a half-finished
  step). --resume continues an existing file instead of repeating steps the
  rover already sat through; without --resume, an existing file is refused
  rather than silently overwritten.

WHAT IS MEASURED
  /cmd_vel linear.x and angular.z, swept independently, each in both signs
  (deadbands are commonly asymmetric on brushed motors, so +/- are measured
  and reported separately, never assumed equal). /odom_raw twist is the
  observed signal. /battery gates the whole thing on pack voltage.

ROS2 Humble. ROS_DOMAIN_ID=20 is mandatory for this rover (see
fpms-teleop.service / micro-ros-agent.service) and is enforced before any
live ROS activity -- see _enforce_domain_id().
"""

import argparse
import json
import math
import os
import random
import signal
import statistics
import sys
import threading
import time


# ===================================================================== CONFIG
# Absolute ceilings this script will NEVER exceed on /cmd_vel, regardless of
# what --cap-lin/--cap-ang are given. This sweep only ever needs to probe a
# small neighbourhood around a deadband well under 1 m/s (the firmware's own
# clamp), so these are generous relative to the default caps (0.10 / 0.20)
# but still nowhere near fast. Applied as the LAST step before every publish,
# same pattern as fpms_teleop's hard clamp immediately before building a
# Twist -- no code path, present or future, can get a bigger number out.
ABSOLUTE_MAX_LIN = 0.20
ABSOLUTE_MAX_ANG = 0.40
ABS_MAX = {"lin": ABSOLUTE_MAX_LIN, "ang": ABSOLUTE_MAX_ANG}

# Below this, a sampled odom twist is indistinguishable from encoder /
# estimator noise at rest. Deliberately not exposed on the CLI: a mistyped
# flag here would silently change what counts as "moving", which is a
# safety-relevant classification, not a tuning knob.
NOISE_FLOOR_LIN = 0.004   # m/s
NOISE_FLOOR_ANG = 0.010   # rad/s

# A late-window reading this far opposite the commanded sign is treated as
# the windup/lurch hazard actively happening, not as "no motion" -- it
# aborts the whole sweep rather than just marking one step as non-smooth.
# Also not CLI-exposed, for the same reason as the noise floors above.
WRONG_WAY_TOL_LIN = 0.015   # m/s
WRONG_WAY_TOL_ANG = 0.030   # rad/s

# How long to wait at sweep start (and at the start of every step) for a
# first /battery and /odom_raw reading before refusing to command anything.
BATTERY_WAIT_S = 5.0

# Two consecutive steps must both classify as "smooth" before the smaller of
# the two is accepted as the threshold. One is not enough: a single step
# landing just above the noise floor by chance would otherwise be mistaken
# for the real breakaway point.
CONFIRM_STEPS = 2

# A step's late-window sample-to-sample spread must stay under this fraction
# of its own mean (or under the noise floor, if the mean is tiny) to count
# as steady. A stall-then-lurch shows the opposite signature: near-zero for
# most of the window, then a brief large excursion -- i.e. high variance.
SMOOTH_JITTER_FRAC = 0.6

# Recommended MIN_CMD = measured smooth threshold * this margin. It buys a
# little headroom for measurement noise and the short (by design) 2-step
# confirmation window. It is NOT a substitute for the loaded-floor caveat
# printed in the summary -- a no-load, wheels-off-the-ground measurement is
# a lower bound, not the number to ship without on-floor verification.
MARGIN_FACTOR = 1.15

DEFAULT_STATE_FILE = "deadband_sweep_state.json"
DRY_RUN_SEED = 1234

AXES = (("lin", 1.0, "lin+"), ("lin", -1.0, "lin-"),
        ("ang", 1.0, "ang+"), ("ang", -1.0, "ang-"))

# The set of parameters that define what a sweep actually DID, compared
# against a resumed state file's recorded config. A mismatch here would mean
# resuming into a differently-shaped sweep than the one that started it, so
# it is refused rather than silently blended.
SHAPE_KEYS = ("step_lin", "cap_lin", "step_ang", "cap_ang", "hold_s", "gap_s",
             "sample_window_s", "full_sweep")


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def jround(v, nd=6):
    """JSON-safe rounded float, or None. Mirrors fpms_teleop's jnum(): NaN /
    Infinity are not valid JSON and would break a naive reader of the state
    file, so they become null instead of corrupting it."""
    if v is None:
        return None
    try:
        f = float(v)
        if not math.isfinite(f):
            return None
        return round(f, nd)
    except Exception:
        return None


class AbortSweep(Exception):
    """Raised anywhere inside a step to unwind the whole sweep to the single
    safe-exit path. Every catch site is a top-level driver (run_dry_sweep /
    run_live_sweep) that zeroes the Twist and saves state before anything
    else -- a step is never allowed to catch this and quietly continue."""


# ================================================================ STATE FILE
class StateStore:
    """Atomic JSON progress file so a sweep survives Ctrl-C, a flat battery,
    or a dropped connection and can be continued with --resume instead of
    repeating steps the rover already sat through.

    Refuses to overwrite an existing file unless --resume is given, and
    refuses to resume into a differently-shaped sweep (different step size,
    cap, hold time, ...) than the one that created the file -- both are
    "ask before doing something you can't get back", not conveniences.
    """

    def __init__(self, path, cfg, resume):
        self.path = path
        self.cfg = cfg
        self.resume = resume
        self.state = None

    def load_or_init(self):
        exists = os.path.exists(self.path)
        if exists and not self.resume:
            raise SystemExit(
                f"refuse: state file already exists at {self.path!r}. Pass "
                "--resume to continue it, or --state-file <other path> to "
                "start somewhere else. (Never silently overwriting recorded "
                "progress.)")
        if exists and self.resume:
            with open(self.path, "r", encoding="utf-8") as fh:
                loaded = json.load(fh)
            saved_cfg = loaded.get("config", {})
            mismatch = [k for k in SHAPE_KEYS if saved_cfg.get(k) != self.cfg.get(k)]
            if mismatch:
                raise SystemExit(
                    f"refuse to resume {self.path!r}: sweep shape differs "
                    f"from the run that created it ({', '.join(mismatch)}). "
                    "Match the original flags, or use a different "
                    "--state-file to start a fresh sweep.")
            self.state = loaded
            log(f"resuming from {self.path}")
        else:
            self.state = {
                "version": 1,
                "config": dict(self.cfg),
                "created_at": time.time(),
                "updated_at": time.time(),
                "axes": {k: {"done": False, "steps": [], "first_motion": None,
                            "first_smooth": None}
                         for k in ("lin+", "lin-", "ang+", "ang-")},
                "aborted": False,
                "abort_reason": None,
            }
            if self.resume:
                log(f"--resume given but no existing state file at "
                    f"{self.path}; starting fresh")
        return self.state

    def save(self):
        self.state["updated_at"] = time.time()
        tmp = f"{self.path}.tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(self.state, fh, indent=2)
        os.replace(tmp, self.path)   # atomic replace on both POSIX and Windows


# ============================================================== CLASSIFIER
def analyze_step(commanded, mean_early, std_early, n_early,
                 mean_late, std_late, n_late, noise_floor, wrong_way_tol):
    """Classify one step's response. Shared by the live and dry-run paths so
    a dry-run review is exercising the exact same decision logic a real
    sweep uses, not a parallel reimplementation that could quietly drift.

    motion   -- threshold (a): the late-window mean exceeds the noise floor
                in the COMMANDED direction. Fires on the first twitch,
                including a stall-then-release lurch if the release happens
                to land inside the late sample window.

    smooth   -- threshold (b), the one that matters: motion is ALSO present
                in the early window (i.e. it started promptly, not after a
                multi-second stall) and the late window is comparatively
                steady rather than spiky (see SMOOTH_JITTER_FRAC). A lurch
                is near-zero-then-a-spike -- high variance, and effectively
                zero in the early window; genuine proportional response at a
                held command is flat and present from close to the start.

    wrong_way -- the late-window mean has the opposite sign of the command
                and exceeds `wrong_way_tol`. This is never folded into "no
                motion": in the live path it is checked every control tick
                and aborts the sweep well before a step even finishes, and
                this end-of-step check is a backstop for the same thing.
    """
    if commanded == 0 or n_late == 0 or mean_late is None:
        return False, False, False

    sign = 1.0 if commanded > 0 else -1.0
    signed_late = mean_late * sign

    wrong_way = signed_late < -wrong_way_tol
    motion = (not wrong_way) and signed_late > noise_floor
    if not motion:
        return motion, False, wrong_way

    early_ok = (mean_early is not None and n_early > 0
               and (mean_early * sign) > noise_floor)
    jitter_cap = max(noise_floor, SMOOTH_JITTER_FRAC * abs(mean_late))
    jitter_ok = (std_late is None) or (std_late < jitter_cap)
    smooth = early_ok and jitter_ok
    return motion, smooth, wrong_way


def _finish_step(axis, sign, mag, commanded, early, late, cfg, simulated):
    mean_e = statistics.mean(early) if early else None
    std_e = statistics.pstdev(early) if len(early) > 1 else (0.0 if early else None)
    mean_l = statistics.mean(late) if late else None
    std_l = statistics.pstdev(late) if len(late) > 1 else (0.0 if late else None)

    noise_floor = NOISE_FLOOR_LIN if axis == "lin" else NOISE_FLOOR_ANG
    wrong_tol = WRONG_WAY_TOL_LIN if axis == "lin" else WRONG_WAY_TOL_ANG

    motion, smooth, wrong_way = analyze_step(
        commanded, mean_e, std_e, len(early), mean_l, std_l, len(late),
        noise_floor, wrong_tol)

    return {
        "axis": axis, "sign": sign, "mag": round(mag, 6),
        "commanded": round(commanded, 6),
        "mean_early": jround(mean_e), "std_early": jround(std_e), "n_early": len(early),
        "mean_late": jround(mean_l), "std_late": jround(std_l), "n_late": len(late),
        "motion": motion, "smooth": smooth, "wrong_way": wrong_way,
        "simulated": simulated, "t": time.time(),
    }


# ============================================================= SWEEP DRIVER
def _recompute_streak(steps):
    """Reconstruct the "consecutive smooth steps" streak purely from the
    recorded steps of a (possibly resumed) axis. Steps are always appended
    in strictly increasing magnitude order and never skipped, so a trailing
    contiguous run of smooth=True steps is exactly the streak the sweep was
    tracking when it last saved."""
    streak, candidate = 0, None
    for s in steps:
        if s["smooth"]:
            if streak == 0:
                candidate = s["mag"]
            streak += 1
        else:
            streak, candidate = 0, None
    return streak, candidate


def sweep_axis(axis_key, axis, sign, cfg, state, store, step_fn, stop_flag):
    """Sweep one axis/direction from just above zero to its cap, calling
    step_fn(axis, sign, mag) for each magnitude. Used identically by the
    live and dry-run drivers -- step_fn is the only thing that differs.
    """
    ax_state = state["axes"][axis_key]
    if ax_state["done"]:
        log(f"{axis_key}: already complete (resume) -- "
            f"first_motion={ax_state['first_motion']} "
            f"first_smooth={ax_state['first_smooth']}")
        return

    step_size = cfg["step_lin"] if axis == "lin" else cfg["step_ang"]
    cap = cfg["cap_lin"] if axis == "lin" else cfg["cap_ang"]

    steps = ax_state["steps"]
    streak, candidate = _recompute_streak(steps)
    next_mag = round((steps[-1]["mag"] + step_size) if steps else step_size, 10)

    log(f"{axis_key}: sweeping from {next_mag:.4f} to cap {cap:.4f} "
        f"(step {step_size:.4f})")

    while next_mag <= cap + 1e-9:
        if stop_flag.is_set():
            raise AbortSweep("operator interrupt (signal) before step "
                             f"{axis_key} mag={next_mag:.4f}")

        result = step_fn(axis, sign, next_mag)
        steps.append(result)

        if result["motion"] and ax_state["first_motion"] is None:
            ax_state["first_motion"] = result["mag"]

        if result["smooth"]:
            if streak == 0:
                candidate = result["mag"]
            streak += 1
        else:
            streak, candidate = 0, None

        if streak >= CONFIRM_STEPS and ax_state["first_smooth"] is None:
            ax_state["first_smooth"] = candidate

        store.save()

        if ax_state["first_smooth"] is not None and not cfg["full_sweep"]:
            # Early stop: two consecutive confirmations is enough to trust
            # the number, and every extra step past that is wheel wear and
            # battery spent reconfirming a fact already established.
            # --full-sweep disables this and always runs to cap.
            ax_state["done"] = True
            log(f"{axis_key}: smooth threshold confirmed at "
                f"{ax_state['first_smooth']:.4f} -- stopping this axis "
                "early (use --full-sweep to always reach the cap)")
            store.save()
            return

        next_mag = round(next_mag + step_size, 10)

    ax_state["done"] = True
    store.save()
    if ax_state["first_smooth"] is None:
        log(f"{axis_key}: reached cap {cap:.4f} without confirming smooth "
            "motion; the deadband may exceed the sweep cap")


# ================================================================= DRY RUN
# Synthetic per-axis/direction deadband model, deliberately asymmetric
# between + and - to exercise the asymmetry-reporting path in the summary.
# These numbers are NOT a measurement of anything -- they exist only so
# --dry-run has something plausible to run the real classifier against.
DRY_RUN_MODEL = {
    "lin+": {"deadband": 0.014, "gain": 0.8, "noise": 0.0015},
    "lin-": {"deadband": 0.018, "gain": 0.8, "noise": 0.0015},
    "ang+": {"deadband": 0.030, "gain": 0.6, "noise": 0.0040},
    "ang-": {"deadband": 0.024, "gain": 0.6, "noise": 0.0040},
}


def run_step_dry(axis, sign, mag, cfg, model, rng):
    commanded = clamp(sign * mag, -ABS_MAX[axis], ABS_MAX[axis])
    log(f"[DRY-RUN] would hold {axis} commanded={commanded:+.4f} for "
        f"{cfg['hold_s']:.1f}s, then publish zero for {cfg['gap_s']:.1f}s "
        "(SIMULATED -- no ROS, no /cmd_vel)")

    deadband, gain, noise = model["deadband"], model["gain"], model["noise"]
    n = max(1, int(round(cfg["sample_window_s"] * cfg["control_hz"])))

    if mag < deadband:
        # Below the synthetic deadband: mostly silent. Occasionally (as the
        # real hazard this script exists to avoid) a small stall-then-wobble
        # that shows up ONLY in the late window -- exactly the pattern
        # analyze_step's early/late split is built to reject as "smooth".
        early = [rng.uniform(-noise, noise) for _ in range(n)]
        if rng.random() < 0.15:
            wobble = -0.35 * gain * deadband
            late = [sign * wobble + rng.uniform(-noise, noise) for _ in range(n)]
        else:
            late = [rng.uniform(-noise, noise) for _ in range(n)]
    else:
        # At/above the synthetic deadband: genuine proportional response,
        # already mostly spun up early and steady by the late window.
        target = gain * (mag - deadband)
        early = [sign * target * 0.7 + rng.uniform(-noise, noise) for _ in range(n)]
        late = [sign * target + rng.uniform(-noise, noise) for _ in range(n)]

    return _finish_step(axis, sign, mag, commanded, early, late, cfg, simulated=True)


def run_dry_sweep(cfg, state, store, stop_flag):
    rng = random.Random(DRY_RUN_SEED)
    for axis_key, axis, sign in AXES:
        model = DRY_RUN_MODEL[axis_key]

        def step_fn(a, s, m, _model=model):
            return run_step_dry(a, s, m, cfg, _model, rng)

        try:
            sweep_axis(axis_key, axis, sign, cfg, state, store, step_fn, stop_flag)
        except AbortSweep as e:
            state["aborted"] = True
            state["abort_reason"] = f"[DRY-RUN simulated] {e}"
            store.save()
            log(f"DRY-RUN ABORT (simulated): {e}")
            return


# ================================================================= LIVE RUN
def _enforce_domain_id():
    """ROS_DOMAIN_ID=20 is mandatory for this rover (fpms-teleop.service /
    micro-ros-agent.service both pin it). If unset we default it here so a
    bare `python deadband_sweep.py --on-blocks` still talks to the right
    graph; if it is set to something else we refuse rather than silently
    override an operator's explicit environment -- publishing on the wrong
    domain either talks to nothing, or worse, to some other ROS graph."""
    cur = os.environ.get("ROS_DOMAIN_ID")
    if cur is None:
        os.environ["ROS_DOMAIN_ID"] = "20"
        log("ROS_DOMAIN_ID was unset; defaulting it to 20 (mandatory for this rover)")
    elif cur != "20":
        raise SystemExit(
            f"refuse: ROS_DOMAIN_ID={cur!r} in this shell but this rover "
            "requires 20. Fix the environment and re-run.")


def run_live_sweep(cfg, state, store, stop_flag):
    _enforce_domain_id()

    # Deferred import: rclpy is only ever touched on the live path, so
    # --dry-run works even on a machine with no ROS2 installed at all.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import UInt16

    class DeadbandNode(Node):
        """Owns exactly the resources this sweep needs: one /cmd_vel
        publisher and read-only subscriptions to /odom_raw and /battery.
        Never touches a serial port, never restarts anything."""

        def __init__(self):
            super().__init__("fpms_deadband_sweep")
            self.lock = threading.Lock()
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
            self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", qos)
            self.create_subscription(Odometry, "/odom_raw", self._on_odom, qos)
            self.create_subscription(UInt16, "/battery", self._on_battery, qos)
            self.odom_vx = None
            self.odom_wz = None
            self.odom_last = None
            self.battery_v = None

        def _on_odom(self, msg):
            try:
                with self.lock:
                    self.odom_vx = float(msg.twist.twist.linear.x)
                    self.odom_wz = float(msg.twist.twist.angular.z)
                    self.odom_last = time.monotonic()
            except Exception as e:
                log(f"odom callback error {e}")

        def _on_battery(self, msg):
            try:
                with self.lock:
                    self.battery_v = int(msg.data) / 10.0
            except Exception as e:
                log(f"battery callback error {e}")

        def component(self, axis):
            with self.lock:
                return self.odom_vx if axis == "lin" else self.odom_wz

        def publish(self, axis, value):
            t = Twist()
            if axis == "lin":
                t.linear.x = value
            else:
                t.angular.z = value
            self.pub_cmd.publish(t)

        def zero_flood(self, n=10, dt=0.03):
            """Called on every exit path. Publishing once is not enough to
            trust over an unreliable link; ten zeros at 30ms apart is cheap
            and matches fpms_teleop's own shutdown_stop pattern."""
            for _ in range(n):
                try:
                    self.pub_cmd.publish(Twist())
                except Exception:
                    pass
                time.sleep(dt)

    def check_watchdogs(node):
        v = node.battery_v
        if v is not None and v < cfg["batt_floor_v"]:
            raise AbortSweep(f"battery {v:.2f}V < floor {cfg['batt_floor_v']:.2f}V")
        with node.lock:
            last = node.odom_last
        if last is None or (time.monotonic() - last) > cfg["odom_timeout_s"]:
            raise AbortSweep(f"no /odom_raw for > {cfg['odom_timeout_s']:.1f}s")

    def preflight(node):
        t0 = time.monotonic()
        while node.battery_v is None or node.odom_last is None:
            if time.monotonic() - t0 > BATTERY_WAIT_S:
                missing = [name for name, ok in
                          (("/battery", node.battery_v is not None),
                           ("/odom_raw", node.odom_last is not None)) if not ok]
                raise AbortSweep(
                    f"no data on {', '.join(missing)} within "
                    f"{BATTERY_WAIT_S:.0f}s -- is micro-ros-agent up with "
                    "ROS_DOMAIN_ID=20?")
            if stop_flag.is_set():
                raise AbortSweep("operator interrupt (signal) during preflight")
            rclpy.spin_once(node, timeout_sec=0.1)
        check_watchdogs(node)

    dt = 1.0 / cfg["control_hz"]

    def run_step_live(axis, sign, mag):
        commanded = clamp(sign * mag, -ABS_MAX[axis], ABS_MAX[axis])
        log(f"step: axis={axis} commanded={commanded:+.4f} "
            f"hold={cfg['hold_s']:.1f}s")

        preflight(node)

        wrong_tol = WRONG_WAY_TOL_LIN if axis == "lin" else WRONG_WAY_TOL_ANG
        cmd_sign = 1.0 if commanded >= 0 else -1.0

        t0 = time.monotonic()
        early, late = [], []
        while True:
            t_rel = time.monotonic() - t0
            if t_rel >= cfg["hold_s"]:
                break
            if stop_flag.is_set():
                raise AbortSweep("operator interrupt (signal) mid-hold "
                                 f"{axis} mag={mag:.4f}")

            node.publish(axis, commanded)
            rclpy.spin_once(node, timeout_sec=dt)
            check_watchdogs(node)

            val = node.component(axis)
            if val is not None:
                # Checked every tick, not just at the end of the hold: this
                # is the windup-release hazard potentially happening live,
                # and the right response is to cut to zero now rather than
                # keep commanding while it finds out how far it can lurch.
                if commanded != 0 and (val * cmd_sign) < -wrong_tol:
                    raise AbortSweep(
                        f"observed {axis}={val:+.4f} opposite to commanded "
                        f"{commanded:+.4f} beyond tolerance {wrong_tol:.3f} "
                        "-- possible windup/lurch in progress")
                if t_rel <= cfg["sample_window_s"]:
                    early.append(val)
                if t_rel >= cfg["hold_s"] - cfg["sample_window_s"]:
                    late.append(val)

        # >=1.0s of published zero here is essential, not a pause between
        # steps: it is what lets a below-deadband integrator discharge
        # before the NEXT, larger step is commanded, so windup can never
        # carry forward and taint a later step's reading. See module
        # docstring and the --gap-s validation in main().
        t1 = time.monotonic()
        while time.monotonic() - t1 < cfg["gap_s"]:
            if stop_flag.is_set():
                raise AbortSweep("operator interrupt (signal) during zero-gap")
            node.publish(axis, 0.0)
            rclpy.spin_once(node, timeout_sec=dt)
            check_watchdogs(node)

        result = _finish_step(axis, sign, mag, commanded, early, late, cfg,
                              simulated=False)
        if result["wrong_way"]:
            raise AbortSweep(
                f"post-step check: {axis} mean_late={result['mean_late']} "
                "opposite to commanded -- aborting sweep")
        return result

    rclpy.init(args=None)
    node = DeadbandNode()
    try:
        for axis_key, axis, sign in AXES:
            sweep_axis(axis_key, axis, sign, cfg, state, store, run_step_live,
                      stop_flag)
    except AbortSweep as e:
        state["aborted"] = True
        state["abort_reason"] = str(e)
        log(f"ABORT: {e}")
    except Exception as e:
        state["aborted"] = True
        state["abort_reason"] = f"unexpected exception: {e!r}"
        log(f"EXCEPTION during live sweep: {e!r}")
    finally:
        # On ANY exit path -- clean finish, abort, or exception above --
        # zero Twist is published repeatedly before anything is torn down.
        log("exit path: publishing zero Twist repeatedly")
        try:
            node.zero_flood()
        except Exception:
            pass
        store.save()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


# =================================================================== SUMMARY
def print_summary(cfg, state):
    print()
    print("=" * 78)
    print("DEADBAND SWEEP SUMMARY")
    print("=" * 78)
    print(f"{'axis':<6}{'dir':<6}{'first motion':<16}{'first SMOOTH (*)':<20}{'status'}")
    print("-" * 78)

    recs = {}
    for axis in ("lin", "ang"):
        for sign_label in ("+", "-"):
            axis_key = f"{axis}{sign_label}"
            ax = state["axes"][axis_key]
            fm, fs = ax["first_motion"], ax["first_smooth"]
            if fs is not None:
                status = "ok"
            elif ax["done"]:
                status = f"NOT FOUND (<= cap)"
            else:
                status = "incomplete"
            fm_s = f"{fm:.4f}" if fm is not None else "none"
            fs_s = f"{fs:.4f}" if fs is not None else "--"
            print(f"{axis:<6}{sign_label:<6}{fm_s:<16}{fs_s:<20}{status}")
            recs[axis_key] = fs
    print("-" * 78)
    print("(*) first SMOOTH PROPORTIONAL motion -- the number a MIN_CMD "
          "floor should be based on.")

    def recommend(axis):
        p, m = recs.get(f"{axis}+"), recs.get(f"{axis}-")
        vals = [abs(v) for v in (p, m) if v is not None]
        rec = (max(vals) * MARGIN_FACTOR) if vals else None
        return rec, p, m

    print()
    print("RECOMMENDATION")
    for axis, label, cap_key in (("lin", "MIN_CMD_LIN", "cap_lin"),
                                 ("ang", "MIN_CMD_ANG", "cap_ang")):
        rec, p, m = recommend(axis)
        if rec is None:
            print(f"  {label} = UNKNOWN -- neither direction confirmed "
                  f"smooth motion within cap {cfg[cap_key]:.4f}; rerun with "
                  f"a higher --cap-{axis}.")
            continue
        if p is not None and m is not None:
            detail = f"(= max({p:.4f}, {m:.4f}) x {MARGIN_FACTOR:.2f} margin)"
            rel = abs(abs(p) - abs(m)) / max(abs(p), abs(m)) if max(abs(p), abs(m)) else 0.0
            asym = (f"\n  NOTE: {axis}+ and {axis}- differ by {rel*100:.0f}% -- "
                    "asymmetric deadband, as is common on brushed motors. "
                    "Consider a per-direction floor instead of one "
                    f"{label}.") if rel > 0.20 else ""
        else:
            have = p if p is not None else m
            detail = f"(only one direction confirmed: {have:.4f}; x {MARGIN_FACTOR:.2f} margin)"
            asym = ""
        print(f"  {label} = {rec:.4f}   {detail}{asym}")

    print()
    print("CAVEAT: this is a NO-LOAD measurement (wheels off the ground,")
    print("--on-blocks). Rolling friction and ground contact under real")
    print("load only ever RAISE the effective deadband. Treat every number")
    print("above as a LOWER BOUND -- verify MIN_CMD on the floor before")
    print("trusting it, and when in doubt round up.")

    if state.get("aborted"):
        print()
        print(f"RUN WAS ABORTED: {state.get('abort_reason')}")
        print("The table above reflects only what was measured before the abort.")
    print("=" * 78)


# ====================================================================== MAIN
STOP_FLAG = threading.Event()


def build_argparser():
    p = argparse.ArgumentParser(
        prog="deadband_sweep.py",
        description="Measure the FPMS rover's motor deadband so a MIN_CMD "
                    "floor can be set. See the module docstring for the "
                    "safety model before running this for real.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Review first (no ROS, nothing moves, no --on-blocks needed):\n"
            "  python deadband_sweep.py --dry-run\n\n"
            "Real measurement, wheels elevated and clear:\n"
            "  python deadband_sweep.py --on-blocks --resume\n\n"
            "Continue an interrupted real run (same command; the state\n"
            "file makes it pick up where it left off):\n"
            "  python deadband_sweep.py --on-blocks --resume\n"
        ),
    )
    p.add_argument("--on-blocks", action="store_true",
                   help="Confirm the wheels are elevated and clear of any "
                        "surface, person, or obstruction. Required for any "
                        "real (non---dry-run) sweep; without it the script "
                        "refuses to run.")
    p.add_argument("--dry-run", action="store_true",
                   help="Exercise all sweep logic against a synthetic "
                        "model. Never touches ROS or /cmd_vel. The safe "
                        "default way to review this script.")
    p.add_argument("--step-lin", type=float, default=0.002,
                   help="linear.x sweep step (default 0.002)")
    p.add_argument("--cap-lin", type=float, default=0.10,
                   help=f"linear.x sweep cap (default 0.10, hard ceiling "
                        f"{ABSOLUTE_MAX_LIN})")
    p.add_argument("--step-ang", type=float, default=0.004,
                   help="angular.z sweep step (default 0.004)")
    p.add_argument("--cap-ang", type=float, default=0.20,
                   help=f"angular.z sweep cap (default 0.20, hard ceiling "
                        f"{ABSOLUTE_MAX_ANG})")
    p.add_argument("--hold-s", type=float, default=1.5,
                   help="seconds to hold each commanded step (default 1.5)")
    p.add_argument("--gap-s", type=float, default=1.2,
                   help="seconds of zero Twist between steps, must be >= "
                        "1.0 (default 1.2)")
    p.add_argument("--sample-window-s", type=float, default=0.5,
                   help="seconds sampled at the start and end of each hold "
                        "(default 0.5)")
    p.add_argument("--batt-floor-v", type=float, default=11.0,
                   help="abort if pack voltage drops below this (default 11.0)")
    p.add_argument("--odom-timeout-s", type=float, default=3.0,
                   help="abort if /odom_raw goes silent this long (default 3.0)")
    p.add_argument("--control-hz", type=float, default=20.0,
                   help="publish/sample rate during a hold (default 20.0)")
    p.add_argument("--resume", action="store_true",
                   help="continue an existing state file instead of "
                        "refusing to overwrite it")
    p.add_argument("--state-file", type=str, default=DEFAULT_STATE_FILE,
                   help=f"progress JSON path (default {DEFAULT_STATE_FILE}; "
                        "--dry-run uses a .dryrun-suffixed path by default "
                        "so it can never collide with real progress)")
    p.add_argument("--full-sweep", action="store_true",
                   help="disable the early-stop-once-confirmed behaviour "
                        "and always sweep every axis/direction to its cap")
    return p


def main():
    args = build_argparser().parse_args()

    # ---- validation up front, before anything touches ROS or hardware ----
    if args.gap_s < 1.0:
        sys.exit(
            "refuse: --gap-s must be >= 1.0s. This is the window the "
            "firmware needs to discharge a wound-up integrator before the "
            "next, larger step is commanded (see module docstring); less "
            "than 1.0s risks carrying windup forward into the next step's "
            "measurement.")
    if min(args.step_lin, args.cap_lin, args.step_ang, args.cap_ang) <= 0:
        sys.exit("refuse: step/cap values must be positive")
    if args.cap_lin > ABSOLUTE_MAX_LIN:
        sys.exit(f"refuse: --cap-lin {args.cap_lin} exceeds the absolute "
                 f"ceiling {ABSOLUTE_MAX_LIN} this script will ever "
                 "command on /cmd_vel linear.x")
    if args.cap_ang > ABSOLUTE_MAX_ANG:
        sys.exit(f"refuse: --cap-ang {args.cap_ang} exceeds the absolute "
                 f"ceiling {ABSOLUTE_MAX_ANG} this script will ever "
                 "command on /cmd_vel angular.z")
    if args.sample_window_s <= 0 or args.sample_window_s * 2 > args.hold_s:
        sys.exit(
            "refuse: --sample-window-s must be positive, and the early + "
            "late sample windows must fit inside --hold-s without "
            "overlapping (need hold-s >= 2 x sample-window-s)")

    if not args.dry_run and not args.on_blocks:
        sys.exit(
            "refuse to run: --on-blocks was not given.\n"
            "This sweep commands real motor motion, including magnitudes "
            "below the motor deadband where the firmware is known to wind "
            "up an integrator and release it as a lurch in an "
            "unpredictable direction (measured 2026-07-31: 0.0041 "
            "commanded -> 290mm BACKWARD after ~3s of stillness).\n"
            "--on-blocks is the operator's attestation that the wheels are "
            "elevated and clear of any surface, person, or obstruction "
            "before that can happen.\n"
            "Use --dry-run instead to review this script's logic with no "
            "hardware and no --on-blocks required.")

    signal.signal(signal.SIGINT, lambda *_: STOP_FLAG.set())
    signal.signal(signal.SIGTERM, lambda *_: STOP_FLAG.set())

    cfg = {
        "step_lin": args.step_lin, "cap_lin": args.cap_lin,
        "step_ang": args.step_ang, "cap_ang": args.cap_ang,
        "hold_s": args.hold_s, "gap_s": args.gap_s,
        "sample_window_s": args.sample_window_s,
        "batt_floor_v": args.batt_floor_v, "odom_timeout_s": args.odom_timeout_s,
        "control_hz": args.control_hz, "full_sweep": args.full_sweep,
    }

    state_path = args.state_file
    if args.dry_run and args.state_file == DEFAULT_STATE_FILE:
        # A dry-run's "progress" is synthetic. Defaulting to a separate
        # path means a casual --dry-run review can never collide with, or
        # be mistaken for, a real in-progress sweep's state file.
        state_path = DEFAULT_STATE_FILE + ".dryrun"

    store = StateStore(state_path, cfg, args.resume)
    state = store.load_or_init()

    if args.dry_run:
        log("DRY RUN: no ROS, no /cmd_vel publish, no hardware required. "
            "All numbers below are SYNTHETIC, for exercising the logic only.")
        run_dry_sweep(cfg, state, store, STOP_FLAG)
    else:
        log("LIVE RUN: --on-blocks acknowledged. This will publish to /cmd_vel.")
        run_live_sweep(cfg, state, store, STOP_FLAG)

    print_summary(cfg, state)

    if state.get("aborted"):
        sys.exit(1)


if __name__ == "__main__":
    main()
