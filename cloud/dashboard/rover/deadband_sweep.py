#!/usr/bin/env python3
"""Firmware velocity-floor sweep for the Yahboom MicroROS Board V2.0 rover.

WHAT THIS ACTUALLY MEASURES -- READ THIS BEFORE QUOTING A NUMBER FROM IT
  The lowest /cmd_vel setpoint the board's velocity loop can still HOLD.
  That is NOT a mechanical deadband, and the difference decides what you do
  about it:

    * A MECHANICAL DEADBAND is stiction in the drivetrain. It scales with
      load, you clear it with torque, and a MIN_CMD floor is a reasonable
      way to live with it.
    * What this rover has is QUANTISATION, and it is in the firmware, not
      the motors. The velocity loop regulates INTEGER encoder counts per
      10 ms PID period (MOTOR_ENCODER_CIRCLE 1040, MOTOR_WHEEL_CIRCLE
      150.8 mm, MOTOR_PID_PERIOD 10). One count per period is 0.0145 m/s ON
      THE WIRE. Below that the setpoint is under a single count and the
      feedback is pure quantisation noise, so the loop has nothing to
      regulate against. The wheels are not stuck -- the controller is
      blind. The 200/400 PWM_MOTOR_DEAD_ZONE is a 50 % duty feed-forward,
      so commanded duty is either exactly 0 or 50-100 %: there is no gentle
      duty to fall back on either.

  Expect the answer to land near 0.0145 wire. Observations already do:
  0.0041 wire (0.28 counts/10 ms) stalls then lurches, 0.012 (0.83) is
  noise, 0.100 (6.9) is clean and correct. A result far BELOW 0.0145 is a
  reason to distrust the measurement, not a discovery.

  THIS FLOOR IS NOT FIXABLE FROM THIS FILE, from PID gains, or from a
  MIN_CMD constant. A floor only stops you ASKING for a speed the loop
  cannot hold; it does not give the loop resolution it does not have. The
  real fix is the encoder/wheel constants, i.e. a firmware rebuild. Until
  then, "slow and controlled" comes from SHORT BOUNDED SEGMENTS WITH STOPS
  BETWEEN, run at a speed the loop can actually hold (>= ~0.0145 wire,
  ~0.18 m/s real with margin), never from a low continuous setpoint.
  See NAV2_BRIEF.md section 3b -- that file wins over this one.

WHY THE FIRST LIVE RUN OF THIS SCRIPT ABORTED (2026-07-31) -- DO NOT REDO IT
  It aborted itself, on correct motion, and the bug was in this file.
  Every reading came from /odom_raw's twist.linear.x, which is
  SIGN-INVERTED relative to its own pose.position (NAV2_BRIEF.md 3a;
  fpms_teleop.py ODOM_TWIST_SIGN). So the wrong-way detector saw every
  correct forward step as a reversal and killed the sweep -- while a
  GENUINE reversal would have been reported as agreeing with the command
  and passed in silence. A safety guard wired exactly backwards is worse
  than no guard, because it is trusted.

  The "290 mm BACKWARD lurch at 0.0041" this script was originally built
  around was the same artefact: a 290 mm FORWARD move read through the
  inverted sign. There is no evidence of PID windup and none of a
  stall-then-release. That premise is retracted.

  EVERYTHING OBSERVED HERE IS NOW DIFFERENTIATED POSE -- position and yaw
  deltas out of /odom_raw's pose, exactly as fpms_teleop.py's lurch guard
  does it. Pose was right in every trial; twist was wrong in every trial.
  If you are about to reintroduce a twist read into a guard: don't.

  What survives the retraction, because it costs nothing and is right for
  a sweep that deliberately commands setpoints the loop cannot hold:
    * Each step's hold is short (default 1.5s), and every hold is followed
      by >=1.0s of published zero Twist before the next, larger step. A
      sub-floor setpoint produces uncontrolled motion by definition, and
      the answer to uncontrolled motion is to stop commanding it promptly
      and let the chassis settle before asking for more.
    * Observed POSE VELOCITY is checked every control tick, not just at the
      end of a step: motion opposite the commanded sign beyond tolerance
      aborts the whole sweep immediately, publishing zero. Fed from pose,
      this now means what it says.
    * Two thresholds are reported, not one. "First motion at all" can be
      quantisation noise or an uncontrolled lurch caught mid-release;
      "first SMOOTH PROPORTIONAL motion" additionally requires motion to
      appear promptly and to hold steady, which is what distinguishes the
      loop actually regulating from the loop flailing.
    * --on-blocks is a hard, unbypassable interlock on publishing anything
      real: this sweep exists BECAUSE sub-floor setpoints move the rover
      unpredictably, so the wheels must be free before it runs for real.
    * --dry-run exercises the same decision logic (step sequencing,
      resume/state-file handling, the motion/smooth classifier, the summary
      table) against a synthetic model, with no rclpy import, no
      ROS_DOMAIN_ID requirement, and no possibility of touching /cmd_vel.
      It is the way to review this script.
    * On every exit path -- normal completion, an abort, an uncaught
      exception, or SIGINT/SIGTERM -- zero Twist is published repeatedly
      before the process ends.

UNITS -- THE COMMAND AND THE OBSERVATION ARE NOT IN THE SAME SCALE
  Commanded magnitudes are WIRE units (what goes into Twist.linear.x).
  Observed magnitudes are REAL m/s and rad/s differentiated from pose. The
  firmware passes linear.x 1:1 to wheel m/s with no gain, yet 0.10 wire
  measures ~0.61 m/s real (~6x) -- the fitted hardware does not match the
  firmware's constants. So the noise floors and wrong-way tolerances below
  are in OBSERVED units, and every threshold this script REPORTS is in
  COMMANDED wire units. Do not mix them.

RESUMABILITY
  Progress is written to a JSON state file after every completed step
  (never mid-step, so a resume never has to reconstruct a half-finished
  step). --resume continues an existing file instead of repeating steps the
  rover already sat through; without --resume, an existing file is refused
  rather than silently overwritten.

WHAT IS SWEPT
  /cmd_vel linear.x and angular.z, independently, each in both signs (+/-
  are measured and reported separately, never assumed equal -- the
  quantisation is symmetric in theory but nothing here assumes theory).
  /odom_raw POSE is the observed signal. /battery gates it on pack voltage.

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
# small neighbourhood around a floor well under 1 m/s (the firmware's own
# clamp), so these are generous relative to the default caps (0.10 / 0.20)
# but still nowhere near fast. Applied as the LAST step before every publish,
# same pattern as fpms_teleop's hard clamp immediately before building a
# Twist -- no code path, present or future, can get a bigger number out.
ABSOLUTE_MAX_LIN = 0.20
ABSOLUTE_MAX_ANG = 0.40
ABS_MAX = {"lin": ABSOLUTE_MAX_LIN, "ang": ABSOLUTE_MAX_ANG}

# The floor this sweep expects to find, derived from firmware constants
# rather than measured here: one encoder count per PID period is
# (150.8 mm / 1040 counts) / 10 ms = 0.0145 m/s ON THE WIRE. Printed beside
# the result so the measurement is read against the prediction instead of
# in a vacuum. NOT used to decide anything -- a constant that quietly
# steered the classifier towards its own value would make the sweep
# pointless.
QUANT_FLOOR_WIRE = 0.0145   # m/s of commanded linear.x

# THE ONE PLACE /odom_raw's twist IS ALLOWED IN THIS FILE, and it is a
# DIAGNOSTIC ONLY -- no guard, classifier or threshold consumes it.
# twist.linear.x is sign-inverted relative to the pose in the SAME message
# (NAV2_BRIEF.md 3a). Reading it uncorrected is what aborted the first live
# sweep on correct forward motion. It is still worth recording, corrected,
# next to the pose-derived number: if the two ever stop disagreeing by
# exactly this factor, the firmware changed and every conclusion in this
# file needs re-checking. One constant, applied at the single read site
# (DeadbandNode._on_odom), same discipline as fpms_teleop.py.
ODOM_TWIST_SIGN = -1

# Below this, a sampled velocity is indistinguishable from odometry noise at
# rest. In OBSERVED units (differentiated pose: real m/s and rad/s), not
# wire units -- see the UNITS section of the module docstring. Deliberately
# not exposed on the CLI: a mistyped flag here would silently change what
# counts as "moving", which is a safety-relevant classification, not a
# tuning knob.
NOISE_FLOOR_LIN = 0.004   # m/s, observed
NOISE_FLOOR_ANG = 0.010   # rad/s, observed

# Pose moving this far opposite the commanded sign is the rover genuinely
# going the wrong way -- it aborts the whole sweep rather than just marking
# one step as non-smooth. Also in observed units, and also not CLI-exposed.
#
# This test is only trustworthy because it is fed from differentiated pose.
# Fed from twist.linear.x it was exactly inverted: it fired on every correct
# step and would have stayed silent through a real reversal.
WRONG_WAY_TOL_LIN = 0.015   # m/s, observed
WRONG_WAY_TOL_ANG = 0.030   # rad/s, observed

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
# as steady. A setpoint under the quantisation floor shows the opposite
# signature: near-zero for most of the window, then a brief large excursion
# as the feed-forward duty breaks through -- i.e. high variance. That is the
# loop failing to regulate, not the wheels breaking free.
SMOOTH_JITTER_FRAC = 0.6

# Lowest setpoint worth COMMANDING = measured smooth threshold * this margin.
# It buys headroom for measurement noise and for the short (by design) 2-step
# confirmation window.
#
# It is NOT a deadband, and raising a command to it does not make a slower
# command safe -- there is no slower command. Below the floor the loop is
# blind (see the module docstring), so the only correct response is to plan
# missions that never ask for it. See also the caveat printed in the summary:
# a no-load, wheels-off measurement is a lower bound.
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


def yaw_from_quat(q):
    """Planar yaw out of a quaternion, inline. Deliberately not
    tf_transformations: it imports transforms3d, which on this rover dies with
    `np.maximum_sctype was removed in NumPy 2.0` because a user-local NumPy 2.x
    shadows the system one. Fixing that would mean touching the NumPy the
    working camera/YOLO path depends on, to save these three lines. Same
    approach as fpms_odom_tf.py and fpms_teleop.py."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_pi(a):
    """Shortest signed angle. Without this, a yaw difference across the +/-pi
    seam reads as ~2pi of rotation in one 10 Hz frame -- which the wrong-way
    check would see as a violent reversal and abort the sweep on."""
    return math.atan2(math.sin(a), math.cos(a))


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

    Every mean/std passed in is DIFFERENTIATED POSE, never twist. The whole
    classifier is sign-sensitive, and twist's sign on this board is wrong.

    motion   -- threshold (a): the late-window mean exceeds the noise floor
                in the COMMANDED direction. Fires on the first twitch,
                including an uncontrolled sub-floor lurch if it happens to
                land inside the late sample window.

    smooth   -- threshold (b), the one that matters: motion is ALSO present
                in the early window (i.e. it started promptly, not after a
                stall) and the late window is comparatively steady rather
                than spiky (see SMOOTH_JITTER_FRAC). Below the firmware's
                quantisation floor the loop cannot regulate, and what comes
                out is near-zero-then-a-spike -- high variance, effectively
                zero early. Once the setpoint is worth at least one encoder
                count per PID period, the response is flat and present from
                close to the start. That transition is what this detects.

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


def _finish_step(axis, sign, mag, commanded, early, late, cfg, simulated,
                 twist_late=None):
    """Reduce one step's samples to a record. `early`/`late` are pose-derived
    velocities. `twist_late` is the board's own (sign-corrected) twist over
    the same window, carried for the record ONLY -- it is never fed to
    analyze_step, which is the entire point."""
    mean_e = statistics.mean(early) if early else None
    std_e = statistics.pstdev(early) if len(early) > 1 else (0.0 if early else None)
    mean_l = statistics.mean(late) if late else None
    std_l = statistics.pstdev(late) if len(late) > 1 else (0.0 if late else None)

    noise_floor = NOISE_FLOOR_LIN if axis == "lin" else NOISE_FLOOR_ANG
    wrong_tol = WRONG_WAY_TOL_LIN if axis == "lin" else WRONG_WAY_TOL_ANG

    motion, smooth, wrong_way = analyze_step(
        commanded, mean_e, std_e, len(early), mean_l, std_l, len(late),
        noise_floor, wrong_tol)

    mean_tw = statistics.mean(twist_late) if twist_late else None

    return {
        "axis": axis, "sign": sign, "mag": round(mag, 6),
        "commanded": round(commanded, 6),
        # Every mean/std below is POSE-DERIVED. The key names say so, because
        # a bare "mean_late" in a state file is exactly the field someone
        # later assumes came out of msg.twist.
        "mean_early_pose": jround(mean_e), "std_early_pose": jround(std_e),
        "n_early": len(early),
        "mean_late_pose": jround(mean_l), "std_late_pose": jround(std_l),
        "n_late": len(late),
        # Diagnostic only, sign-corrected by ODOM_TWIST_SIGN. Compare it
        # against mean_late_pose; do not classify on it.
        "mean_late_twist_corrected": jround(mean_tw),
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
            "motion; the holdable floor may lie above the sweep cap")


# ================================================================= DRY RUN
# Synthetic per-axis/direction model of the FIRMWARE QUANTISATION FLOOR, not
# of a mechanical deadband: below `floor` (wire units) the loop is blind and
# the output is noise with the occasional uncontrolled excursion; at or above
# it the response is proportional to the WHOLE command with no offset, at the
# measured ~6x wire->real gain, because nothing is being overcome -- the
# controller simply starts working.
#
# Made slightly asymmetric between + and - to exercise the asymmetry-reporting
# path in the summary. These numbers are NOT a measurement: they exist so
# --dry-run has something plausible to run the real classifier against.
DRY_RUN_MODEL = {
    # `wobble` is the sub-floor excursion, in observed units. Kept deliberately
    # UNDER WRONG_WAY_TOL_* so the dry run exercises the "rejected as not
    # smooth" path rather than the abort path -- a review run that always
    # aborts teaches nothing about the sweep it is meant to review.
    "lin+": {"floor": 0.0145, "gain": 6.0, "noise": 0.0015, "wobble": 0.010},
    "lin-": {"floor": 0.0175, "gain": 6.0, "noise": 0.0015, "wobble": 0.010},
    "ang+": {"floor": 0.0300, "gain": 2.0, "noise": 0.0040, "wobble": 0.020},
    "ang-": {"floor": 0.0240, "gain": 2.0, "noise": 0.0040, "wobble": 0.020},
}


def run_step_dry(axis, sign, mag, cfg, model, rng):
    commanded = clamp(sign * mag, -ABS_MAX[axis], ABS_MAX[axis])
    log(f"[DRY-RUN] would hold {axis} commanded={commanded:+.4f} for "
        f"{cfg['hold_s']:.1f}s, then publish zero for {cfg['gap_s']:.1f}s "
        "(SIMULATED -- no ROS, no /cmd_vel)")

    floor, gain, noise = model["floor"], model["gain"], model["noise"]
    n = max(1, int(round(cfg["sample_window_s"] * cfg["control_hz"])))

    if mag < floor:
        # Under the floor: the setpoint is worth less than one encoder count
        # per PID period, so mostly silence. Occasionally the 50% feed-forward
        # duty breaks through as an uncontrolled excursion in the late window
        # only -- exactly the pattern analyze_step's early/late split exists
        # to refuse to call "smooth".
        early = [rng.uniform(-noise, noise) for _ in range(n)]
        if rng.random() < 0.15:
            wobble = -model["wobble"]
            late = [sign * wobble + rng.uniform(-noise, noise) for _ in range(n)]
        else:
            late = [rng.uniform(-noise, noise) for _ in range(n)]
    else:
        # At/above the floor: the loop regulates. Proportional to the whole
        # command (no subtracted offset -- there is no stiction to overcome),
        # already mostly spun up early and steady by the late window.
        target = gain * mag
        early = [sign * target * 0.7 + rng.uniform(-noise, noise) for _ in range(n)]
        late = [sign * target + rng.uniform(-noise, noise) for _ in range(n)]

    # No twist_late: the dry path models pose only, because pose is the only
    # thing the classifier is allowed to see.
    return _finish_step(axis, sign, mag, commanded, early, late, cfg, simulated=True)


def run_dry_sweep(cfg, state, store, stop_flag):
    rng = random.Random(DRY_RUN_SEED)
    # AXES entries are (axis, sign, axis_key) — unpacking them in any other
    # order silently binds axis_key to "lin"/"ang" and the model lookup dies
    # with KeyError. That is exactly what happened on the first run.
    for axis, sign, axis_key in AXES:
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
            # Observed motion, DIFFERENTIATED FROM POSE. These are the only
            # numbers any decision in this file is allowed to be made on.
            self.pose_vx = None       # real m/s along the previous heading
            self.pose_wz = None       # real rad/s about z
            self.twist_vx = None      # the board's own claim, sign-corrected
            self._prev = None         # (x, y, yaw, t) of the previous frame
            # Bumped once per frame that yielded a velocity. Sampling keys off
            # this instead of the control tick -- see sample().
            self.vel_seq = 0
            self.odom_last = None
            self.battery_v = None

        def _on_odom(self, msg):
            """Velocity from POSE DELTAS, the same way fpms_teleop.py feeds its
            lurch guard.

            The previous version of this method read msg.twist.twist.linear.x,
            and that single line is what aborted the first live sweep: twist is
            sign-inverted relative to the pose in its own message, so the
            wrong-way check fired on every correct step and would have passed a
            real reversal. Pose was right in every trial ever run on this
            board. Do not put twist back in front of a guard."""
            try:
                x = float(msg.pose.pose.position.x)
                y = float(msg.pose.pose.position.y)
                yaw = yaw_from_quat(msg.pose.pose.orientation)
                if not (math.isfinite(x) and math.isfinite(y)
                        and math.isfinite(yaw)):
                    return
                now = time.monotonic()
                with self.lock:
                    if self._prev is not None:
                        px, py, pyaw, pt = self._prev
                        dt = now - pt
                        # Same absurd-gap rule as fpms_teleop's integrator: a
                        # stalled link resuming must not manufacture a huge
                        # apparent velocity, which here would abort the sweep.
                        if 0.0 < dt < 0.5:
                            # Signed along the heading the frame STARTED from,
                            # so a skid-steer yaw does not read as translation.
                            along = ((x - px) * math.cos(pyaw)
                                     + (y - py) * math.sin(pyaw))
                            vx = along / dt
                            wz = wrap_pi(yaw - pyaw) / dt
                            # Light smoothing, matching fpms_teleop. One noisy
                            # frame must not be able to abort a sweep; a real
                            # wrong-way move lasts many frames and still shows
                            # through, as does the spiky signature the smooth
                            # test looks for.
                            self.pose_vx = (vx if self.pose_vx is None
                                            else 0.5 * self.pose_vx + 0.5 * vx)
                            self.pose_wz = (wz if self.pose_wz is None
                                            else 0.5 * self.pose_wz + 0.5 * wz)
                            self.vel_seq += 1
                    self._prev = (x, y, yaw, now)
                    self.odom_last = now
                    # Diagnostic only -- recorded, never classified on. See
                    # ODOM_TWIST_SIGN for why this is the single read site.
                    tw = float(msg.twist.twist.linear.x)
                    if math.isfinite(tw):
                        self.twist_vx = ODOM_TWIST_SIGN * tw
            except Exception as e:
                log(f"odom callback error {e}")

        def _on_battery(self, msg):
            try:
                with self.lock:
                    self.battery_v = int(msg.data) / 10.0
            except Exception as e:
                log(f"battery callback error {e}")

        def sample(self, axis):
            """(pose velocity, frame counter, sign-corrected twist).

            The counter is returned so the caller can sample once per NEW odom
            frame rather than once per control tick: /odom_raw runs at ~10 Hz
            and the control loop at 20, so tick-rate sampling would enter every
            reading twice and halve the apparent spread that the smooth test is
            entirely built on."""
            with self.lock:
                v = self.pose_vx if axis == "lin" else self.pose_wz
                return v, self.vel_seq, self.twist_vx

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
        early, late, twist_late = [], [], []
        # Only a NEW odom frame is a new sample; see DeadbandNode.sample().
        _, last_seq, _ = node.sample(axis)
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

            val, seq, tw = node.sample(axis)
            if val is not None and seq != last_seq:
                last_seq = seq
                # Checked on every frame, not just at the end of the hold: if
                # the rover is genuinely running away from the command, the
                # right response is to cut to zero now rather than keep
                # commanding while it finds out how far it can go.
                #
                # `val` is DIFFERENTIATED POSE. This is the check that fired on
                # correct motion and killed the first live sweep when it was
                # fed twist.linear.x instead (NAV2_BRIEF.md 3a).
                if commanded != 0 and (val * cmd_sign) < -wrong_tol:
                    raise AbortSweep(
                        f"pose says {axis}={val:+.4f} opposite to commanded "
                        f"{commanded:+.4f} beyond tolerance {wrong_tol:.3f} "
                        "-- rover is moving the wrong way")
                if t_rel <= cfg["sample_window_s"]:
                    early.append(val)
                if t_rel >= cfg["hold_s"] - cfg["sample_window_s"]:
                    late.append(val)
                    if tw is not None:
                        twist_late.append(tw)

        # >=1.0s of published zero here is a settle window, not a pause
        # between steps: a sub-floor setpoint drives the chassis
        # uncontrolled, and the next step is LARGER, so the rover is brought
        # to a stop and left there before it is asked for more. See the
        # module docstring and the --gap-s validation in main().
        t1 = time.monotonic()
        while time.monotonic() - t1 < cfg["gap_s"]:
            if stop_flag.is_set():
                raise AbortSweep("operator interrupt (signal) during zero-gap")
            node.publish(axis, 0.0)
            rclpy.spin_once(node, timeout_sec=dt)
            check_watchdogs(node)

        result = _finish_step(axis, sign, mag, commanded, early, late, cfg,
                              simulated=False, twist_late=twist_late)
        if result["wrong_way"]:
            raise AbortSweep(
                f"post-step check: {axis} pose mean_late="
                f"{result['mean_late_pose']} opposite to commanded "
                "-- aborting sweep")
        return result

    rclpy.init(args=None)
    node = DeadbandNode()
    try:
        # Same unpack order as run_dry_sweep: AXES is (axis, sign, axis_key).
        for axis, sign, axis_key in AXES:
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
    print("FIRMWARE VELOCITY-FLOOR SWEEP SUMMARY")
    print("=" * 78)
    print("Lowest /cmd_vel setpoint the firmware's velocity loop can HOLD.")
    print("This is a QUANTISATION floor (integer encoder counts per 10ms PID")
    print("period), NOT a mechanical deadband. Nothing below is a claim about")
    print("stiction, torque or wheel load. All magnitudes are COMMANDED WIRE")
    print("units; the motion behind them was measured as differentiated pose.")
    print("-" * 78)
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
    print("(*) first SMOOTH PROPORTIONAL motion -- the lowest setpoint the")
    print("    velocity loop actually regulated. 'first motion' above it is")
    print("    the loop failing to regulate: quantisation noise, or the 50%")
    print("    feed-forward duty breaking through. Motion is not control.")

    def recommend(axis):
        p, m = recs.get(f"{axis}+"), recs.get(f"{axis}-")
        vals = [abs(v) for v in (p, m) if v is not None]
        rec = (max(vals) * MARGIN_FACTOR) if vals else None
        return rec, p, m

    print()
    print("LOWEST SETPOINT WORTH COMMANDING (wire units)")
    print("  NOT a deadband, and NOT a floor to snap commands up to. Below")
    print("  this the loop is blind, so the answer is to never ask for it:")
    print("  plan motion as short bounded segments with stops between, run")
    print("  at a speed the loop can hold. See NAV2_BRIEF.md section 3b.")
    for axis, label, cap_key in (("lin", "HOLDABLE_LIN", "cap_lin"),
                                 ("ang", "HOLDABLE_ANG", "cap_ang")):
        rec, p, m = recommend(axis)
        if rec is None:
            print(f"  {label} = UNKNOWN -- neither direction confirmed "
                  f"smooth motion within cap {cfg[cap_key]:.4f}; rerun with "
                  f"a higher --cap-{axis}.")
            continue
        if p is not None and m is not None:
            detail = f"(= max({p:.4f}, {m:.4f}) x {MARGIN_FACTOR:.2f} margin)"
            rel = abs(abs(p) - abs(m)) / max(abs(p), abs(m)) if max(abs(p), abs(m)) else 0.0
            asym = (f"\n  NOTE: {axis}+ and {axis}- differ by {rel*100:.0f}%. "
                    "Quantisation alone is symmetric, so a gap this large is "
                    "either a measurement artefact or something real that is "
                    "NOT the quantisation floor -- investigate before "
                    "quoting either number.") if rel > 0.20 else ""
        else:
            have = p if p is not None else m
            detail = f"(only one direction confirmed: {have:.4f}; x {MARGIN_FACTOR:.2f} margin)"
            asym = ""
        print(f"  {label} = {rec:.4f}   {detail}{asym}")

    # The prediction, printed next to the result. A measured linear floor far
    # below one encoder count per PID period means the classifier called
    # quantisation noise "motion", not that the firmware beat its own maths.
    rec_lin, _, _ = recommend("lin")
    print()
    print(f"  Predicted from firmware constants: {QUANT_FLOOR_WIRE:.4f} wire "
          "(1 encoder count / 10ms PID period).")
    if rec_lin is not None:
        ratio = rec_lin / QUANT_FLOOR_WIRE
        print(f"  Measured linear result is {ratio:.2f}x that.", end=" ")
        print("Far below 1.0 means the measurement is suspect, not the maths."
              if ratio < 0.7 else
              "Consistent with the quantisation explanation."
              if ratio < 2.0 else
              "Well above it -- something else is also limiting; do not "
              "attribute all of it to quantisation.")

    print()
    print("CAVEAT: this is a NO-LOAD measurement (wheels off the ground,")
    print("--on-blocks). Load does not change the quantisation floor -- the")
    print("counts-per-period maths is the same -- but it can add a real")
    print("stiction requirement ON TOP. Treat every number above as a LOWER")
    print("BOUND, verify on the floor, and round up when in doubt.")
    print()
    print("NOT FIXABLE BY TUNING. No PID gain and no MIN_CMD constant buys")
    print("the loop resolution it does not have; the fix is the firmware's")
    print("encoder/wheel constants, i.e. a rebuild.")

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
        description="Measure the lowest /cmd_vel setpoint this rover's "
                    "firmware velocity loop can actually HOLD. That floor is "
                    "quantisation (integer encoder counts per PID period), "
                    "not a mechanical deadband -- the file name predates the "
                    "finding. See the module docstring before running it for "
                    "real.",
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
            "refuse: --gap-s must be >= 1.0s. A setpoint under the firmware's "
            "velocity floor drives the chassis uncontrolled, and the next "
            "step is always LARGER; this is the window in which the rover is "
            "commanded to zero and allowed to settle before being asked for "
            "more (see module docstring). Less than 1.0s risks carrying "
            "motion forward into the next step's measurement.")
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
            "This sweep commands real motor motion, and most of its steps "
            "sit BELOW the setpoint the firmware's velocity loop can hold "
            "(~0.0145 on the wire = one encoder count per 10ms PID period). "
            "Down there the loop has no feedback to regulate against and the "
            "50% feed-forward duty is all that reaches the motors, so the "
            "rover stalls and then moves in a way nothing is controlling. "
            "Measured: 0.0041 commanded produced ~3s of stillness and then a "
            "290mm move.\n"
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
