#!/usr/bin/env python3
"""fpms_motion against an INDEPENDENT plant model.

    python3 selftest/test_motion_plant.py

Exit 0 all pass, 1 one or more fail.

WHY A SECOND PLANT
==================
docs/MOTION.md 8.2 quotes a simulation written by the same author as the
controller. That is worth something, but a modelling mistake shared between a
controller and its own test bench agrees with itself perfectly and reports
success. This file is the second opinion: it uses the PUBLIC API only, knows
nothing about the controller's internals, and models the chassis in the most
ordinary way available - a first-order lag between commanded and actual rate.

It has already earned its place. Against this plant every turn from 15 to 180
deg arrived a consistent 3.0-4.7 deg PAST target - the same direction every
time, ~= w_peak * tau - while the author's own plant reported +0.35 deg. The
disagreement was real and the bug was real: _command_v decided when to cut
from POSITION alone, while the carry after the cut is set by the SPEED at that
moment, and the chassis trails its command by tau for the whole decel ramp.
See MOTION.md 8.2.1.

WHAT THIS IS NOT
================
It is not a rover. Both plants assume a first-order lag with a KNOWN tau, and
tau is UNMEASURED on this chassis. A turn that is perfect here and a turn that
is perfect on the arena floor are different claims, and only the second one
matters. The tau-mismatch case below exists to keep that honest.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.join(HERE, "..", "overlay", "usr", "local", "lib", "fpms")
sys.path.insert(0, os.path.normpath(LIB))

try:
    import fpms_motion as M
except Exception as exc:                                  # pragma: no cover
    # SKIP LOUDLY. A test that cannot import the thing it tests must never
    # look like a pass - that is how a module gets deleted and nothing goes red.
    print(f"SKIP: cannot import fpms_motion from {LIB}: {exc}")
    sys.exit(1)

DT = 0.05                      # 20 Hz, CONTROL_HZ
FAILURES = []
CHECKS = [0]


def check(name, ok, detail=""):
    CHECKS[0] += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:<52} {detail}")
    if not ok:
        FAILURES.append(name)


# A calibration that is entirely INVENTED. Real values do not exist yet; these
# are internally consistent and in-range so the controller will run at all.
# Nothing here may be copied into calibration.json.
SIM = {
    "measured": True,
    "firmware": "v3",
    "measured_at": "SIMULATED - NOT A MEASUREMENT",
    "counts_per_mm": 5.5,
    "gyro_scale": 1.0,
    "mps_per_duty": 0.010,
    "radps_per_duty": 0.020,
    "min_moving_duty": 8.0,
    "duty_forward_sign": 1,
    "side_map_confirmed": True,
}


def run_turn(angle_deg, tau_plant, tau_assumed=0.15, max_s=40.0):
    """Drive TurnInPlace against a first-order-lag plant. Returns (err_deg,
    reversals, seconds). The controller sees only the gyro integral, exactly
    as it would on the rover."""
    cal = M.Calibration.from_profile(SIM)
    ctl = M.TurnInPlace(math.radians(angle_deg), cal,
                        tau_actuator_s=tau_assumed, dt_nominal=DT)
    st = ctl.start(0.0, 0.0)
    yaw = omega = t = 0.0
    reversals, prev = 0, 0
    out = None
    while t < max_s:
        st, out = ctl.step(st, t, yaw, omega)
        d = out.command.duty                      # [FL, FR, RL, RR]
        diff = ((d[0] + d[2]) - (d[1] + d[3])) / 2.0
        target_w = -SIM["radps_per_duty"] * diff
        omega += (target_w - omega) * (DT / tau_plant)
        yaw += omega * DT
        s = 0 if diff == 0 else (1 if diff > 0 else -1)
        if s and prev and s != prev:
            reversals += 1
        if s:
            prev = s
        t += DT
        if out.done:
            break
    return math.degrees(yaw) - angle_deg, reversals, t


def run_drive(dist_mm, tau_plant, max_s=60.0):
    cal = M.Calibration.from_profile(SIM)
    run = M.StraightRun(dist_mm, cal, dt_nominal=DT)
    st = run.start(0.0, 0.0)
    travel = v = t = 0.0
    while t < max_s:
        st, out = run.step(st, t, travel, 0.0, 0.0)
        mean = sum(out.command.duty) / 4.0
        v += (SIM["mps_per_duty"] * mean - v) * (DT / tau_plant)
        travel += v * 1000.0 * DT
        t += DT
        if out.done:
            break
    return travel - dist_mm, t


print(__doc__.strip().splitlines()[0])
print()
print("Turns, plant tau == assumed tau == 0.15 s")
print("-" * 72)

# 1.0 deg is the controller's own default tol_rad, so anything inside it is at
# the limit of what the terminal band even claims to deliver. Before the
# measured-rate cut band this was 4.66 deg.
worst = 0.0
for a in (15, 30, 45, 90, -90, 135, 180, -180):
    err, rev, secs = run_turn(a, 0.15)
    worst = max(worst, abs(err))
    check(f"turn {a:>5} deg lands within 1.0 deg", abs(err) <= 1.0,
          f"err {err:+.3f} deg in {secs:.2f}s")
    # THE ANTI-HUNTING CLAIM. A turn that reverses direction is hunting even
    # if it happens to stop in the right place.
    check(f"turn {a:>5} deg never reverses direction", rev == 0,
          f"{rev} reversals")

check("worst turn error over all angles < 1.0 deg", worst < 1.0,
      f"worst |err| = {worst:.3f} deg")

# THE BIAS TEST, and the reason this file exists. A signed MEAN near zero is a
# much stronger claim than a small magnitude: a retrace cancels a random error
# and cannot cancel a bias, so a consistent 4 deg overshoot is far worse than a
# random 4 deg scatter. Sample one sense only - +/- pairs cancel by symmetry
# and would make any bias invisible.
errs = [run_turn(a, 0.15)[0] for a in (15, 30, 45, 90, 135, 180)]
mean_err = sum(errs) / len(errs)
check("turn error is not a systematic bias", abs(mean_err) <= 0.75,
      f"signed mean {mean_err:+.3f} deg over 6 angles (was +3.97 before the fix)")

print()
print("Turns, plant tau MISMATCHED against an assumed 0.15 s")
print("-" * 72)
# tau is UNMEASURED. These are not pass/fail on accuracy - they are pass/fail
# on DIRECTION: under-estimating tau must never produce a runaway or a hunt,
# because that is the failure an operator cannot recover from.
for tp in (0.30, 0.60):
    err, rev, _ = run_turn(90, tp)
    check(f"90 deg with plant tau {tp:.2f}s does not hunt", rev == 0,
          f"err {err:+.2f} deg, {rev} reversals")
    check(f"90 deg with plant tau {tp:.2f}s stays bounded", abs(err) < 30.0,
          f"err {err:+.2f} deg")

print()
print("Straight runs, plant tau 0.15 s")
print("-" * 72)
for d in (100.0, 250.0, 744.0, 1000.0):
    err, secs = run_drive(d, 0.15)
    # 2 % OR 12 mm. This is deliberately looser than the turn tolerance and it
    # is NOT a statement that 12 mm is acceptable: MOTION.md 8.2.1 records a
    # known 5-9 mm undershoot caused by arrive_tol being used as a cut
    # threshold. The bound is here to catch a REGRESSION, not to bless the gap.
    lim = max(12.0, 0.02 * d)
    check(f"drive {d:>7.1f} mm within {lim:.0f} mm", abs(err) <= lim,
          f"err {err:+.2f} mm in {secs:.2f}s")

# Continuous motion is the entire point: 1 m must not take burst-executor time.
_, secs = run_drive(1000.0, 0.15)
check("1000 mm completes in under 10 s", secs < 10.0, f"{secs:.2f}s")

print()
print("=" * 72)
if FAILURES:
    print(f"  {len(FAILURES)} of {CHECKS[0]} checks FAILED")
    for f in FAILURES:
        print(f"    - {f}")
    print("=" * 72)
    sys.exit(1)
print(f"  all {CHECKS[0]} checks passed")
print("  This is a SIMULATION. tau is unmeasured, the calibration above is")
print("  invented, and no rover has run this code.")
print("=" * 72)
sys.exit(0)
