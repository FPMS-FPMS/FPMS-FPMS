#!/usr/bin/env python3
"""
FPMS CALIBRATION FITTER -- offline, stdlib-only, and willing to say no.

    python3 fpms_learn_fit.py /home/ubuntu/fpms_learn_data
    python3 fpms_learn_fit.py runs-20260807.ndjson --json fit.json
    python3 fpms_learn_fit.py DIR --verbose

Reads the NDJSON written by `fpms_learn_recorder.py` and estimates the drive
constants this project has been arguing about, WITH confidence intervals and
an explicit sample count on every number.

    counts_per_mm         encoder scale          disputed 5.5 / 6.0 / 14.8
    effective_track_mm    skid-steer track       disputed 105 / 170 / 255 / 271
    wheel_slip[0..3]      per-wheel slip factor  never measured
    duty_velocity         v = a * (duty - u0)    never measured
    stall_duty            u0, the breakaway      never measured
    min_pulse_s           under LOAD             measured WHEELS OFF only

THIS IS PARAMETER ESTIMATION, NOT MACHINE LEARNING. It is ordinary least
squares on a few dozen numbers. Nothing here is trained, nothing generalises
beyond this chassis, and there is no model file. Said plainly in README_LEARN.md
and repeated here so nobody has to open a second document to find it out.

--------------------------------------------------------------------------
THE REFUSAL RULE, WHICH IS THE POINT OF THIS FILE
--------------------------------------------------------------------------
Every estimator here can be run on three samples and will happily print a
number with a tiny standard error, because with three samples the residual
variance is estimated from one degree of freedom and is meaningless. So each
estimator carries TWO gates and must pass both:

  1. A HARD FLOOR on n, and on the design (how much the inputs actually vary).
     A perfect regression through five points that all sit at the same
     commanded distance determines the slope not at all, however small its
     residual looks.

  2. A RESOLUTION gate: the 95% CI must be NARROWER THAN THE DISPUTE it is
     supposed to settle. Estimating counts/mm as 5.7 +/- 0.9 does not choose
     between 5.5 and 6.0 -- it contains both, and reporting it as "5.7" would
     retire an open question by rounding. Where a parameter has competing
     published values, the required precision is derived FROM THAT GAP, and
     the fitter reports how many more segments are needed to reach it.

A refusal names what is missing and how much more of it is needed. A refusal
is a result.

--------------------------------------------------------------------------
THE REFERENCE PROBLEM -- READ THIS BEFORE TRUSTING ANY NUMBER BELOW
--------------------------------------------------------------------------
Encoders cannot calibrate encoders. counts_per_mm can only be fitted against
a length reference that does not come from the encoders:

  ACCEPTED   `truth` records -- an operator's tape measure, appended by hand
             to the NDJSON. The gold standard, and the only one that is
             traceable to a real metre.
  ACCEPTED   LiDAR front-wall closure across a straight run at a flat wall.
             Independent of the drivetrain. Noisier, and only valid when the
             heading barely changed and both readings are sane, which is
             gated below.
  REFUSED    /odom and /odom_raw pose deltas. The board integrates those from
             these same encoders using its own COUNTS_PER_REV, so regressing
             ticks on odom recovers the FIRMWARE'S ASSUMED CONSTANT and
             nothing else. It would produce a beautiful r^2 of 1.000 and a
             confidently wrong answer. It is computed and printed as a
             CONSISTENCY CHECK -- if it does not come out at the firmware's
             own constant, the odom pipeline has a second bug -- but it is
             never allowed to be the headline estimate.

--------------------------------------------------------------------------
WHY TURNS ARE CURRENTLY UNIDENTIFIABLE
--------------------------------------------------------------------------
`effective_track_mm` is the constant that makes

    dyaw = (d_right - d_left) / track

true. Fitting it needs a MEASURED dyaw from something that is not the wheels.
On this build /imu angular_velocity is identically 0.0 -- every turn in the
dataset measures 0 degrees -- and /odom's yaw is derived on the board from
these same wheels with the board's own track, which is circular twice over.

So the estimator below is written, tested against the data format, and
REFUSES, naming the gyro. It will start returning numbers on the first
dataset recorded after the gyro deadband is fixed, with no change here. The
manual escape hatch is a `truth` record carrying `true_deg` (an operator
marking the floor and turning the rover 360 degrees by hand); the estimator
accepts those too.

Everything downstream of the track is stuck behind the same wall:
TURN_WIRE_SIGN cannot be confirmed from data in which every turn reads zero,
and MIN_TURN_DEG is TURN_RADPS * MIN_PULSE_S, whose first factor is the
thing that cannot be measured.
"""
import os
import sys
import json
import math
import glob
import statistics
from collections import defaultdict

SCHEMA_MIN = 3

# =========================================================== SAMPLE FLOORS
# Every one of these is a HARD floor. Passing it is necessary, never
# sufficient -- the resolution gate below still has to pass too.

# counts_per_mm. 16 is not a ritual number: with a per-segment scatter around
# 8% (which is what a chassis whose rear-right wheel slips 24% produces), a
# 95% CI half-width of 4.5% -- half the 5.5-vs-6.0 gap, i.e. just enough to
# choose between them -- needs n ~ (t*sigma/target)^2 = (2.13*0.08/0.045)^2
# ~= 14. Rounded up, with two spare for the segments that get rejected by the
# straightness gate. The fitter recomputes the real requirement from the
# OBSERVED scatter and reports it, so this floor is only the starting point.
MIN_N_CPM = 16
MIN_LEVER_CPM = 2.5        # max(|d|)/min(|d|); a regression needs a lever arm
MIN_TRAVEL_MM = 60.0       # below MIN_MOVE_MM the chassis cannot express it
MIN_REFS_CPM = 3           # distinct reference OBSERVATIONS, not segments

# per-wheel slip. Both directions, because a wheel that reads long forward
# and short backward is a wiring/backlash asymmetry, not slip, and averaging
# the two would hide it as a clean 1.00.
MIN_N_SLIP = 12
MIN_PER_DIR_SLIP = 4

# duty -> velocity. Five duty levels to see curvature, three reps each for a
# variance estimate at each level, and at least two observations where a
# non-zero duty produced NO motion -- without those the stall threshold is an
# extrapolation off the end of the data rather than a measurement.
MIN_DUTY_LEVELS = 5
MIN_REPS_PER_DUTY = 3
MIN_ZERO_MOTION_OBS = 2

# effective track. Turns, in both directions, big enough that the encoder
# quantisation is not the dominant term.
MIN_N_TRACK = 12
MIN_PER_DIR_TRACK = 4
MIN_TURN_DEG = 20.0

# MIN_PULSE_S under load: distinct commanded pulse durations bracketing the
# threshold, with repeats so a single unlucky burst does not set it.
MIN_PULSE_DURATIONS = 6
MIN_PULSE_REPS = 3
# A burst that moved nothing is only evidence about the STALL DUTY if it was
# long enough to have moved had the duty been sufficient. Shorter than this and
# it is evidence about MIN_PULSE_S instead. Set above the configured 0.35 s
# (itself measured wheels-off, so a floor) with margin, because under load
# MIN_PULSE_S can only be longer than the wheels-off figure, never shorter.
MIN_STALL_EVIDENCE_PULSE_S = 0.60

# Straightness gate for a segment used as a DISTANCE sample: the two front
# wheels must agree. A run that curved is a run whose path length is not its
# displacement, and the wall reference measures displacement.
STRAIGHT_MAX_FRONT_DIFF_FRAC = 0.10
STRAIGHT_MAX_ODOM_DYAW_DEG = 8.0
# LiDAR wall pair sanity: both readings present, in range, and the closure a
# sane fraction of what the ticks suggest at ANY candidate scale.
LIDAR_MIN_MM, LIDAR_MAX_MM = 120.0, 4000.0

# The published values this fitter is trying to arbitrate between.
CPM_CANDIDATES = {"hand push (fpms_drive.py)": 5.5,
                  "derived, TICKS_PER_REV 1320": 6.0,
                  "tape 2026-08-04 (unconfirmed)": 14.8}
TRACK_CANDIDATES = {"fpms_rtos_follower.py": 105.0,
                    "fpms_odom_tf.py geometric": 170.0,
                    "fpms_drive.py effective": 255.0,
                    "session notes": 271.0}


# ============================================================== STATISTICS
# Student's t, two-sided 95%, by degrees of freedom. Hard-coded rather than
# approximated because at the sample sizes this fitter works with (n = 10-30)
# the normal approximation understates the interval by 5-15%, and an interval
# that is too narrow is exactly the failure mode this whole file exists to
# prevent.
_T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447,
        7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179,
        13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101,
        19: 2.093, 20: 2.086, 21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064,
        25: 2.060, 26: 2.056, 27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042,
        40: 2.021, 60: 2.000, 120: 1.980}


def t95(dof):
    if dof <= 0:
        return float("inf")
    if dof in _T95:
        return _T95[dof]
    if dof > 120:
        return 1.960
    keys = sorted(_T95)
    hi = min(k for k in keys if k >= dof)
    lo = max(k for k in keys if k <= dof)
    if hi == lo:
        return _T95[hi]
    f = (dof - lo) / (hi - lo)
    return _T95[lo] + f * (_T95[hi] - _T95[lo])


def ols(xs, ys):
    """y = slope*x + intercept, ordinary least squares.

    ASSUMPTIONS, stated because they are all violable on this rover:
      * x (the reference distance) is measured without error. A tape is close
        enough; the LiDAR is not, and errors-in-variables biases the slope
        TOWARD ZERO, so a LiDAR-only counts/mm estimate is an UNDER-estimate.
        Reported as such rather than silently corrected.
      * residuals are independent with constant variance. Wheel slip is
        neither -- it is worse at higher duty and correlated within a run --
        so the interval below is optimistic. Treated as a floor on the
        uncertainty, never a ceiling.
      * the relationship is linear through the range sampled. True for an
        encoder; not true for the duty curve near stall, which is why that
        one is fitted piecewise.
    """
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    inter = my - slope * mx
    resid = [y - (slope * x + inter) for x, y in zip(xs, ys)]
    dof = n - 2
    if dof <= 0:
        return None
    s2 = sum(r * r for r in resid) / dof
    se_slope = math.sqrt(s2 / sxx)
    se_inter = math.sqrt(s2 * (1.0 / n + mx * mx / sxx))
    syy = sum((y - my) ** 2 for y in ys)
    t = t95(dof)
    return {"n": n, "slope": slope, "intercept": inter,
            "se_slope": se_slope, "se_intercept": se_inter,
            "ci95_slope": [slope - t * se_slope, slope + t * se_slope],
            "ci95_intercept": [inter - t * se_inter, inter + t * se_inter],
            "resid_sd": math.sqrt(s2), "dof": dof,
            "r2": (1.0 - sum(r * r for r in resid) / syy) if syy > 0 else None,
            "x_min": min(xs), "x_max": max(xs), "x_mean": mx}


def ols_origin(xs, ys):
    """y = slope*x, forced through the origin.

    The right model whenever the intercept is known a priori to be zero --
    zero millimetres of travel is zero encoder counts, always -- and it buys
    back a degree of freedom, which matters at n = 16. The free-intercept fit
    is computed alongside precisely so a NON-ZERO intercept can be spotted:
    that would mean the reference has an offset (a tape read from the wrong
    mark, or a LiDAR pair taken at different rover attitudes), and the
    through-origin number would then be quietly wrong."""
    n = len(xs)
    if n < 2:
        return None
    sxx = sum(x * x for x in xs)
    if sxx <= 0:
        return None
    slope = sum(x * y for x, y in zip(xs, ys)) / sxx
    resid = [y - slope * x for x, y in zip(xs, ys)]
    dof = n - 1
    s2 = sum(r * r for r in resid) / dof
    se = math.sqrt(s2 / sxx)
    t = t95(dof)
    return {"n": n, "slope": slope, "se_slope": se, "dof": dof,
            "ci95_slope": [slope - t * se, slope + t * se],
            "resid_sd": math.sqrt(s2)}


def mean_ci(vals):
    n = len(vals)
    if n < 2:
        return None
    m = statistics.fmean(vals)
    sd = statistics.stdev(vals)
    se = sd / math.sqrt(n)
    t = t95(n - 1)
    return {"n": n, "mean": m, "sd": sd, "se": se,
            "ci95": [m - t * se, m + t * se],
            "median": statistics.median(vals)}


def ratio_ci(num, den, se_num, se_den, dof):
    """95% CI for num/den by the delta method, with a Fieller guard.

    Used for the stall duty, which is an x-intercept -b/a: a ratio of two
    estimates. The delta method is a first-order approximation and it FAILS
    when the denominator is not comfortably away from zero -- at which point
    the true interval is unbounded and any number printed would be fiction.
    The guard below detects that and returns None so the caller refuses."""
    if den == 0 or abs(den) < 3.0 * se_den:
        return None                      # denominator indistinguishable from 0
    r = num / den
    var = (se_num ** 2) / (den ** 2) + (num ** 2) * (se_den ** 2) / (den ** 4)
    se = math.sqrt(max(var, 0.0))
    t = t95(dof)
    return {"value": r, "se": se, "ci95": [r - t * se, r + t * se]}


def required_n(resid_rel_sd, target_rel_halfwidth, floor):
    """How many samples the OBSERVED scatter says are needed to hit a target
    precision. n ~ (t * sigma / halfwidth)^2, iterated because t depends on n.

    This is what turns "not enough data" into "seven more segments"."""
    if not resid_rel_sd or target_rel_halfwidth <= 0:
        return None
    n = max(floor, 4)
    for _ in range(60):
        nn = math.ceil((t95(n - 2) * resid_rel_sd / target_rel_halfwidth) ** 2) + 2
        if nn == n:
            break
        n = max(4, nn)
    return max(int(n), floor)


def rel_halfwidth(est):
    lo, hi = est["ci95_slope"]
    if est["slope"] == 0:
        return None
    return (hi - lo) / 2.0 / abs(est["slope"])


# ================================================================= LOADING
def load(paths):
    """Read every NDJSON file given, or every runs-*.ndjson in a directory."""
    files = []
    for p in paths:
        if os.path.isdir(p):
            files += sorted(glob.glob(os.path.join(p, "runs-*.ndjson")))
            files += sorted(glob.glob(os.path.join(p, "*.ndjson")))
        else:
            files.append(p)
    files = sorted(set(files))
    recs, bad = [], 0
    for f in files:
        try:
            with open(f, encoding="utf-8") as fh:
                for ln, line in enumerate(fh, 1):
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        r = json.loads(line)
                        r["_file"] = os.path.basename(f)
                        r["_line"] = ln
                        recs.append(r)
                    except Exception:
                        # A truncated last line is normal after a power cut.
                        bad += 1
        except OSError as e:
            print(f"  ! cannot read {f}: {e}", file=sys.stderr)
    return files, recs, bad


def index(recs):
    """Split into the shapes the estimators want, joined on `seq`.

    TWO SOURCES OF `asked`, AND ONE OF THEM DOES NOT EXIST ON THIS ROVER.

      `join`            built by the recorder from `telemetry/residual`. That
                        topic is documented in STACK.md, advertised on six ROS
                        topics, mirrored by the foxglove bridge, subscribed by
                        two files -- and PUBLISHED BY NOTHING. The deployed
                        fpms_missions.py has no `_publish_residual` at all (the
                        repo mirror does; the two copies diverged). So this is
                        expected to be empty, and it is read anyway because it
                        starts working by itself the day that is closed.
      `residual_join`   built by the recorder from `events/mission_done` ->
                        `measured[]`, which this rover really does emit. This
                        is where the commanded-vs-measured numbers come from
                        today.

    The second is a POSITIONAL join -- the k-th episode of a run is the k-th
    executed segment -- and the recorder only emits pairs when the two counts
    agree exactly. `confident: false` records are read for the diagnostics they
    carry and are never allowed to supply an `asked`, because a residual
    attached to the wrong segment enters the fit as a real measurement and
    there is no way to tell afterwards."""
    segs, truths, joins, hb = [], {}, {}, []
    md_joins, md_unmatched, dones = {}, 0, []
    for r in recs:
        k = r.get("rec")
        if k == "segment":
            if int(r.get("schema", 0)) < SCHEMA_MIN:
                continue
            segs.append(r)
        elif k == "truth":
            s = r.get("seq")
            if s is not None:
                truths[int(s)] = r
        elif k == "join" and r.get("matched") and r.get("seq") is not None:
            joins[int(r["seq"])] = r
        elif k == "residual_join":
            if r.get("confident"):
                for p in r.get("pairs") or []:
                    if p.get("seq") is not None:
                        q = dict(p)
                        q["_source"] = "events/mission_done"
                        md_joins[int(p["seq"])] = q
            else:
                md_unmatched += 1
        elif k == "mission_done":
            dones.append(r)
        elif k == "health":
            hb.append(r)
    for s in segs:
        s["_truth"] = truths.get(s.get("seq"))
        # telemetry/residual wins if it ever exists -- it is per-segment and
        # timestamped rather than positional. It does not exist today.
        s["_join"] = joins.get(s.get("seq")) or md_joins.get(s.get("seq"))
    meta = {"joins_from_telemetry_residual": len(joins),
            "joins_from_mission_done": len(md_joins),
            "runs_with_unmatched_episode_counts": md_unmatched,
            "missions_done": len(dones)}
    return segs, truths, joins, hb, meta


# =============================================================== SELECTION
def is_straight(s):
    """A segment usable as a straight-line DISTANCE sample.

    Rejects anything that curved, because the wall reference and the tape
    both measure DISPLACEMENT while the encoders measure PATH LENGTH, and on
    a curve those are different numbers. Also rejects anything the executor
    called a turn."""
    m = s.get("measured") or {}
    ctx = s.get("context") or {}
    j = s.get("_join") or {}
    if ctx.get("segment_kind") == "turn" or j.get("kind") == "turn":
        return False, "turn segment"
    med = m.get("ticks_median")
    if med is None or abs(med) < MIN_TRAVEL_MM * 1.0:
        # 1 count/mm is the loosest candidate scale; anything under
        # MIN_TRAVEL_MM counts is short at EVERY candidate.
        return False, "too short at every candidate scale"
    df = m.get("tick_diff_front_counts")
    if df is None:
        return False, "no front tick differential"
    if abs(df) > STRAIGHT_MAX_FRONT_DIFF_FRAC * abs(med):
        return False, f"curved: front wheels differ by {abs(df)/abs(med):.0%}"
    d = ((s.get("odom") or {}).get("delta") or {})
    dy = d.get("dyaw_deg")
    if dy is not None and abs(dy) > STRAIGHT_MAX_ODOM_DYAW_DEG:
        return False, f"odom says it rotated {dy:.1f} deg"
    return True, None


def wheel_responses(s):
    """The three ways this codebase combines four wheels into one distance.

    THEY ARE NOT INTERCHANGEABLE, AND THE FITTED CONSTANT IS ONLY VALID FOR
    THE ONE IT WAS FITTED ON. That is the single most important thing in this
    file after the reference problem, and it is not obvious:

      median4  the mean of the 2nd and 3rd of four sorted deltas. What
               fpms_drive.dist_mm uses. Chosen there because it is immune to
               one bad wheel in either direction -- which is true of the
               VALUE but not of the BIAS. With one wheel persistently high
               (the rear-right reads +24% under power on this rover) the
               median of four degenerates to the mean of the top TWO of the
               remaining three, so it rides UP with per-wheel noise:
               +0.6% at 1% inter-wheel scatter, +1.3% at 3%, +2.6% at 6%.
      front2   the mean of the two front wheels. What fpms_drive.tick_yaw
               uses for heading, after checking it against a watched floor
               run. Stable against noise -- it does not move at all across
               that same 1%-to-6% sweep.
      mean4    the plain mean. Carries the full slip of every wheel,
               including the bad one, and so reads ~6% long here.

    A counts_per_mm fitted on front2 and then pasted into code that measures
    with median4 is wrong by 1-3%, in a way that gets WORSE as the wheels get
    worse. Against a 5.5-vs-6.0 dispute that is 9% wide, that is not a
    rounding error. All three are therefore fitted and reported, and the
    consumer must take the one matching its own combining rule."""
    m = s.get("measured") or {}
    pw = m.get("ticks_per_wheel")
    med = m.get("ticks_median")
    if med is None:
        return None
    out = {"median4": float(med)}
    if pw and len(pw) == 4:
        out["front2"] = (float(pw[0]) + float(pw[1])) / 2.0
        out["mean4"] = sum(float(v) for v in pw) / 4.0
    return out


def distance_samples(segs):
    """(responses, reference_mm, source, seq) for every usable pair.

    A tape reading wins over a LiDAR pair for the same segment: it is
    traceable and the LiDAR is not."""
    out, rejected = [], []
    for s in segs:
        ok, why = is_straight(s)
        if not ok:
            rejected.append((s.get("seq"), why))
            continue
        med = wheel_responses(s)
        if med is None:
            rejected.append((s.get("seq"), "no tick deltas"))
            continue
        t = s.get("_truth")
        if t and t.get("true_mm") is not None:
            out.append((med, float(t["true_mm"]), "tape", s.get("seq")))
            continue
        li = s.get("lidar") or {}
        a, b = li.get("front_mm_start"), li.get("front_mm_end")
        c = li.get("front_closure_mm")
        if a is None or b is None or c is None:
            # FALL BACK TO THE MQTT LIDAR PATH. Measured 2026-08-07:
            # /scan_lidar is advertised, the producer logs 9.5 Hz and 0 drops,
            # and NO ROS SUBSCRIBER RECEIVES A MESSAGE -- two independent
            # subscribers plus `ros2 topic echo` all sit silent on a topic with
            # a live publisher. The MQTT `front_mm` in telemetry/mission comes
            # through fpms-rover-agent instead and is the path that actually
            # works today, so it is accepted rather than throwing the only
            # encoder-independent reference away over a transport bug.
            a, b = li.get("front_mm_mqtt_start"), li.get("front_mm_mqtt_end")
            c = None if (a is None or b is None) else (a - b)
        if a is None or b is None or c is None:
            rejected.append((s.get("seq"), "no tape and no LiDAR wall pair"))
            continue
        if not (LIDAR_MIN_MM <= a <= LIDAR_MAX_MM
                and LIDAR_MIN_MM <= b <= LIDAR_MAX_MM):
            rejected.append((s.get("seq"), "LiDAR pair out of usable range"))
            continue
        if abs(c) < MIN_TRAVEL_MM:
            rejected.append((s.get("seq"), "wall closure below MIN_TRAVEL_MM"))
            continue
        out.append((med, float(c), "lidar", s.get("seq")))
    return out, rejected


# ============================================================== ESTIMATORS
def fit_counts_per_mm(segs):
    """counts_per_mm: OLS of signed encoder counts on an INDEPENDENT length.

    Two fits are reported. The free-intercept fit is the diagnostic -- a
    significant intercept means the reference is biased and the answer is not
    to be trusted. The through-origin fit is the estimate, because zero
    millimetres is zero counts by construction.

    The slope's SIGN is kept and reported: this firmware drives all four
    wheels backward on positive duty until MOTORn_INV is reflashed, so
    encoder-vs-world polarity is a live question and throwing the sign away
    would hide the answer."""
    res = {"parameter": "counts_per_mm",
           "estimator": "OLS through the origin (counts ~ k * mm), with a "
                        "free-intercept fit as a bias diagnostic",
           "candidates": CPM_CANDIDATES}
    samples, rejected = distance_samples(segs)
    res["rejected_segments"] = len(rejected)
    res["rejection_reasons"] = _top_reasons(rejected)
    by_src = defaultdict(list)
    for tk, mm, src, seq in samples:
        by_src[src].append((tk, mm, seq))
    res["n_by_source"] = {k: len(v) for k, v in by_src.items()}

    usable = [(tk, mm, src) for tk, mm, src, _ in samples]
    res["n"] = len(usable)
    if not usable:
        res["status"] = "REFUSED"
        res["reason"] = ("no segment carries an encoder-independent length "
                         "reference. Every straight run needs either a `truth` "
                         "record with true_mm, or a valid LiDAR front-wall "
                         "pair (drive straight at a flat wall).")
        res["need"] = f"{MIN_N_CPM} referenced straight segments"
        return res

    xs = [mm for _, mm, _ in usable]
    ys = [tk["median4"] for tk, _, _ in usable]
    lever = (max(abs(x) for x in xs) / max(min(abs(x) for x in xs), 1e-6))
    res["lever_ratio"] = round(lever, 2)

    # ALL THREE COMBINING RULES, because the constant is only valid for the
    # one it was fitted on -- see wheel_responses(). median4 is the headline
    # because that is what fpms_drive.dist_mm measures with, so it is the
    # number that can be pasted straight into the code that exists.
    variants = {}
    for key in ("median4", "front2", "mean4"):
        vy = [tk.get(key) for tk, _, _ in usable]
        if any(v is None for v in vy):
            continue
        f = ols_origin(xs, vy)
        if f:
            variants[key] = {"counts_per_mm": round(abs(f["slope"]), 4),
                             "ci95": sorted(round(abs(v), 4)
                                            for v in f["ci95_slope"])}
    res["by_combining_rule"] = variants
    res["combining_rule"] = "median4 (matches fpms_drive.dist_mm)"
    if "median4" in variants and "front2" in variants:
        a = variants["median4"]["counts_per_mm"]
        b = variants["front2"]["counts_per_mm"]
        res["combining_rule_spread_pct"] = round(abs(a - b) / max(b, 1e-9)
                                                 * 100.0, 2)
        res["combining_rule_warning"] = (
            f"median4 and front2 disagree by {res['combining_rule_spread_pct']}%. "
            "That is not measurement error -- they are different estimators of "
            "different quantities. Use the one your consuming code combines "
            "wheels with, or the distance will be wrong by that much.")

    free = ols(xs, ys)
    org = ols_origin(xs, ys)
    if free:
        res["free_intercept_fit"] = {
            "counts_per_mm": round(free["slope"], 4),
            "ci95": [round(v, 4) for v in free["ci95_slope"]],
            "intercept_counts": round(free["intercept"], 1),
            "intercept_ci95": [round(v, 1) for v in free["ci95_intercept"]],
            "r2": round(free["r2"], 5) if free["r2"] is not None else None,
            "resid_sd_counts": round(free["resid_sd"], 1)}
        # A CI on the intercept that excludes zero means the reference is
        # offset, and the through-origin slope would absorb that offset as
        # scale error. Worth a loud note, not a silent correction.
        lo, hi = free["ci95_intercept"]
        res["intercept_significant"] = bool(lo > 0 or hi < 0)

    # ------- gates -------
    fail = []
    if res["n"] < MIN_N_CPM:
        fail.append(f"n={res['n']} < hard floor {MIN_N_CPM}")
    if lever < MIN_LEVER_CPM:
        fail.append(f"lever ratio {lever:.2f} < {MIN_LEVER_CPM} -- every run "
                    "was about the same length, which pins the slope badly. "
                    "Drive a short leg and a long leg, not ten identical ones.")
    if len(usable) and len(set(round(x, 0) for x in xs)) < 3:
        fail.append("fewer than 3 distinct reference distances")
    if org is None:
        fail.append("degenerate design")

    if org is not None:
        k = org["slope"]
        half = (org["ci95_slope"][1] - org["ci95_slope"][0]) / 2.0
        rel = abs(half / k) if k else None
        res["estimate"] = round(abs(k), 4)
        res["polarity"] = "+1 (counts increase forward)" if k > 0 else \
                          "-1 (counts DECREASE on forward travel)"
        res["ci95"] = sorted(round(abs(v), 4) for v in org["ci95_slope"])
        res["se"] = round(org["se_slope"], 5)
        res["ci95_rel"] = round(rel, 4) if rel else None
        res["dof"] = org["dof"]

        # RESOLUTION GATE. The gap this number exists to close is the gap
        # between the two serious candidates, 5.5 and 6.0. To choose between
        # them the interval must be narrower than half that gap.
        near = sorted(CPM_CANDIDATES.values(),
                      key=lambda c: abs(c - abs(k)))[:2]
        gap = abs(near[0] - near[1]) if len(near) > 1 else 0.5
        target_rel = (gap / 2.0) / max(abs(k), 1e-9)
        res["resolution_target"] = {
            "must_separate": near,
            "required_ci95_halfwidth": round(gap / 2.0, 4),
            "required_rel": round(target_rel, 4)}
        # Relative to the mean MAGNITUDE, not the mean. Reverse segments carry
        # negative counts and a signed mean over a balanced forward/reverse
        # design is near zero, which would make this ratio explode and demand
        # tens of thousands of samples. Caught by the synthetic dataset.
        rel_sd = org["resid_sd"] / max(statistics.fmean([abs(y) for y in ys]),
                                       1e-9)
        need_n = required_n(rel_sd, target_rel, MIN_N_CPM)
        res["n_for_resolution"] = need_n
        contains = [name for name, c in CPM_CANDIDATES.items()
                    if res["ci95"][0] <= c <= res["ci95"][1]]
        res["candidates_inside_ci"] = contains
        if len(contains) > 1:
            fail.append(f"the 95% CI still contains {len(contains)} of the "
                        f"published candidates ({', '.join(contains)}) -- it "
                        "does not settle the dispute it was run to settle")
        if need_n and res["n"] < need_n:
            fail.append(f"observed scatter needs n>={need_n} for a CI narrow "
                        f"enough to choose; have {res['n']}")

    if by_src.get("lidar") and not by_src.get("tape"):
        res["caveat"] = (
            "LiDAR-only. The reference then carries error of its own, and "
            "errors-in-variables biases an OLS slope TOWARD ZERO -- so this "
            "counts/mm is an UNDER-estimate of the truth by an unknown amount. "
            "One taped run of a known length converts this from indicative to "
            "traceable; two makes it checkable.")

    if fail:
        res["status"] = "REFUSED"
        res["reason"] = "; ".join(fail)
        res["need"] = _need_line(res.get("n", 0),
                                 max(MIN_N_CPM, res.get("n_for_resolution")
                                     or MIN_N_CPM),
                                 "referenced straight segments")
        # The point estimate is kept in `estimate` but the status is what a
        # consumer must read. Nothing downstream may adopt a REFUSED value.
    else:
        res["status"] = "ESTIMATED"
    return res


def fit_effective_track(segs):
    """effective_track_mm from dyaw = (d_R - d_L) / (counts_per_mm * track).

    Regression of the tick differential (counts) on the measured yaw change
    (radians), through the origin: slope = counts_per_mm * track, so the
    track needs counts_per_mm to have been settled first. Both dependencies
    are reported, because a track quoted without the scale it was divided by
    is not a number anyone can reuse.

    Requires a yaw reference that is not the wheels. On this build there
    isn't one -- see the module docstring."""
    res = {"parameter": "effective_track_mm",
           "estimator": "OLS through the origin: tick_diff_counts ~ "
                        "(counts_per_mm * track) * dyaw_rad",
           "candidates": TRACK_CANDIDATES}
    turns, gyro_live, gyro_seen = [], 0, 0
    for s in segs:
        g = s.get("gyro") or {}
        gyro_seen += int(g.get("samples") or 0)
        gyro_live += int(g.get("nonzero_samples") or 0)
        t = s.get("_truth") or {}
        dyaw_deg = None
        source = None
        if t.get("true_deg") is not None:
            dyaw_deg, source = float(t["true_deg"]), "operator"
        elif g.get("alive") and g.get("yaw_deg") is not None:
            dyaw_deg, source = float(g["yaw_deg"]), "gyro"
        if dyaw_deg is None or abs(dyaw_deg) < MIN_TURN_DEG:
            continue
        d = (s.get("measured") or {}).get("tick_diff_front_counts")
        if d is None:
            continue
        turns.append((math.radians(dyaw_deg), float(d), source))

    res["gyro_samples_total"] = gyro_seen
    res["gyro_samples_nonzero"] = gyro_live
    res["n"] = len(turns)
    if gyro_seen and gyro_live == 0:
        res["gyro_verdict"] = (
            f"all {gyro_seen} /imu samples in this dataset are exactly 0.0 -- "
            "the gyro deadband fault is still present")

    if len(turns) < MIN_N_TRACK:
        res["status"] = "REFUSED"
        res["reason"] = (
            "no independent yaw reference. /imu angular_velocity is "
            "identically 0.0 on this firmware, so every turn in this dataset "
            "measures 0 degrees; /odom yaw is integrated on the board from "
            "these same wheels with the board's own track, which is circular. "
            "The effective track, TURN_WIRE_SIGN and TURN_RADPS are all "
            "UNIDENTIFIABLE until the gyro deadband is fixed.")
        res["need"] = (
            f"{MIN_N_TRACK} turns of >= {MIN_TURN_DEG:.0f} deg "
            f"(>= {MIN_PER_DIR_TRACK} each direction) with a live gyro -- OR "
            "the same number of hand-marked turns appended as `truth` records "
            "carrying true_deg")
        res["unblocked_by"] = "firmware gyro deadband fix"
        return res

    xs = [d for d, _, _ in turns]
    ys = [c for _, c, _ in turns]
    pos = sum(1 for d in xs if d > 0)
    neg = len(xs) - pos
    org = ols_origin(xs, ys)
    fail = []
    if pos < MIN_PER_DIR_TRACK or neg < MIN_PER_DIR_TRACK:
        fail.append(f"turns are one-sided ({pos} CCW / {neg} CW); "
                    f"need >= {MIN_PER_DIR_TRACK} each way to separate the "
                    "track from a directional wiring asymmetry")
    if org is None:
        fail.append("degenerate design")
    else:
        res["slope_counts_per_rad"] = round(org["slope"], 2)
        res["ci95_slope"] = [round(v, 2) for v in org["ci95_slope"]]
        res["note"] = ("divide by a SETTLED counts_per_mm to get the track in "
                       "mm; it is deliberately not divided here by a disputed "
                       "constant")
    res["status"] = "REFUSED" if fail else "ESTIMATED"
    if fail:
        res["reason"] = "; ".join(fail)
    return res


def fit_wheel_slip(segs):
    """Per-wheel slip factor s_i = d_i / signed_median(d), by direction.

    Ratios rather than a regression, because the quantity is dimensionless
    and the reference (the median) is internal -- no external length is
    needed, which is the one thing here that CAN be measured today without
    the gyro or a tape. Forward and reverse are estimated separately: a wheel
    that reads 1.24 forward and 0.81 in reverse is not slipping, it is wired
    or geared asymmetrically, and pooling the two would report a clean 1.02
    and hide a real fault."""
    res = {"parameter": "wheel_slip",
           "estimator": "per-wheel ratio to the signed median of four, "
                        "t-interval on the per-segment ratios, split by "
                        "direction of travel",
           "wheel_index": {"0": "front_left", "1": "front_right",
                           "2": "rear_left", "3": "rear_right"}}
    fwd = defaultdict(list)
    rev = defaultdict(list)
    n_f = n_r = 0
    for s in segs:
        m = s.get("measured") or {}
        pw, med = m.get("ticks_per_wheel"), m.get("ticks_median")
        if not pw or len(pw) != 4 or not med or abs(med) < MIN_TRAVEL_MM:
            continue
        bucket = fwd if med > 0 else rev
        if med > 0:
            n_f += 1
        else:
            n_r += 1
        for i in range(4):
            bucket[i].append(pw[i] / med)
    res["n_forward"], res["n_reverse"] = n_f, n_r
    res["n"] = n_f + n_r

    out = {}
    for i in range(4):
        entry = {}
        for name, b, cnt in (("forward", fwd, n_f), ("reverse", rev, n_r)):
            ci = mean_ci(b[i]) if len(b[i]) >= 2 else None
            if ci:
                entry[name] = {"n": ci["n"], "factor": round(ci["mean"], 4),
                               "ci95": [round(v, 4) for v in ci["ci95"]],
                               "sd": round(ci["sd"], 4)}
        # A wheel whose forward and reverse intervals do not overlap is
        # asymmetric, which is a mechanical or wiring finding, not slip.
        f, r = entry.get("forward"), entry.get("reverse")
        if f and r:
            entry["direction_asymmetric"] = bool(
                f["ci95"][1] < r["ci95"][0] or r["ci95"][1] < f["ci95"][0])
        out[str(i)] = entry
    res["per_wheel"] = out

    fail = []
    if res["n"] < MIN_N_SLIP:
        fail.append(f"n={res['n']} < {MIN_N_SLIP}")
    if n_f < MIN_PER_DIR_SLIP or n_r < MIN_PER_DIR_SLIP:
        fail.append(f"need >= {MIN_PER_DIR_SLIP} forward AND "
                    f"{MIN_PER_DIR_SLIP} reverse segments to separate slip "
                    f"from a directional asymmetry (have {n_f}/{n_r})")
    if fail:
        res["status"] = "REFUSED"
        res["reason"] = "; ".join(fail)
        res["need"] = (f"{MIN_N_SLIP} segments of >= {MIN_TRAVEL_MM:.0f} "
                       f"counts, at least {MIN_PER_DIR_SLIP} in each direction")
    else:
        res["status"] = "ESTIMATED"
        flagged = [k for k, v in out.items() if v.get("direction_asymmetric")]
        if flagged:
            res["warning"] = (
                f"wheels {flagged} read differently forward vs reverse. That "
                "is not slip. Check the wiring polarity and the encoder sign "
                "for those wheels before trusting any distance from them -- "
                "a mismatched encoder/motor sign makes PID firmware run away "
                "at full duty until reset.")
    return res


def fit_duty_velocity(segs):
    """v = a * (duty - u0), and u0 is the stall (breakaway) threshold.

    TWO STAGES, because a single-stage fit of distance/pulse against duty
    would be corrupted by coast. Every burst on this chassis is
    command -> cut -> COAST -> stop, and the recorder deliberately measures
    after the coast, so the distance for a burst is

        distance = v * t_commanded + coast

    Stage 1 -- for each duty level with enough repeats at DIFFERENT commanded
    durations, regress distance on duration. The SLOPE is the steady-state
    velocity at that duty and the INTERCEPT is the coast, which is separated
    out instead of contaminating the answer. Both are reported: coast is a
    number this rover has only ever guessed at (40 mm 'conservative',
    86 mm measured once at duty 70).

    Stage 2 -- regress those per-duty velocities on duty. The slope is the
    gain and the x-intercept -b/a is the stall threshold, with a delta-method
    interval and a Fieller guard.

    The stall estimate is then CHECKED against direct observation: the
    highest duty that produced no motion must be below it, and the lowest
    duty that produced motion above it. An extrapolated intercept that
    contradicts a burst somebody actually watched is reported as an
    inconsistency, not averaged in.

    Velocities are in COUNTS PER SECOND, not mm/s. Converting would require
    counts_per_mm, and this estimator refuses to inherit that dispute -- the
    caller multiplies once counts_per_mm is settled."""
    res = {"parameter": "duty_velocity_curve",
           "estimator": "two-stage: (1) distance ~ duration per duty level, "
                        "separating coast as the intercept; (2) velocity ~ "
                        "duty, stall = x-intercept by the delta method",
           "units": "counts/s per duty step; duty is the raw 0-255 wire value"}
    by_duty = defaultdict(list)
    zero_motion, short_pulse = [], []
    for s in segs:
        c = s.get("commanded") or {}
        m = s.get("measured") or {}
        duty = c.get("duty_peak")
        t = c.get("pulse_s")
        med = m.get("ticks_median")
        if duty is None or not t or t <= 0 or med is None:
            continue
        if abs(med) < 3:                      # under quantisation = no motion
            row = {"duty": duty, "pulse_s": round(t, 3), "seq": s.get("seq")}
            # THE TWO THRESHOLDS ARE CONFOUNDED AND MUST BE SEPARATED. A burst
            # that moved nothing did so for one of two completely different
            # reasons: the duty was below breakaway (evidence about the STALL),
            # or the pulse ended before the velocity loop got going (evidence
            # about MIN_PULSE_S, and evidence about nothing else). Feeding a
            # too-short burst at a healthy duty into the stall estimator makes
            # it report that a duty which demonstrably drives this rover is
            # below its own breakaway -- which is what the synthetic dataset
            # caught this doing. Only a burst long enough to have moved if it
            # could is admissible as stall evidence.
            (zero_motion if t >= MIN_STALL_EVIDENCE_PULSE_S
             else short_pulse).append(row)
            continue
        by_duty[int(duty)].append((float(t), abs(float(med))))

    res["short_pulse_no_motion_excluded"] = len(short_pulse)
    res["short_pulse_note"] = (
        "bursts shorter than "
        f"{MIN_STALL_EVIDENCE_PULSE_S}s that moved nothing are excluded from "
        "the stall evidence -- they are MIN_PULSE_S evidence, not duty "
        "evidence, and are counted by the min_pulse_s estimator instead")
    res["duty_levels_seen"] = sorted(by_duty)
    res["n_levels"] = len(by_duty)
    res["n_total"] = sum(len(v) for v in by_duty.values())
    res["zero_motion_observations"] = len(zero_motion)
    res["zero_motion_max_duty"] = (max(z["duty"] for z in zero_motion)
                                   if zero_motion else None)
    res["motion_min_duty"] = min(by_duty) if by_duty else None

    stage1, crude = {}, {}
    for duty, obs in sorted(by_duty.items()):
        durs = set(round(t, 2) for t, _ in obs)
        if len(obs) >= 3 and len(durs) >= 2:
            f = ols([t for t, _ in obs], [d for _, d in obs])
            if f:
                stage1[duty] = {
                    "velocity_counts_per_s": round(f["slope"], 1),
                    "ci95": [round(v, 1) for v in f["ci95_slope"]],
                    "coast_counts": round(f["intercept"], 1),
                    "coast_ci95": [round(v, 1) for v in f["ci95_intercept"]],
                    "n": f["n"], "r2": round(f["r2"], 4)
                    if f["r2"] is not None else None}
                continue
        # Not enough shape at this duty to separate coast. Kept, clearly
        # labelled, because it still shows WHERE the data is thin.
        crude[duty] = {"n": len(obs),
                       "mean_counts_per_s": round(
                           statistics.fmean(d / t for t, d in obs), 1),
                       "note": "coast NOT separated -- biased HIGH; needs >=2 "
                               "distinct pulse durations at this duty"}
    res["per_duty"] = stage1
    res["per_duty_unresolved"] = crude

    fail = []
    if len(stage1) < MIN_DUTY_LEVELS:
        fail.append(f"only {len(stage1)} duty levels have enough shape to "
                    f"separate coast; need {MIN_DUTY_LEVELS}")
    thin = [d for d, v in by_duty.items() if len(v) < MIN_REPS_PER_DUTY]
    if thin:
        fail.append(f"duty levels {sorted(thin)} have < {MIN_REPS_PER_DUTY} "
                    "repeats")
    if len(zero_motion) < MIN_ZERO_MOTION_OBS:
        fail.append(f"only {len(zero_motion)} observations of a non-zero duty "
                    f"that produced NO motion; need {MIN_ZERO_MOTION_OBS}. "
                    "Without them the stall threshold is an extrapolation off "
                    "the end of the data, not a measurement.")

    if len(stage1) >= 3:
        xs = sorted(stage1)
        ys = [stage1[d]["velocity_counts_per_s"] for d in xs]
        f2 = ols([float(x) for x in xs], ys)
        if f2:
            res["gain_counts_per_s_per_duty"] = round(f2["slope"], 2)
            res["gain_ci95"] = [round(v, 2) for v in f2["ci95_slope"]]
            res["curve_r2"] = round(f2["r2"], 4) if f2["r2"] is not None else None
            st = ratio_ci(-f2["intercept"], f2["slope"],
                          f2["se_intercept"], f2["se_slope"], f2["dof"])
            if st:
                res["stall_duty"] = round(st["value"], 1)
                res["stall_ci95"] = [round(v, 1) for v in st["ci95"]]
                lo_obs = res["zero_motion_max_duty"]
                hi_obs = res["motion_min_duty"]
                bad = []
                if lo_obs is not None and st["ci95"][1] < lo_obs:
                    bad.append(f"a duty of {lo_obs} was observed to produce NO "
                               f"motion, above the fitted stall CI")
                if hi_obs is not None and st["ci95"][0] > hi_obs:
                    bad.append(f"a duty of {hi_obs} DID produce motion, below "
                               f"the fitted stall CI")
                if bad:
                    res["inconsistency"] = "; ".join(bad)
                    fail.append("the fitted stall contradicts a burst that was "
                                "actually observed: " + "; ".join(bad))
            else:
                fail.append("the duty->velocity slope is not distinguishable "
                            "from zero, so the stall x-intercept is unbounded; "
                            "no interval can honestly be printed")

    res["status"] = "REFUSED" if fail else "ESTIMATED"
    if fail:
        res["reason"] = "; ".join(fail)
        res["need"] = (f"{MIN_DUTY_LEVELS} duty levels x "
                       f"{MIN_REPS_PER_DUTY} repeats at >= 2 distinct pulse "
                       f"durations = {MIN_DUTY_LEVELS * MIN_REPS_PER_DUTY} "
                       f"segments, PLUS {MIN_ZERO_MOTION_OBS} bursts at a duty "
                       "below breakaway that move nothing")
    return res


def fit_min_pulse(segs):
    """MIN_PULSE_S under LOAD, as a bracket rather than a point.

    The config value 0.35 was measured WHEELS OFF and the code says so
    (MIN_PULSE_MEASURED_UNDER_LOAD = False). Under load it can only be
    longer. This reports the largest commanded pulse that moved nothing and
    the smallest that moved -- a bracket is the honest form of this answer,
    because the transition is a probability, not an edge: the same 0.30 s
    burst moves on a warm battery and does not on a cold one."""
    res = {"parameter": "min_pulse_s_under_load",
           "estimator": "empirical bracket on commanded pulse duration vs "
                        "whether any wheel moved; deliberately not a point "
                        "estimate",
           "config_value": 0.35,
           "config_provenance": "measured WHEELS OFF; "
                                "MIN_PULSE_MEASURED_UNDER_LOAD = False"}
    # SAME CONFOUND AS THE STALL, FROM THE OTHER SIDE. A burst that moved
    # nothing at a duty below breakaway says nothing about the minimum pulse:
    # it would not have moved at any duration. So only duties that have been
    # SEEN TO MOVE THIS ROVER are admissible here, and the bracket is built
    # within those duties only.
    per_duty = defaultdict(lambda: {"moved": [], "still": []})
    for s in segs:
        c = s.get("commanded") or {}
        t = c.get("pulse_s")
        med = (s.get("measured") or {}).get("ticks_median")
        duty = c.get("duty_peak")
        if not t or med is None or not duty:
            continue
        key = "moved" if abs(med) >= 3 else "still"
        per_duty[int(duty)][key].append(round(float(t), 3))

    live = {d: v for d, v in per_duty.items() if v["moved"]}
    res["duties_known_to_move"] = sorted(live)
    res["duties_excluded_never_moved"] = sorted(
        d for d, v in per_duty.items() if not v["moved"])
    moved = [t for v in live.values() for t in v["moved"]]
    still = [t for v in live.values() for t in v["still"]]
    res["n_moved"], res["n_still"] = len(moved), len(still)
    res["n"] = len(moved) + len(still)
    res["distinct_durations"] = len(set(moved) | set(still))
    if moved:
        res["shortest_pulse_that_moved_s"] = min(moved)
    if still:
        res["longest_pulse_that_did_not_move_s"] = max(still)

    fail = []
    if res["distinct_durations"] < MIN_PULSE_DURATIONS:
        fail.append(f"only {res['distinct_durations']} distinct pulse "
                    f"durations at duties known to move")
    if len(moved) < MIN_PULSE_REPS or len(still) < MIN_PULSE_REPS:
        fail.append(f"{len(moved)} bursts moved and {len(still)} did not; "
                    f"need >= {MIN_PULSE_REPS} of each")
    if moved and still:
        lo, hi = max(still), min(moved)
        res["bracket_s"] = [lo, hi]
        if lo >= hi:
            # A longer burst failed while a shorter one succeeded at a duty
            # that works. There is no single duration threshold in this data.
            fail.append(
                f"the bracket is INVERTED: a {lo}s burst moved nothing while a "
                f"{hi}s burst did, at duties known to work. Something other "
                "than duration decided those outcomes -- battery sag, a "
                "latched estop, or the duty being near breakaway where the "
                "outcome is a coin flip. No threshold can be read off this.")
    res["status"] = "REFUSED" if fail else "ESTIMATED"
    if fail:
        res["reason"] = "; ".join(fail)
        res["need"] = (f"{MIN_PULSE_DURATIONS} distinct commanded durations "
                       f"straddling the threshold, >= {MIN_PULSE_REPS} of "
                       "each outcome, at a duty WELL ABOVE breakaway, ON THE "
                       "FLOOR with the wheels loaded")
    return res


def odom_consistency(segs):
    """NOT an estimate. The circular reference, computed on purpose.

    Regressing ticks on the board's own odom recovers the constant the
    FIRMWARE is using, which is worth knowing for exactly one reason: if it
    does not come out at the firmware's configured value, something in the
    odom pipeline is broken on top of the scale dispute -- and that would
    otherwise present as a mysterious extra error nobody could source."""
    xs, ys = [], []
    for s in segs:
        ok, _ = is_straight(s)
        if not ok:
            continue
        d = ((s.get("odom") or {}).get("delta") or {}).get("dist_mm")
        med = (s.get("measured") or {}).get("ticks_median")
        if d is None or med is None or abs(d) < MIN_TRAVEL_MM:
            continue
        xs.append(abs(float(d)))
        ys.append(abs(float(med)))
    out = {"check": "ticks vs board /odom (CIRCULAR -- never an estimate)",
           "n": len(xs)}
    if len(xs) >= 4:
        f = ols_origin(xs, ys)
        if f:
            out["firmware_implied_counts_per_mm"] = round(f["slope"], 4)
            out["ci95"] = [round(v, 4) for v in f["ci95_slope"]]
            out["interpretation"] = (
                "this is what the BOARD believes, not what is true. If it "
                "differs from the flashed COUNTS_PER_REV / (pi*wheel_diam), "
                "the odom pipeline has a second fault.")
    else:
        out["interpretation"] = "too few straight segments to compute"
    return out


# ================================================================= REPORT
def _top_reasons(rejected, k=5):
    c = defaultdict(int)
    for _, why in rejected:
        c[str(why)] += 1
    return dict(sorted(c.items(), key=lambda kv: -kv[1])[:k])


def _need_line(have, need, unit):
    short = max(0, need - have)
    if short == 0:
        return f"design gate, not sample count -- vary the inputs, not n"
    return f"{short} more {unit} (have {have}, need {need})"


def grid_summary(recs):
    """What the dataset knows about the ARENA, as opposed to the drivetrain.

    Deliberately NOT fitted. There is no constant to estimate here -- the
    learned grid prior is a decayed hit/miss count maintained live by
    fpms_grid_learn, and this is only a readout so that one command answers
    "is the grid half working" as well as "is the drive half working"."""
    segs = [r for r in recs if r.get("rec") == "segment" and r.get("grid")]
    replans = [r for r in recs if r.get("rec") == "replan"]
    plans = [r for r in recs if r.get("rec") == "plan"]
    hits = set()
    agree_delta = []
    posed = 0
    for s in segs:
        g = s["grid"]
        for c in g.get("hit_cells") or []:
            hits.add(tuple(c))
        a = g.get("agreement") or {}
        if a.get("delta") is not None:
            agree_delta.append(int(a["delta"]))
        if ((s.get("route") or {}).get("driven")):
            posed += 1
    out = {"segments_with_grid": len(segs),
           "distinct_cells_ever_hit": len(hits),
           "replan_events": len(replans),
           "plans_seen": len(plans),
           "segments_with_planned_and_driven": posed,
           "mirror_vs_executor_cell_delta": None}
    if agree_delta:
        out["mirror_vs_executor_cell_delta"] = {
            "n": len(agree_delta),
            "median": statistics.median(agree_delta),
            "max_abs": max(abs(d) for d in agree_delta)}
    return out


def dataset_summary(files, recs, segs, bad, hb):
    n_turn = sum(1 for s in segs
                 if (s.get("context") or {}).get("segment_kind") == "turn")
    gyro_n = sum(int((s.get("gyro") or {}).get("samples") or 0) for s in segs)
    gyro_nz = sum(int((s.get("gyro") or {}).get("nonzero_samples") or 0)
                  for s in segs)
    tapes = sum(1 for s in segs if s.get("_truth"))
    lidar = sum(1 for s in segs
                if (s.get("lidar") or {}).get("front_closure_mm") is not None)
    joined = sum(1 for s in segs if s.get("_join"))
    return {"files": len(files), "records": len(recs),
            "unparseable_lines": bad, "segments": len(segs),
            "turn_segments": n_turn, "drive_segments": len(segs) - n_turn,
            "segments_with_tape_truth": tapes,
            "segments_with_lidar_wall_pair": lidar,
            "segments_joined_to_executor_residual": joined,
            "imu_samples": gyro_n, "imu_samples_nonzero": gyro_nz,
            "gyro_alive": bool(gyro_nz),
            "heartbeats": len(hb)}


# The open arguments this fitter exists to close, each with the value the
# DEPLOYED CODE is using right now. A report that lists estimates without
# saying what is currently in the rover leaves the reader to go and look, and
# the whole point of the refusal rule is that the next action must be obvious.
DISPUTES = [
    {"name": "counts_per_mm",
     "parameter": "counts_per_mm",
     "in_use": "6.00 counts/mm (0.16657 mm/tick in the deployed drive path)",
     "competing": "5.5 (hand push, 1000 mm tape, motors off) / "
                  "6.0 (derived from TICKS_PER_REV 1320) / "
                  "14.8 (tape 2026-08-04, unconfirmed)",
     "standing": "The value in use was validated to 2.5% against odometry on "
                 "the successful M2 run (798.5 ticks x 0.16657 = 133.0 mm vs "
                 "odom 129.7 mm) -- but odometry is the board integrating "
                 "THESE SAME ENCODERS, so that check can only catch a gross "
                 "error, never confirm the scale. 14.8 is 2.5x away and would "
                 "have made that run read 2.2 m; it is very hard to reconcile "
                 "with any observed run and is the weakest of the three. "
                 "Settling 5.5 vs 6.0 needs a reference from outside the "
                 "drivetrain.",
     "needs": "a tape measure (`truth` records) or a straight run at a flat "
              "wall with the LiDAR. A LiDAR-only answer is a biased "
              "UNDER-estimate -- errors in the reference pull an OLS slope "
              "toward zero -- so it can support 5.5-over-6.0 only weakly and "
              "must never be the last word."},
    {"name": "effective_track_mm",
     "parameter": "effective_track_mm",
     "in_use": "three live values and none of them measured: 105 mm "
               "(fpms_rtos_follower.py), 170 mm (fpms_odom_tf.py, geometric), "
               "255 mm (fpms_drive.py, effective); 271 mm in older notes",
     "competing": "105 / 170 / 255 / 271",
     "standing": "Only turning exercises it, and the M2 run was straight "
                 "ahead, so nothing measured so far touches it. Fitting it "
                 "needs a dyaw from something that is not the wheels.",
     "needs": "a working gyro, or twelve hand-turned `truth` records carrying "
              "`true_deg`. Note the project rule: never turn-test this rover "
              "for liveness -- these must be deliberate, measured, "
              "operator-supervised turns."},
    {"name": "gyro_scale",
     "parameter": "effective_track_mm",
     "in_use": "unmeasured; TURN_WIRE_SIGN is DERIVED from the 2026-08-01 "
               "rewiring, not measured, and a wrong value has already cost a "
               "session",
     "competing": "n/a -- there is no competing value, there is no value",
     "standing": "A gyro scale cannot be fitted from a dataset in which the "
                 "gyro reads 0.0. Note the trap in the other direction: a "
                 "HEALTHY gyro on this build also reads exactly 0.0 when "
                 "parked, because of a +/-0.01 rad/s deadband -- a parked "
                 "reading proves nothing either way. What decides it is "
                 "whether any sample during a real rotation was non-zero, "
                 "which is what `imu_samples_nonzero` counts.",
     "needs": "one supervised rotation with the recorder running, then re-run "
              "this fitter and read `gyro_alive`."},
]


def disputes(out):
    """Pair each live dispute with what this dataset can say about it."""
    rows = []
    for d in DISPUTES:
        p = out["parameters"].get(d["parameter"]) or {}
        st = p.get("status")
        if st == "ESTIMATED":
            verdict = "ESTIMATED -- see the parameter block above"
        elif st:
            verdict = f"CANNOT DETERMINE ({p.get('reason') or 'refused'})"
        else:
            verdict = "CANNOT DETERMINE (no estimator ran)"
        rows.append(dict(d, verdict=verdict,
                         estimate=p.get("estimate"), ci95=p.get("ci95"),
                         need=p.get("need"),
                         blocked_on=p.get("unblocked_by")))
    return rows


def render(out, verbose=False):
    L = []
    a = L.append
    a("=" * 74)
    a("FPMS CALIBRATION FIT -- parameter estimation, not machine learning")
    a("=" * 74)
    d = out["dataset"]
    a(f"dataset: {d['segments']} segments from {d['files']} file(s) "
      f"({d['drive_segments']} drive / {d['turn_segments']} turn)")
    a(f"         {d['segments_with_tape_truth']} with a tape truth, "
      f"{d['segments_with_lidar_wall_pair']} with a LiDAR wall pair, "
      f"{d['segments_joined_to_executor_residual']} joined to the executor")
    if d["imu_samples"] and not d["imu_samples_nonzero"]:
        a(f"         GYRO DEAD: {d['imu_samples']} /imu samples, all exactly "
          "0.0 -- every turn here measures 0 degrees")
    if d["unparseable_lines"]:
        a(f"         {d['unparseable_lines']} unparseable line(s) skipped "
          "(normal after a power cut)")
    a("")
    for key in ("counts_per_mm", "effective_track_mm", "wheel_slip",
                "duty_velocity_curve", "min_pulse_s_under_load"):
        r = out["parameters"].get(key)
        if not r:
            continue
        mark = "OK " if r.get("status") == "ESTIMATED" else "-- "
        a(f"{mark}{key}   [{r.get('status')}]  n={r.get('n', r.get('n_total', 0))}")
        if r.get("estimate") is not None:
            a(f"      estimate {r['estimate']}   95% CI {r.get('ci95')}")
        if r.get("stall_duty") is not None:
            a(f"      stall duty {r['stall_duty']}  95% CI {r.get('stall_ci95')}")
        if r.get("gain_counts_per_s_per_duty") is not None:
            a(f"      gain {r['gain_counts_per_s_per_duty']} counts/s per duty "
              f"step  95% CI {r.get('gain_ci95')}")
        if r.get("bracket_s"):
            a(f"      bracket {r['bracket_s'][0]}s (no motion) .. "
              f"{r['bracket_s'][1]}s (motion)")
        if r.get("status") != "ESTIMATED":
            a(f"      why:  {r.get('reason')}")
            if r.get("need"):
                a(f"      need: {r['need']}")
            if r.get("unblocked_by"):
                a(f"      blocked on: {r['unblocked_by']}")
        if r.get("by_combining_rule"):
            for k, v in r["by_combining_rule"].items():
                a("      via {:<8} {:<8} 95% CI {}".format(
                    k, v["counts_per_mm"], v["ci95"]))
        if r.get("combining_rule_warning"):
            a(f"      NOTE: {r['combining_rule_warning']}")
        if r.get("caveat"):
            a(f"      caveat: {r['caveat']}")
        if r.get("warning"):
            a(f"      WARNING: {r['warning']}")
        if r.get("candidates_inside_ci"):
            a(f"      CI still contains: "
              f"{', '.join(r['candidates_inside_ci'])}")
        a("")
    if verbose:
        a("-- per-wheel slip detail " + "-" * 48)
        ws = out["parameters"].get("wheel_slip", {}).get("per_wheel", {})
        names = {"0": "front_left", "1": "front_right",
                 "2": "rear_left", "3": "rear_right"}
        for i in sorted(ws):
            e = ws[i]
            f, rv = e.get("forward"), e.get("reverse")
            a(f"  wheel {i} {names[i]:<12} "
              f"fwd {f['factor'] if f else '-':<8} "
              f"rev {rv['factor'] if rv else '-':<8} "
              f"{'ASYMMETRIC' if e.get('direction_asymmetric') else ''}")
        a("")
    a("-- consistency check " + "-" * 52)
    oc = out["odom_consistency"]
    a(f"  {oc['check']}")
    if oc.get("firmware_implied_counts_per_mm") is not None:
        a(f"  the board behaves as if counts/mm = "
          f"{oc['firmware_implied_counts_per_mm']} (n={oc['n']})")
    a(f"  {oc.get('interpretation', '')}")
    a("")
    est = [k for k, v in out["parameters"].items()
           if v.get("status") == "ESTIMATED"]
    ref = [k for k, v in out["parameters"].items()
           if v.get("status") != "ESTIMATED"]
    a(f"ESTIMATED: {', '.join(est) if est else 'nothing'}")
    a(f"REFUSED:   {', '.join(ref) if ref else 'nothing'}")
    a("")
    a("A refusal is a result. None of the REFUSED values above may be copied")
    a("into /etc/fpms/config.env or fpms_missions.py -- a confident number")
    a("from too few runs is worse than the honest dispute it replaced.")
    a("")
    a("-- the live disputes, and what this dataset says about them " + "-" * 14)
    for d in out.get("disputes") or []:
        a(f"  {d['name']}")
        a(f"      in use now : {d['in_use']}")
        a(f"      competing  : {d['competing']}")
        a(f"      VERDICT    : {d['verdict']}")
        if d.get("estimate") is not None:
            a(f"      estimate   : {d['estimate']}  95% CI {d.get('ci95')}")
        if d.get("need"):
            a(f"      need       : {d['need']}")
        if d.get("blocked_on"):
            a(f"      blocked on : {d['blocked_on']}")
        a(f"      note       : {d['standing']}")
        a(f"      to settle  : {d['needs']}")
        a("")
    a("-- residual source " + "-" * 54)
    j = out.get("joins") or {}
    a(f"  from events/mission_done -> measured[] : "
      f"{j.get('joins_from_mission_done', 0)} segment(s)")
    a(f"  from telemetry/residual               : "
      f"{j.get('joins_from_telemetry_residual', 0)} segment(s)")
    a("  telemetry/residual is documented, advertised on six ROS topics and")
    a("  subscribed by two files, and NOTHING PUBLISHES IT -- the deployed")
    a("  fpms_missions.py has no _publish_residual. events/mission_done is the")
    a("  working source and is what the number above comes from.")
    if j.get("runs_with_unmatched_episode_counts"):
        a(f"  {j['runs_with_unmatched_episode_counts']} run(s) had an episode "
          "count that did not match the executor's segment count; nothing was")
        a("  paired for those, on purpose -- a positional join would be a guess.")
    a("")
    g = out.get("grid") or {}
    a("-- grid " + "-" * 65)
    a(f"  {g.get('segments_with_grid', 0)} episode(s) carry a grid snapshot, "
      f"{g.get('distinct_cells_ever_hit', 0)} distinct cell(s) ever believed")
    a(f"  {g.get('replan_events', 0)} obstacle reroute event(s), "
      f"{g.get('plans_seen', 0)} planned route(s) seen, "
      f"{g.get('segments_with_planned_and_driven', 0)} planned-vs-driven pair(s)")
    md = g.get("mirror_vs_executor_cell_delta")
    if md:
        a(f"  mirror vs executor cell count: median {md['median']}, worst "
          f"|{md['max_abs']}| over n={md['n']}")
        a("  A large or growing delta means this recorder's mirror of the")
        a("  occupancy grid disagrees with the executor's own -- suspect the")
        a("  bearing convention or a stale pose, and distrust the learned")
        a("  grid prior until it is explained.")
    else:
        a("  no mirror-vs-executor comparison yet (needs telemetry/mission's")
        a("  `occupancy` alongside a folded scan in the same episode)")
    a("  The learned prior itself is NOT fitted here. Read it with:")
    a("      python3 fpms_grid_learn.py --show")
    return "\n".join(L)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or "-h" in argv or "--help" in argv:
        print(__doc__)
        return 0
    verbose = "--verbose" in argv
    out_json = None
    if "--json" in argv:
        i = argv.index("--json")
        out_json = argv[i + 1] if i + 1 < len(argv) else None
        del argv[i:i + 2]
    paths = [a for a in argv if not a.startswith("-")]
    if not paths:
        paths = [os.environ.get("FPMS_LEARN_DIR",
                                "/home/ubuntu/fpms_learn_data")]

    files, recs, bad = load(paths)
    if not recs:
        print(f"no records found in {paths}. Has fpms_learn_recorder.py run "
              "while the rover was driven?", file=sys.stderr)
        return 1
    segs, truths, joins, hb, jmeta = index(recs)

    out = {"tool": "fpms_learn_fit", "schema_min": SCHEMA_MIN,
           "sources": [os.path.basename(f) for f in files],
           "dataset": dataset_summary(files, recs, segs, bad, hb),
           "joins": jmeta,
           "grid": grid_summary(recs),
           "parameters": {}}
    for f in (fit_counts_per_mm, fit_effective_track, fit_wheel_slip,
              fit_duty_velocity, fit_min_pulse):
        r = f(segs)
        out["parameters"][r["parameter"]] = r
    out["odom_consistency"] = odom_consistency(segs)
    out["disputes"] = disputes(out)

    print(render(out, verbose=verbose))
    if out_json:
        with open(out_json, "w", encoding="utf-8") as fh:
            json.dump(out, fh, indent=2)
        print(f"\nwrote {out_json}")
    # Exit 0 always: a refusal is a valid, expected outcome and must not look
    # like a crashed tool to a script that runs this nightly.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
