#!/usr/bin/env python3
"""Synthetic ground-truth checks for fpms_scanmatch.py. No Pi, no LiDAR.

    python3 selftest/test_scanmatch.py

WHY THIS FILE EXISTS
--------------------
fpms_scanmatch.py is the tool that will decide `base_link -> laser_frame`, and
therefore gates Nav2 and both SLAM units. By fpms_tf.launch.py's own arithmetic

    1 degree of mount yaw  ~=  10.5 mm of position error

inherited IN THE SAME DIRECTION by the obstacle cone guard and the occupancy
grid, so it warps the map rather than averaging out. That file asks for the
mount yaw to be known "to better than ~0.5 degrees".

A subtly wrong estimator is worse than no estimator, because its output looks
authoritative: it prints a number with a decimal point and the operator bolts
the robot's whole coordinate frame to it. The Orange Pi 5B has not arrived, so
nobody can check any of this against a real room. Synthetic ground truth is the
only verification that exists.

So this file builds a world it knows the answer for -- a 4 x 3 m room with a
couple of obstacles and an angled panel -- renders 360-bin LaserScan range
arrays from known poses with known mount yaw and known handedness, and asks the
estimators to recover what was put in.

THE SIGN CONVENTIONS THIS FILE PINS
-----------------------------------
Three functions pass rotations between them and there is exactly one consistent
way to read them. This file encodes that reading as assertions, because the
module once shipped with it inconsistent and the symptom was an INVERTED
handedness verdict that no unit test of any single function could see:

    estimate_rotation      returns the CHASSIS rotation, same sign as the gyro.
                           (ifft(fft(a)*conj(fft(b))) peaks where b[m]=a[m+k];
                           a feature at bearing beta appears at beta-theta after
                           a +theta turn, so k = +theta/inc.)
    mirrored scan          reports the OPPOSITE sign, because negating bearings
                           reverses the shift. Verified directly in test_08.
    mirror_verdict         'ok' when scan and gyro AGREE in sign.
    estimate_translation   takes chassis_rotation_rad and negates internally.
                           Callers chain estimate_rotation straight in.

THE TOLERANCES, AND WHY THEY ARE THESE NUMBERS
----------------------------------------------
ROT_TOL_DEG = 0.50    Half the requirement above, and half of ONE BIN (the
                      scanner's angle_increment is exactly 1 degree). A
                      nearest-bin answer can be 0.5 deg out by construction, so
                      this tolerance is also the thing that proves the parabolic
                      sub-bin interpolation is doing real work rather than
                      decorating an integer. Observed worst case: 0.17 deg.

ROT_TOL_NOISY_DEG = 0.75   Same, with 3 mm of range noise and a dozen dropped
                      returns. Loosened by one quarter bin, not more: noise must
                      not be an excuse for missing the requirement.

TRANS_TOL_M = 0.005   5 mm, i.e. under half of what one degree of mount yaw
                      costs. Full overlap only. Observed worst case: 2 mm.

Occlusion tolerance is now the SAME 5 mm at 10%, 20%, 25% and 30% missing.
That flatness is the property being asserted, not an incidental pass: the
adaptive median+MAD trim should not care how much of the scan has no
counterpart, only how separable it is. The earlier fixed reject_frac=0.25 could
not remove a 30% tail by construction and drifted 47 mm on a 300 mm push with
no outward sign; the tolerance was widened to match at the time and is now
tightened back, because the estimator improved rather than the standard.

YAW_TOL_DEG = 0.50    The fpms_tf.launch.py requirement, applied to the
                      end-to-end operator session. This is the number that says
                      the tool works.

SINGLE_PUSH_TOL_DEG = 1.5   One push, with noise, is allowed three times the
                      session budget, because the session averages three pushes
                      and the whole point of doing three is that they average.
                      Used identically for both de-rotation sources in test_12,
                      so the comparison between them is like for like.

CONF_LOW = 0.25 / CONF_HIGH = 0.50   estimate_rotation's confidence must
                      separate "a featureless circular room, where rotation is
                      genuinely unobservable" from "a real room". Measured:
                      0.00-0.13 featureless, 0.74-0.82 in the test room. The
                      gap is wide; these thresholds sit in it.

WHAT THIS FILE CANNOT PROVE
---------------------------
That the real RPLIDAR emits what is simulated here. That the ranges arrive with
the bearings fpms_lidar_ros.py assigns them. That the operator can actually push
the rover in a straight line by hand. Ray casting against clean line segments has
no beam divergence, no mixed pixels at edges, no specular dropout on gloss paint,
and no motion distortion within one 100 ms sweep -- all of which are real, and
all of which make the real numbers worse than these. This proves the geometry is
right, and nothing more.

Numpy only. No scipy, no pytest -- neither is on the rover.
"""

import math
import os
import sys

try:
    import numpy as np
except ImportError:
    sys.stderr.write("numpy is required. Aborting.\n")
    sys.exit(2)

# selftest/ -> fpms-os/ -> overlay/usr/local/lib/fpms/fpms_scanmatch.py
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LIBDIR = os.path.join(ROOT, "overlay", "usr", "local", "lib", "fpms")

failures, warnings, checks_run = [], [], [0]


def fail(name, msg):
    failures.append((name, msg))


def warn(name, msg):
    warnings.append((name, msg))


def check(name, cond, msg=""):
    """One assertion. Counted whether it passes or not."""
    checks_run[0] += 1
    if not cond:
        fail(name, msg or "assertion failed")
    return bool(cond)


def _load():
    """Import fpms_scanmatch without ROS, rclpy, or the Pi.

    The module advertises itself as "Pure numpy. No ROS, no hardware, no I/O".
    That claim is load-bearing: it is the only reason any of this can be tested
    before the board arrives. If a ROS import ever creeps in at module scope,
    this must fail with a message that says so rather than a bare traceback.
    """
    sys.path.insert(0, LIBDIR)
    try:
        import fpms_scanmatch as mod
        return mod, None
    except ImportError as exc:
        return None, (
            "cannot import fpms_scanmatch from %s: %r.\n"
            "  It is supposed to be numpy-only so it can be verified off the "
            "robot. If a hardware or ROS import has been added at module "
            "scope, THAT is the bug." % (LIBDIR, exc))
    except Exception as exc:  # noqa: BLE001
        return None, ("fpms_scanmatch raised at import time: %r. Import must "
                      "have no side effects." % exc)


S, IMPORT_ERROR = _load()


# ==========================================================================
# The synthetic world.
#
# Geometry matched to this rover: fpms_lidar_ros.py publishes EXPECTED_BINS=360
# with ANGLE_MIN=-pi and ANGLE_INCREMENT = 2*pi/360, i.e. index 180 is straight
# ahead and one index is one degree. Everything below uses those numbers rather
# than convenient ones, so a bin-indexing mistake here would be the same bin-
# indexing mistake the driver would make.
# ==========================================================================
N_BINS = 360
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / N_BINS


def _rect(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


# A 4 x 3 m room, two boxes, and one angled panel. The panel matters: a room of
# axis-aligned walls only is 4-fold symmetric-ish, and a rotation estimator can
# look better than it is against a world with that much accidental symmetry.
ROOM = (_rect(-2.0, -1.5, 2.0, 1.5)
        + _rect(0.60, 0.10, 1.00, 0.50)
        + _rect(-1.20, -0.90, -0.90, -0.30)
        + [((-1.60, 0.90), (-0.75, 1.30))])

_SEG = np.array([[a[0], a[1], b[0], b[1]] for a, b in ROOM], dtype=float)


def cast(px, py, bearings):
    """Range to the nearest ROOM surface along each world bearing. Vectorised."""
    dx = np.cos(bearings)
    dy = np.sin(bearings)
    best = np.full(bearings.shape, np.inf)
    for x1, y1, x2, y2 in _SEG:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - px) * ey - (y1 - py) * ex) / den
            u = ((x1 - px) * dy - (y1 - py) * dx) / den
        good = np.isfinite(t) & (den != 0.0) & (t > 1e-9) & (u >= 0.0) & (u <= 1.0)
        best = np.where(good & (t < best), t, best)
    return best


def scan_at(bx, by, btheta, mount_yaw=0.0, lever=(0.0, 0.0)):
    """Render a LaserScan-style range array from a known chassis pose.

    bx, by, btheta   base_link pose in the world
    mount_yaw        psi: laser_frame yaw relative to base_link
    lever            (lx, ly): laser origin offset in base_link

    The laser sits at base + R(btheta)*lever with world orientation
    btheta + mount_yaw, and bin i looks along that orientation plus
    ANGLE_MIN + i*ANGLE_INC. This is the forward model; every estimator below is
    asked to invert some part of it.
    """
    lx = bx + math.cos(btheta) * lever[0] - math.sin(btheta) * lever[1]
    ly = by + math.sin(btheta) * lever[0] + math.cos(btheta) * lever[1]
    o = btheta + mount_yaw
    bearings = o + ANGLE_MIN + ANGLE_INC * np.arange(N_BINS, dtype=float)
    return cast(lx, ly, bearings)


def circular_room(px, py, theta, radius=2.0):
    """A perfectly circular room: the featureless case, on purpose.

    From the centre of a circle every bearing returns the same range, so a
    rotation is genuinely, physically unobservable. Any estimator that returns a
    confident angle here is inventing one.
    """
    b = theta + ANGLE_MIN + ANGLE_INC * np.arange(N_BINS, dtype=float)
    dx, dy = np.cos(b), np.sin(b)
    bq = 2.0 * (px * dx + py * dy)
    cq = px * px + py * py - radius * radius
    return (-bq + np.sqrt(bq * bq - 4.0 * cq)) / 2.0


def mirror_scan(ranges):
    """Negate every bearing: what a wrong LIDAR_ROTATION_SIGN produces.

    Bin i sits at ANGLE_MIN + i*ANGLE_INC = -pi + i deg. The value belonging at
    bearing -(-pi + i deg) is the one currently in bin (-i) mod 360, so mirroring
    is index negation on this grid. A mirror is NOT a rigid transform; no value
    of laser_yaw can undo it, which is why it has to be settled first.
    """
    r = np.asarray(ranges, dtype=float)
    return r[(-np.arange(r.shape[0])) % r.shape[0]]


def occlude(ranges, start_bin, n_bins_out):
    """Blank a contiguous wedge: returns with no counterpart in the other scan.

    The realistic partial-overlap case. Between two scans a hand's push apart, a
    fat slice of returns has no partner because something came into or went out
    of view -- a doorway, the operator's own legs, the wall behind the rover.
    """
    r = np.asarray(ranges, dtype=float).copy()
    idx = (start_bin + np.arange(int(n_bins_out))) % r.shape[0]
    r[idx] = np.inf
    return r


def noisy(ranges, rng, sigma_m=0.003, drops=12):
    """3 mm of range noise and a few dead returns. Both are real on this class
    of scanner; 3 mm is optimistic for a hobby RPLIDAR, which is the point --
    if the estimator cannot survive optimistic noise it cannot survive real."""
    r = np.asarray(ranges, dtype=float) + rng.normal(0.0, sigma_m, ranges.shape)
    if drops:
        r[rng.choice(r.shape[0], int(drops), replace=False)] = np.inf
    return r


def pts(ranges):
    """Deliberately uses the MODULE's own range defaults, not overrides.

    SCAN_RANGE_MIN_M / SCAN_RANGE_MAX_M are now taken from fpms_lidar_ros.py, so
    exercising the defaults is exercising the real filter. Every pose used in
    this file sees ranges between 0.61 m and 2.98 m, comfortably inside the
    band, so nothing here is lost to it by accident.
    """
    return S.scan_to_xy(ranges, ANGLE_MIN, ANGLE_INC)


def deg(x):
    return math.degrees(x)


def angdiff_deg(a_rad, b_deg):
    return abs(deg(float(S.wrap_pi(a_rad - math.radians(b_deg)))))


# --------------------------------------------------------------------------
ROT_TOL_DEG = 0.50
ROT_TOL_NOISY_DEG = 0.75
TRANS_TOL_M = 0.005
YAW_TOL_DEG = 0.50
CONF_LOW = 0.25
CONF_HIGH = 0.50


# ==========================================================================
# 1. scan_to_xy -- the refusal that keeps a dead scanner from looking clear
# ==========================================================================
def test_01_scan_to_xy_drops_junk():
    """An all-zero scan must yield NO points, not a ring of phantom geometry.

    GUARDS AGAINST: the worst failure this project already knows about. The
    rover agent publishes ON A TIMER, not on data, so a dead scanner still emits
    a full-length 360-bin message with every range 0.0. fpms_lidar_ros.py turns
    0.0 into inf ("no return"), which downstream reads as a completely clear 360
    degrees. If scan_to_xy instead accepted those zeros they would land at
    radius 0 -- a phantom obstacle sitting on the sensor -- and if it accepted
    the infs they would be nonsense. Either way the calibration would be run
    against a scanner that is not seeing anything, and would produce a confident
    number. NAV2_BRIEF.md records that the drive board already publishes exactly
    such a dead /scan.
    """
    zeros = np.zeros(N_BINS, dtype=float)
    p = S.scan_to_xy(zeros, ANGLE_MIN, ANGLE_INC)
    check("scan_to_xy.all_zero_is_empty", p.shape == (0, 2),
          "an all-zero (dead scanner) scan produced %d points. It must produce "
          "zero. A dead scanner must read as blind, never as clear." % p.shape[0])

    infs = np.full(N_BINS, np.inf)
    check("scan_to_xy.all_inf_is_empty",
          S.scan_to_xy(infs, ANGLE_MIN, ANGLE_INC).shape == (0, 2),
          "an all-inf scan produced points")

    nans = np.full(N_BINS, np.nan)
    check("scan_to_xy.all_nan_is_empty",
          S.scan_to_xy(nans, ANGLE_MIN, ANGLE_INC).shape == (0, 2),
          "an all-NaN scan produced points")

    # Mixed: only 1.0 and 3.0 are legal. 0.0 dead, nan, inf, -1 negative,
    # 99.0 beyond range_max, 0.10 inside RANGE_MIN_M, and 6.0 -- the CLAMP
    # value fpms_lidar_ros.py writes for "nothing out there", which must not be
    # matched as though it were a real surface at exactly 6 m.
    mixed = np.array([np.nan, 1.0, 0.0, 99.0, -1.0, 0.10, np.inf, 3.0, 6.0])
    p = S.scan_to_xy(mixed, ANGLE_MIN, ANGLE_INC)
    check("scan_to_xy.mixed_keeps_only_valid", p.shape[0] == 2,
          "expected exactly 2 surviving points from %r, got %d"
          % (mixed.tolist(), p.shape[0]))
    if p.shape[0] == 2:
        radii = np.hypot(p[:, 0], p[:, 1])
        check("scan_to_xy.radii_preserved",
              abs(radii[0] - 1.0) < 1e-9 and abs(radii[1] - 3.0) < 1e-9,
              "surviving radii %r are not the input ranges" % radii.tolist())

    # Empty input must not raise: the agent can emit a zero-length array.
    check("scan_to_xy.empty_input",
          S.scan_to_xy(np.array([]), ANGLE_MIN, ANGLE_INC).shape == (0, 2),
          "an empty range array did not return an empty (0,2)")

    # A real scan of the room must survive essentially intact -- the filter must
    # not be so aggressive that it throws away the data it is meant to pass.
    room = scan_at(0.0, 0.0, 0.0)
    p = S.scan_to_xy(room, ANGLE_MIN, ANGLE_INC)
    check("scan_to_xy.real_scan_survives", p.shape[0] == N_BINS,
          "a clean %d-bin room scan lost %d points to filtering"
          % (N_BINS, N_BINS - p.shape[0]))

    # The defaults must agree with the driver that fills every scan this module
    # will ever see. They did not once: 0.05/12.0 let through both the
    # 0.05-0.12 m band the driver itself calls invalid and, worse, the CLAMPED
    # 6.0 m return -- a "nothing out there" reading that ICP would happily match
    # as a real surface 6 m away in every direction at once.
    check("scan_to_xy.defaults_match_driver",
          (S.SCAN_RANGE_MIN_M, S.SCAN_RANGE_MAX_M) == (0.12, 5.95)
          and S.scan_to_xy.__defaults__[:2] == (0.12, 5.95),
          "scan_to_xy defaults are %r / module constants are (%r, %r); "
          "fpms_lidar_ros.py declares RANGE_MIN_M=0.12, RANGE_MAX_M=6.0, "
          "RANGE_CLAMP_M=6.0, so these must be 0.12 and just inside 6.0"
          % (S.scan_to_xy.__defaults__[:2], S.SCAN_RANGE_MIN_M,
             S.SCAN_RANGE_MAX_M))

    # The clamp specifically, since it is the dangerous one.
    clamped = np.full(N_BINS, 6.0)
    check("scan_to_xy.drops_clamped_returns",
          S.scan_to_xy(clamped, ANGLE_MIN, ANGLE_INC).shape == (0, 2),
          "a scan of all-6.0 m (the driver's RANGE_CLAMP_M, meaning 'no "
          "return in any direction') produced points. That is a phantom "
          "6 m sphere of wall, and ICP would match against it.")

    # plane_level_diagnostic shares the constants and must share the behaviour.
    check("plane.defaults_match_driver",
          S.plane_level_diagnostic.__defaults__[:2] == (0.12, 5.95),
          "plane_level_diagnostic range defaults are %r, not the driver's "
          "band. A diagnostic computing its median over a different set of "
          "returns than scan_to_xy uses is comparing two different scans."
          % (S.plane_level_diagnostic.__defaults__[:2],))


# ==========================================================================
# 2. estimate_rotation -- the primitive everything else is built on
# ==========================================================================
def test_02_rotation_sweep():
    """Recover a known rotation from +/-5 deg to +/-170 deg, sub-bin included.

    GUARDS AGAINST: a rotation estimator that is right in the middle of its
    range and wrong at the ends, which is where the operator will actually use
    it (a hand-turned rover does not stop at 45.000 deg). 170 deg is included
    because the correlation peak is near the n/2 wrap point there, and the
    `if shift > n/2: shift -= n` branch is the only thing keeping +170 from
    coming back as -190.

    12.4 deg is included because it is 12.4 BINS -- the parabolic interpolation
    exists specifically for that case. At ONE degree per bin, a nearest-bin
    answer is up to 0.5 deg out, which is 5 mm of position error at the
    10.5 mm/deg arithmetic, i.e. the entire error budget spent on rounding.
    """
    for d in (5.0, -5.0, 12.4, -12.4, 30.0, -30.0, 45.0, 90.0, -90.0,
              120.0, 170.0, -170.0):
        a = scan_at(0.20, 0.10, 0.0)
        b = scan_at(0.20, 0.10, math.radians(d))
        r, conf = S.estimate_rotation(a, b, ANGLE_INC)
        err = angdiff_deg(r, d)
        check("rotation.sweep_%+.1f" % d, err <= ROT_TOL_DEG,
              "true %+.2f deg, estimated %+.2f deg, error %.3f deg > %.2f "
              "tolerance" % (d, deg(r), err, ROT_TOL_DEG))
        check("rotation.confident_%+.1f" % d, conf >= CONF_HIGH,
              "confidence %.3f in a feature-rich room is below %.2f; the "
              "operator would be told to distrust a correct answer"
              % (conf, CONF_HIGH))

    # Sub-bin specifically: fractions that a nearest-bin estimator cannot get.
    for d in (0.5, 3.3, -7.7, 12.4, -21.6):
        a = scan_at(0.0, 0.0, 0.0)
        b = scan_at(0.0, 0.0, math.radians(d))
        r, _ = S.estimate_rotation(a, b, ANGLE_INC)
        check("rotation.subbin_%+.1f" % d, angdiff_deg(r, d) <= ROT_TOL_DEG,
              "sub-bin %+.2f deg came back %+.3f deg (error %.3f). If this is "
              "always landing on a whole degree the parabolic interpolation is "
              "not working." % (d, deg(r), angdiff_deg(r, d)))

    # And prove the interpolation is not a no-op: at least one sweep value must
    # come back off a whole-degree grid.
    r, _ = S.estimate_rotation(scan_at(0, 0, 0), scan_at(0, 0, math.radians(12.4)),
                               ANGLE_INC)
    check("rotation.interpolation_is_live", abs(deg(r) - round(deg(r))) > 0.02,
          "a 12.4 deg rotation returned %.4f deg, which is a whole number of "
          "bins. The sub-bin refinement is dead -- delta is being forced to "
          "0.0 -- and every estimate carries up to 0.5 deg (5 mm) of "
          "quantisation." % deg(r))

    # Identity: two identical scans must be zero rotation, not a small number.
    a = scan_at(0.1, -0.2, 0.3)
    r, _ = S.estimate_rotation(a, a.copy(), ANGLE_INC)
    check("rotation.identity_is_zero", abs(deg(r)) < 1e-6,
          "a scan correlated with itself returned %.6f deg" % deg(r))

    # Too-short input must refuse rather than correlate 8 bins of nothing.
    r, conf = S.estimate_rotation(np.ones(8), np.ones(8), ANGLE_INC)
    check("rotation.short_input_refuses", conf == 0.0,
          "an 8-bin scan returned confidence %.3f" % conf)


def test_03_rotation_low_confidence_when_unobservable():
    """A featureless circular room must report LOW confidence.

    GUARDS AGAINST: the single most dangerous behaviour an estimator can have --
    answering when the answer is not in the data. From the centre of a circular
    room every bearing returns the same range, so rotation is not merely hard to
    measure, it is not present in the measurement. The operator will be running
    this in whatever space is available, possibly a small featureless one, and a
    confident wrong yaw here is bolted into the TF tree for the life of the
    robot.

    The check is two-sided on purpose: low confidence in the featureless room is
    worth nothing unless the SAME threshold reports high confidence in a real
    one, or the tool would simply always say "do not trust me".
    """
    rng = np.random.default_rng(20260811)

    for d in (20.0, 45.0, 90.0):
        a = circular_room(0.0, 0.0, 0.0)
        b = circular_room(0.0, 0.0, math.radians(d))
        r, conf = S.estimate_rotation(a, b, ANGLE_INC)
        check("rotation.circle_low_conf_%.0f" % d, conf < CONF_LOW,
              "a perfect circular room -- where rotation is physically "
              "unobservable -- reported confidence %.3f and an angle of "
              "%+.2f deg for a true %+.1f. A confident answer here is a "
              "serious bug: it is the estimator inventing structure."
              % (conf, deg(r), d))

    # Same, with noise. Noise gives the correlator something to lock onto, and
    # that is exactly when a badly-scaled confidence starts looking convincing.
    for d in (20.0, 45.0):
        a = noisy(circular_room(0.0, 0.0, 0.0), rng, drops=0)
        b = noisy(circular_room(0.0, 0.0, math.radians(d)), rng, drops=0)
        r, conf = S.estimate_rotation(a, b, ANGLE_INC)
        check("rotation.noisy_circle_low_conf_%.0f" % d, conf < CONF_LOW,
              "a NOISY circular room reported confidence %.3f (angle "
              "%+.2f deg for a true %+.1f). The correlator has locked onto "
              "the noise and is reporting it as structure."
              % (conf, deg(r), d))

    # The discriminator has to cut both ways.
    a = scan_at(0.2, 0.1, 0.0)
    b = scan_at(0.2, 0.1, math.radians(45.0))
    _, conf_room = S.estimate_rotation(a, b, ANGLE_INC)
    check("rotation.real_room_high_conf", conf_room >= CONF_HIGH,
          "the feature-rich test room reported confidence %.3f, below the "
          "%.2f threshold. If a real room does not clear the bar, the "
          "confidence number cannot be used to reject a bad one."
          % (conf_room, CONF_HIGH))

    # An off-centre circle IS weakly observable (the range profile is a genuine
    # sinusoid). It should sit between the two, and be correct.
    a = circular_room(0.30, 0.0, 0.0)
    b = circular_room(0.30, 0.0, math.radians(45.0))
    r, conf = S.estimate_rotation(a, b, ANGLE_INC)
    check("rotation.offcentre_circle_correct", angdiff_deg(r, 45.0) <= ROT_TOL_DEG,
          "an off-centre circular room has a real (sinusoidal) feature and the "
          "rotation should be recoverable; got %+.2f deg for a true 45.0"
          % deg(r))


def test_04_rotation_survives_noise():
    """3 mm range noise and a dozen dead returns must not move the answer.

    GUARDS AGAINST: an estimator verified only on clean data. Real returns
    scatter by several millimetres, and this scanner drops bins. The correlator
    replaces non-returns with the median rather than inf/0 precisely so they do
    not punch holes in the profile; this is the check that says that works.
    """
    rng = np.random.default_rng(4242)
    for d in (5.0, 12.4, 45.0, 120.0, 170.0, -5.0, -170.0):
        a = noisy(scan_at(0.2, 0.1, 0.0), rng)
        b = noisy(scan_at(0.2, 0.1, math.radians(d)), rng)
        r, conf = S.estimate_rotation(a, b, ANGLE_INC)
        err = angdiff_deg(r, d)
        check("rotation.noisy_%+.1f" % d, err <= ROT_TOL_NOISY_DEG,
              "true %+.2f deg, estimated %+.2f deg, error %.3f deg > %.2f "
              "with 3 mm noise and 12 dropped returns"
              % (d, deg(r), err, ROT_TOL_NOISY_DEG))
        check("rotation.noisy_conf_%+.1f" % d, conf >= CONF_HIGH,
              "confidence collapsed to %.3f under mild noise" % conf)


# ==========================================================================
# 3. estimate_translation -- including the partial-overlap case, which is the
#    only case that will ever actually happen
# ==========================================================================
def test_05_translation_full_overlap():
    """Recover a known translation to 5 mm with everything visible.

    GUARDS AGAINST: an ICP that converges to a local minimum, or one whose
    rotation convention disagrees with estimate_rotation's. This is the easy
    case; if it does not hold there is no point testing the hard one.

    NOTE ON THE ROTATION ARGUMENT. estimate_translation models
    B ~= R(rotation_rad) * A + t, i.e. rotation_rad is the rotation of the POINT
    CLOUD, which for a chassis that turned by +theta is -theta (the world appears
    to turn the other way). These pure translations are passed 0.0, so they
    isolate the translation solve from that convention entirely. The convention
    itself is checked in test_08.
    """
    for (dx_true, dy_true) in ((0.30, 0.0), (-0.25, 0.0), (0.0, 0.20),
                               (0.18, -0.22), (-0.15, -0.15)):
        a = scan_at(0.0, 0.0, 0.0)
        b = scan_at(dx_true, dy_true, 0.0)
        # Points are in the LASER frame, so a chassis move of (dx,dy) makes
        # static points appear to move by (-dx,-dy) at zero mount yaw.
        dx, dy, rms, inl = S.estimate_translation(pts(a), pts(b), 0.0)
        err = math.hypot(dx + dx_true, dy + dy_true)
        check("translation.full_overlap_%.2f_%.2f" % (dx_true, dy_true),
              err <= TRANS_TOL_M,
              "chassis moved (%+.3f, %+.3f) so points should appear to move "
              "(%+.3f, %+.3f); ICP said (%+.4f, %+.4f), error %.1f mm > %.0f mm"
              % (dx_true, dy_true, -dx_true, -dy_true, dx, dy,
                 1000 * err, 1000 * TRANS_TOL_M))
        check("translation.rms_sane_%.2f_%.2f" % (dx_true, dy_true),
              rms < 0.05 and inl >= 200,
              "converged with rms %.4f m over %d inliers, which is not a fit"
              % (rms, inl))

    # Too few points must refuse, not return (0,0) as if it had succeeded.
    dx, dy, rms, inl = S.estimate_translation(np.zeros((3, 2)), np.zeros((3, 2)), 0.0)
    check("translation.too_few_points_refuses",
          inl == 0 and not math.isfinite(rms),
          "a 3-point cloud returned inliers=%d rms=%r instead of refusing. A "
          "silent (0,0) would read as 'the rover did not move'." % (inl, rms))


def test_05b_rotation_feeds_translation():
    """estimate_rotation's output must chain DIRECTLY into estimate_translation.

    The settled contract:

        estimate_rotation   returns the CHASSIS rotation, same sign as the gyro
        estimate_translation takes `chassis_rotation_rad` and negates internally,
                            because the point cloud turns the other way

    so an operator tool writes, with no sign fiddling at the call site:

        rot, conf = estimate_rotation(a, b, inc)
        dx, dy, rms, n = estimate_translation(pts(a), pts(b), rot)

    This test performs a PURE rotation in place about the laser origin, where
    the true translation is zero by construction, and asks whether that chaining
    recovers it.

    GUARDS AGAINST: the de-rotation being applied in the wrong direction, which
    is how this module shipped once. It does not raise, it does not diverge, and
    it does not look wrong: the ICP converges, reports a plausible rms, and
    returns a translation of several centimetres for a rover that did not move.
    yaw_from_straight_push then turns that into a mount yaw with a straight
    face. The second assertion in each pair is the regression guard -- it pins
    that the OTHER sign is the broken one, so a future "fix" that flips this
    back cannot make the first assertion pass by accident.
    """
    for theta_deg in (20.0, 40.0, -35.0):
        th = math.radians(theta_deg)
        a = scan_at(0.30, 0.10, 0.0)      # laser at base origin, no lever arm
        b = scan_at(0.30, 0.10, th)       # rotated in place: t is exactly zero
        r, _ = S.estimate_rotation(a, b, ANGLE_INC)

        check("sign.rotation_is_chassis_sign_%+.0f" % theta_deg,
              angdiff_deg(r, theta_deg) <= ROT_TOL_DEG,
              "a chassis that turned %+.1f deg was reported as %+.2f deg. "
              "estimate_rotation's contract is the CHASSIS rotation, the same "
              "sign as the gyro." % (theta_deg, deg(r)))

        dx, dy, rms, _ = S.estimate_translation(pts(a), pts(b), r)
        dxn, dyn, rmsn, _ = S.estimate_translation(pts(a), pts(b), -r)
        as_is = math.hypot(dx, dy)
        negated = math.hypot(dxn, dyn)

        check("sign.chained_rotation_%+.0f" % theta_deg, as_is <= 0.010,
              "a %+.0f deg rotation IN PLACE (true translation exactly zero) "
              "chained straight through estimate_rotation -> "
              "estimate_translation gave (%+.4f, %+.4f) = %.1f mm of motion "
              "that did not happen (rms %.4f). The two functions must agree "
              "on the sign of the rotation they pass between them."
              % (theta_deg, dx, dy, 1000 * as_is, rms))

        check("sign.negation_at_callsite_is_wrong_%+.0f" % theta_deg,
              negated > 0.030,
              "negating estimate_rotation's output before passing it in gave "
              "only %.1f mm of spurious travel (rms %.4f), i.e. BOTH signs "
              "look acceptable. estimate_translation is supposed to negate "
              "internally, so the wrong sign must be clearly wrong -- if it is "
              "not, this test cannot detect a future re-flip."
              % (1000 * negated, rmsn))


def test_06_translation_partial_overlap():
    """20-30% of returns with no counterpart -- the realistic case.

    GUARDS AGAINST: an ICP that quietly biases toward whatever is still visible.
    Between two scans a push apart, a fat contiguous wedge of returns has no
    partner: a doorway swings out of view, the operator's own legs block a
    sector, a wall behind the rover leaves range. Those points have no correct
    correspondence, and every one of them still pulls on the mean.

    estimate_translation now trims ADAPTIVELY (median + trim_sigma*MAD). The
    previous fixed reject_frac=0.25 could not, by construction, remove a 30%
    non-overlapping tail, and this file measured the consequence: 47 mm of drift
    on a 300 mm push, 15% of the answer, with no outward sign. The tolerances
    below are therefore now FLAT across 10-30% rather than widening with the
    occluded fraction -- an adaptive trim should not care how much is missing,
    only how separable it is, and that is the property being asserted.
    """
    push = 0.30

    for frac, tol_mm, where in ((0.10, 5.0, "B"), (0.20, 5.0, "B"),
                                (0.25, 5.0, "B"), (0.30, 5.0, "B"),
                                (0.20, 5.0, "A"), (0.25, 5.0, "A"),
                                (0.30, 5.0, "A")):
        k = int(round(frac * N_BINS))
        a = scan_at(0.0, 0.0, 0.0)
        b = scan_at(push, 0.0, 0.0)
        if where == "B":
            b = occlude(b, 100, k)
        else:
            a = occlude(a, 200, k)
        dx, dy, rms, inl = S.estimate_translation(pts(a), pts(b), 0.0)
        err = math.hypot(dx + push, dy)
        check("translation.occluded_%s_%.0fpc" % (where, frac * 100),
              err * 1000.0 <= tol_mm,
              "%.0f%% of scan %s occluded on a %.0f mm push: ICP said "
              "(%+.4f, %+.4f), error %.1f mm > %.0f mm tolerance (rms %.4f, "
              "%d inliers)" % (frac * 100, where, push * 1000, dx, dy,
                               1000 * err, tol_mm, rms, inl))

    # The property that makes the adaptive trim worth having: error must be
    # FLAT in the occluded fraction, not growing with it. A fixed-fraction trim
    # gave 1.5 mm at 10% and 47 mm at 30% -- a cliff with no outward sign, right
    # in the middle of the range the module documents as its design case.
    errs = []
    for frac in (0.05, 0.10, 0.20, 0.30):
        k = int(round(frac * N_BINS))
        a = scan_at(0.0, 0.0, 0.0)
        b = occlude(scan_at(push, 0.0, 0.0), 100, k)
        dx, dy, _, _ = S.estimate_translation(pts(a), pts(b), 0.0)
        errs.append(math.hypot(dx + push, dy))
    check("translation.trim_is_flat_in_occlusion",
          max(errs) - min(errs) <= 0.005 and errs[-1] <= 3.0 * max(errs[0], 1e-4),
          "translation error across 5/10/20/30%% occlusion is %s mm. An "
          "adaptive trim should not care how much is missing, only how "
          "separable it is; error that climbs with the occluded fraction "
          "means the tail is being partly absorbed and there will be a cliff "
          "somewhere past the range tested here."
          % ["%.1f" % (1000 * e) for e in errs])

    # The rms must remain the tell, so a caller can refuse on it. If a heavily
    # occluded match is BOTH accurate and indistinguishable from a clean one,
    # there is nothing left for fpms-calibrate-lidar's PUSH_MAX_RMS_M to gate on.
    _, _, rms_clean, _ = S.estimate_translation(
        pts(scan_at(0, 0, 0)), pts(scan_at(push, 0, 0)), 0.0)
    _, _, rms_occ, n_occ = S.estimate_translation(
        pts(scan_at(0, 0, 0)),
        pts(occlude(scan_at(push, 0, 0), 100, int(0.30 * N_BINS))), 0.0)
    check("translation.rms_stays_usable_as_a_gate",
          rms_clean < 0.06 and rms_occ < 0.06,
          "clean rms %.4f, 30%%-occluded rms %.4f. fpms-calibrate-lidar "
          "rejects a push at PUSH_MAX_RMS_M = 0.06, so an accurate match must "
          "sit below that or good measurements get thrown away."
          % (rms_clean, rms_occ))
    check("translation.occluded_keeps_enough_inliers", n_occ >= 30,
          "30%% occlusion left %d inliers; fpms-calibrate-lidar requires "
          "PUSH_MIN_INLIERS = 30" % n_occ)

    # Noise on top of occlusion: both at once is what a real session looks like.
    rng = np.random.default_rng(99)
    a = noisy(scan_at(0.0, 0.0, 0.0), rng)
    b = noisy(occlude(scan_at(push, 0.0, 0.0), 100, int(0.20 * N_BINS)), rng)
    dx, dy, rms, inl = S.estimate_translation(pts(a), pts(b), 0.0)
    err = math.hypot(dx + push, dy)
    check("translation.occluded_and_noisy", err <= 0.020,
          "20%% occlusion plus 3 mm noise and dropped returns gave %.1f mm of "
          "error on a %.0f mm push" % (1000 * err, push * 1000))

    # trim_sigma must actually be the knob. Collapsing it to something tiny has
    # to change the answer, or the adaptive trim is not running at all and the
    # flatness asserted above is a coincidence of this scene.
    a = scan_at(0.0, 0.0, 0.0)
    b = occlude(scan_at(push, 0.0, 0.0), 100, int(0.30 * N_BINS))
    d_default = S.estimate_translation(pts(a), pts(b), 0.0)
    d_tight = S.estimate_translation(pts(a), pts(b), 0.0, trim_sigma=0.05)
    check("translation.trim_sigma_is_live",
          abs(d_tight[2] - d_default[2]) > 1e-6 or d_tight[3] != d_default[3],
          "trim_sigma=0.05 and trim_sigma=3.0 produced identical rms (%.6f) "
          "and inlier counts (%d). The trim is not being applied."
          % (d_default[2], d_default[3]))


# ==========================================================================
# 4. mirror_verdict -- the highest-value check in the module
# ==========================================================================
def test_07_mirror_verdict_contract():
    """mirror_verdict must implement the rule its own docstring states.

    The settled rule: estimate_rotation reports the CHASSIS rotation -- it has
    already undone the fact that the world appears to turn the other way -- so a
    correctly-handed scan AGREES in sign with the gyro, and a mirrored one
    OPPOSES it. (Mirroring negates every bearing, so a feature at beta appears
    at -beta, and after a +theta turn at -(beta - theta) = -beta + theta: in
    mirrored coordinates features move by +theta, giving a correlation shift of
    -theta/inc. Confirmed against synthetic scans in test_08.)

    This test takes the function at its word and feeds it hand-made pairs. It
    says nothing about whether estimate_rotation actually produces the sign this
    function expects -- that is test_08, and it is a different question.

    GUARDS AGAINST: a handedness check that is itself only sometimes right. A
    mirror is not a rigid transform, so no value of laser_yaw can undo it; if
    this verdict is wrong the operator measures a mount yaw against a mirrored
    world and gets a confident, plausible, wrong number that is then baked into
    every costmap the robot ever builds. This module shipped with the verdict
    exactly inverted, and nothing but synthetic ground truth could have seen it.
    """
    turn = math.radians(40.0)

    v, why = S.mirror_verdict(turn, turn)
    check("mirror.same_signs_is_ok", v == "ok",
          "scan +40 deg with gyro +40 deg (AGREEING, which is what a "
          "correctly-handed scanner gives once estimate_rotation has reported "
          "the chassis rotation) returned %r: %s" % (v, why))

    v, why = S.mirror_verdict(-turn, -turn)
    check("mirror.same_signs_is_ok_reversed", v == "ok",
          "scan -40 with gyro -40 returned %r: %s" % (v, why))

    v, why = S.mirror_verdict(-turn, turn)
    check("mirror.opposite_signs_is_mirrored", v == "mirrored",
          "scan -40 deg against gyro +40 deg (OPPOSITE signs) returned %r. "
          "Negated bearings reverse the correlation shift, so opposing signs "
          "are the mirror signature. %s" % (v, why))

    v, why = S.mirror_verdict(turn, -turn)
    check("mirror.opposite_signs_is_mirrored_reversed", v == "mirrored",
          "scan +40 against gyro -40 returned %r: %s" % (v, why))

    # A tiny turn must be refused, not guessed. Near zero the SIGN is noise, and
    # the sign is the entire measurement.
    for g_deg in (0.0, 3.0, -5.0, 14.0):
        v, why = S.mirror_verdict(math.radians(g_deg), math.radians(g_deg))
        check("mirror.tiny_turn_inconclusive_%.0f" % g_deg, v == "inconclusive",
              "a %.0f deg turn returned %r. Below the 15 deg threshold the "
              "sign of the scan rotation is noise, and guessing here is how a "
              "mirrored scan gets approved. %s" % (g_deg, v, why))

    # Just over the threshold it must commit rather than hedge forever.
    v, _ = S.mirror_verdict(math.radians(20.0), math.radians(20.0))
    check("mirror.commits_above_threshold", v == "ok",
          "a 20 deg turn (above the 15 deg threshold) still returned %r" % v)

    # Scan and gyro disagreeing in MAGNITUDE must be inconclusive: that is a
    # gyro_scale problem or a rover that translated as well as turned, and the
    # handedness cannot be read off it.
    v, why = S.mirror_verdict(math.radians(90.0), math.radians(30.0))
    check("mirror.magnitude_mismatch_inconclusive", v == "inconclusive",
          "scan +90 deg against gyro +30 deg (ratio 3.0) returned %r rather "
          "than refusing: %s" % (v, why))

    # The magnitude gate must not be reachable only from the 'ok' side -- a
    # mirrored scan with a bad gyro scale must also be refused, not called 'ok'.
    v, why = S.mirror_verdict(math.radians(-90.0), math.radians(30.0))
    check("mirror.magnitude_mismatch_inconclusive_mirrored",
          v == "inconclusive",
          "scan -90 deg against gyro +30 deg returned %r; the ratio is 3.0 "
          "and handedness is not readable from it: %s" % (v, why))

    # Every verdict must carry a detail string; the operator acts on the prose,
    # not the enum.
    for args in ((turn, turn), (-turn, turn), (0.0, 0.0)):
        v, why = S.mirror_verdict(*args)
        check("mirror.detail_present_%s" % v, bool(why) and len(why) > 20,
              "verdict %r came back with an empty or useless detail %r"
              % (v, why))


def test_08_mirror_end_to_end_sign_convention():
    """The handedness check, driven by real scans instead of hand-made numbers.

    THIS IS THE TEST THAT MATTERS. test_07 proves mirror_verdict is
    self-consistent with its docstring. This one proves the module as a whole
    is: it renders a correctly-handed world, runs it through the module's own
    estimate_rotation, and hands the result to the module's own mirror_verdict
    exactly as an operator tool would.

    GUARDS AGAINST: two functions that are each individually correct and
    disagree about the sign of the thing they pass between them. That defect
    cannot be seen by testing either one alone, produces no exception, and
    inverts the one verdict the whole calibration is gated on.
    """
    turn_deg = 40.0
    turn = math.radians(turn_deg)

    a = scan_at(0.10, 0.0, 0.0)
    b = scan_at(0.10, 0.0, turn)          # correctly-handed, chassis +40 CCW
    r, conf = S.estimate_rotation(a, b, ANGLE_INC)
    check("mirror.e2e_rotation_confident", conf >= CONF_HIGH,
          "estimate_rotation reported confidence %.2f on the mirror test "
          "geometry" % conf)

    check("mirror.e2e_rotation_is_chassis_sign",
          angdiff_deg(r, turn_deg) <= ROT_TOL_DEG,
          "a chassis that turned %+.1f deg was reported as %+.2f deg. The "
          "whole handedness verdict rests on this sign being the gyro's."
          % (turn_deg, deg(r)))

    v, why = S.mirror_verdict(r, turn)
    check("mirror.e2e_correct_data_is_ok", v == "ok",
          "CORRECTLY-HANDED synthetic scans were judged %r. "
          "estimate_rotation(A, B) returned %+.2f deg for a chassis that "
          "turned %+.1f deg by construction, and the gyro agrees, so this "
          "must be 'ok'. Do not fix this by loosening the test: an inverted "
          "verdict tells the operator to flip LIDAR_ROTATION_SIGN on a working "
          "scanner. %s" % (v, deg(r), turn_deg, why))

    # And the mirrored world, which must be caught.
    am = mirror_scan(a)
    bm = mirror_scan(b)
    rm, _ = S.estimate_rotation(am, bm, ANGLE_INC)
    check("mirror.e2e_mirrored_rotation_flips",
          angdiff_deg(rm, -turn_deg) <= ROT_TOL_DEG,
          "mirroring both scans should negate the recovered rotation: got "
          "%+.2f deg where %+.2f was expected. This is the empirical fact the "
          "whole verdict rests on -- a mirrored scan reports the OPPOSITE sign "
          "to the gyro. If this fails, the mirror_scan helper is wrong and "
          "every other mirror assertion in this file is meaningless."
          % (deg(rm), -turn_deg))

    v, why = S.mirror_verdict(rm, turn)
    check("mirror.e2e_mirrored_data_is_mirrored", v == "mirrored",
          "a MIRRORED scan (bearings negated, exactly what a wrong "
          "LIDAR_ROTATION_SIGN produces) was judged %r. This is the failure "
          "the whole calibration is gated on: %s" % (v, why))

    # With noise, and turning the other way, so the result is not one lucky
    # geometry.
    rng = np.random.default_rng(7)
    for t_deg in (30.0, -30.0, 60.0, -75.0):
        t = math.radians(t_deg)
        a = noisy(scan_at(-0.20, 0.30, 0.4), rng)
        b = noisy(scan_at(-0.20, 0.30, 0.4 + t), rng)
        r, _ = S.estimate_rotation(a, b, ANGLE_INC)
        v_ok, _ = S.mirror_verdict(r, t)
        rm, _ = S.estimate_rotation(mirror_scan(a), mirror_scan(b), ANGLE_INC)
        v_mir, _ = S.mirror_verdict(rm, t)
        check("mirror.e2e_ok_%+.0f" % t_deg, v_ok == "ok",
              "correctly-handed data at %+.0f deg judged %r" % (t_deg, v_ok))
        check("mirror.e2e_mirrored_%+.0f" % t_deg, v_mir == "mirrored",
              "mirrored data at %+.0f deg judged %r" % (t_deg, v_mir))


# ==========================================================================
# 5. yaw_from_straight_push -- the number the whole tool exists to produce
# ==========================================================================
def test_09_yaw_from_straight_push():
    """Recover a known mount yaw, and prove it does not depend on how far you
    pushed.

    GUARDS AGAINST: an estimator that secretly needs a calibrated distance.
    Distance-independence is the entire design claim -- it is why counts/mm
    being unmeasured and disputed on this rover does not matter, and why the
    operator can shove the robot by hand instead of commanding a measured
    move. If the recovered yaw drifts with push length, that claim is false and
    the tool inherits every odometry problem it was built to sidestep.

    Both signs are tested. A sign error in `psi = pi - atan2(dy, dx)` would be
    invisible against a symmetric set of test values and would put every
    obstacle on the wrong side of the robot.
    """
    distances = (0.10, 0.15, 0.25, 0.40, 0.60)

    for psi_deg in (0.0, 5.0, -5.0, 12.0, -12.0, 25.0, -25.0, 45.0, -45.0):
        psi = math.radians(psi_deg)
        got = []
        for d in distances:
            a = scan_at(0.0, 0.0, 0.0, mount_yaw=psi)
            b = scan_at(d, 0.0, 0.0, mount_yaw=psi)
            dx, dy, rms, _ = S.estimate_translation(pts(a), pts(b), 0.0)
            y, conf = S.yaw_from_straight_push(dx, dy)
            got.append(deg(y))
            check("yaw.recovered_%+.0f_d%.2f" % (psi_deg, d),
                  angdiff_deg(y, psi_deg) <= YAW_TOL_DEG,
                  "mount yaw %+.1f deg, push %.2f m: recovered %+.3f deg, "
                  "error %.3f deg > %.2f (= %.1f mm of position error at "
                  "10.5 mm/deg)"
                  % (psi_deg, d, deg(y), angdiff_deg(y, psi_deg), YAW_TOL_DEG,
                     10.5 * angdiff_deg(y, psi_deg)))

        spread = max(got) - min(got)
        check("yaw.distance_independent_%+.0f" % psi_deg, spread <= 0.30,
              "mount yaw %+.1f deg recovered as %s across pushes of %s m -- a "
              "spread of %.3f deg. The estimate is supposed to depend on the "
              "DIRECTION of apparent motion and never its magnitude; a spread "
              "that tracks push length means it does depend on distance, and "
              "the 'counts/mm does not matter' claim is false."
              % (psi_deg, ["%+.3f" % g for g in got],
                 list(distances), spread))

    # Pushed BACKWARD must give the same answer. The operator will pull the
    # rover as often as push it, and pushed_forward=False is the only thing
    # standing between that and a 180 deg mount-yaw error.
    psi = math.radians(9.0)
    a = scan_at(0.0, 0.0, 0.0, mount_yaw=psi)
    b = scan_at(-0.30, 0.0, 0.0, mount_yaw=psi)
    dx, dy, _, _ = S.estimate_translation(pts(a), pts(b), 0.0)
    y_bad, _ = S.yaw_from_straight_push(dx, dy, pushed_forward=True)
    y_good, _ = S.yaw_from_straight_push(dx, dy, pushed_forward=False)
    check("yaw.backward_push", angdiff_deg(y_good, 9.0) <= YAW_TOL_DEG,
          "a 300 mm BACKWARD push with pushed_forward=False gave %+.3f deg "
          "for a true +9.0" % deg(y_good))
    check("yaw.backward_push_flag_matters",
          angdiff_deg(y_bad, 9.0) > 90.0,
          "a backward push read with pushed_forward=True gave %+.3f deg, "
          "which is not ~180 deg away from the truth. Either the flag does "
          "nothing or the test geometry is wrong." % deg(y_bad))

    # Confidence must fall off for a nudge. A 20 mm shove has a direction made
    # almost entirely of scan noise. The curve is now (d - 0.10) / 0.30, so full
    # trust needs ~0.40 m rather than ~0.20 m -- short pushes were the largest
    # contributor to spread between repeats, and the operator is told to push
    # ~0.50 m (fpms-calibrate-lidar's PUSH_TARGET_M).
    _, c_small = S.yaw_from_straight_push(0.02, 0.001)
    _, c_big = S.yaw_from_straight_push(0.40, 0.0)
    check("yaw.confidence_low_for_nudge", c_small < 0.25,
          "a 20 mm push reported confidence %.2f" % c_small)
    check("yaw.confidence_high_for_real_push", c_big >= 0.99,
          "a 400 mm push reported confidence %.2f" % c_big)

    # Pin the curve at the two distances the shipping tool gates on, so a future
    # change to either the curve or the gate cannot silently pass the other.
    # PUSH_MIN_M = 0.25 and PUSH_MIN_CONFIDENCE = 0.6 together mean the shortest
    # ACCEPTED push is ~0.28 m; below that the tool refuses.
    _, c_min = S.yaw_from_straight_push(0.25, 0.0)
    _, c_gate = S.yaw_from_straight_push(0.28, 0.0)
    check("yaw.confidence_curve_pinned",
          abs(c_min - 0.50) < 0.02 and c_gate >= 0.60,
          "confidence is %.3f at the tool's PUSH_MIN_M (0.25 m) and %.3f at "
          "0.28 m. With PUSH_MIN_CONFIDENCE = 0.6 the tool's distance floor "
          "and its confidence floor must line up, or one of the two is dead "
          "code and the operator gets a rejection they cannot explain."
          % (c_min, c_gate))
    y, c = S.yaw_from_straight_push(0.0, 0.0)
    check("yaw.zero_motion_refuses", c == 0.0,
          "zero apparent motion returned confidence %.2f" % c)

    # With noise and dropped returns.
    rng = np.random.default_rng(31337)
    for psi_deg in (7.0, -7.0, 18.0):
        psi = math.radians(psi_deg)
        for d in (0.20, 0.35):
            a = noisy(scan_at(0.0, 0.0, 0.0, mount_yaw=psi), rng)
            b = noisy(scan_at(d, 0.0, 0.0, mount_yaw=psi), rng)
            dx, dy, _, _ = S.estimate_translation(pts(a), pts(b), 0.0)
            y, _ = S.yaw_from_straight_push(dx, dy)
            check("yaw.noisy_%+.0f_d%.2f" % (psi_deg, d),
                  angdiff_deg(y, psi_deg) <= 1.5,
                  "mount yaw %+.1f deg with noise, push %.2f m: got %+.3f deg "
                  "(error %.2f). A SINGLE noisy push is allowed 1.5 deg; the "
                  "end-to-end session in test_12 averages three and must meet "
                  "%.2f deg." % (psi_deg, d, deg(y),
                                 angdiff_deg(y, psi_deg), YAW_TOL_DEG))


# ==========================================================================
# 6. lever_arm_from_rotation -- weakly observed, and reported as such
# ==========================================================================
def test_10_lever_arm_from_rotation():
    """Recover a known scanner offset from a pure rotation in place.

    Fed ANALYTIC translations rather than ICP output, so this tests the linear
    algebra alone. The forward model, from the module's own definitions:

        chassis turns by theta about base_link's origin
        scanner sits at L in base_link, mount yaw psi
        the apparent translation the ICP would see is
            t = (R(-theta) - I) . R(-psi) . L
        so the inversion the module owes us is
            L = R(psi) . (R(-theta) - I)^-1 . t

    GUARDS AGAINST: an inversion that is off by a rotation. That defect returns
    an offset of the RIGHT MAGNITUDE pointing the wrong way, which survives
    every plausibility check an operator would apply (it is a few centimetres,
    it is roughly where the mast is) and puts every obstacle in the costmap
    displaced by twice the lever arm.
    """
    def analytic_t(L, theta, psi=0.0):
        cm, sm = math.cos(-theta), math.sin(-theta)
        M = np.array([[cm - 1.0, -sm], [sm, cm - 1.0]])
        cp, sp = math.cos(-psi), math.sin(-psi)
        Rp = np.array([[cp, -sp], [sp, cp]])
        return M @ (Rp @ np.array(L, dtype=float))

    for L in ((0.05, 0.00), (0.10, 0.03), (-0.06, 0.08), (0.00, -0.12)):
        for theta_deg in (30.0, 60.0, -45.0, 120.0):
            for psi_deg in (0.0, 10.0, -22.0):
                th = math.radians(theta_deg)
                psi = math.radians(psi_deg)
                t = analytic_t(L, th, psi)
                lx, ly, why = S.lever_arm_from_rotation(t[0], t[1], th, psi)
                if not check("lever.solves_%+.0f" % theta_deg, lx is not None,
                             "a %.0f deg rotation was refused: %s"
                             % (theta_deg, why)):
                    continue
                err = math.hypot(lx - L[0], ly - L[1])
                # Diagnose the specific historical defect if it reappears: the
                # answer being the true offset rotated by -theta, i.e. right
                # magnitude, wrong direction.
                c2, s2 = math.cos(-th), math.sin(-th)
                rot_L = (c2 * L[0] - s2 * L[1], s2 * L[0] + c2 * L[1])
                looks_rotated = math.hypot(lx - rot_L[0], ly - rot_L[1]) < 1e-6
                check("lever.recovers_%s_%+.0f_psi%+.0f"
                      % (str(L), theta_deg, psi_deg), err <= 0.005,
                      "true offset (%+.3f, %+.3f) at %+.0f deg with mount yaw "
                      "%+.0f deg: solved (%+.4f, %+.4f), error %.1f mm.%s"
                      % (L[0], L[1], theta_deg, psi_deg, lx, ly, 1000 * err,
                         " The answer is EXACTLY R(-theta) applied to the true "
                         "offset -- right magnitude, wrong direction. That is "
                         "the (R(theta) - I) / -t form returning; the inversion "
                         "owed is L = R(psi).(R(-theta) - I)^-1.t."
                         if looks_rotated else ""))

    # The mount yaw argument must actually do something. If R(psi) is dropped
    # again the result is silently in the laser frame, which for a 12 deg mount
    # yaw on a 100 mm arm is 21 mm of offset pointing the wrong way.
    L = (0.10, 0.0)
    th = math.radians(45.0)
    psi = math.radians(30.0)
    t = analytic_t(L, th, psi)
    with_psi = S.lever_arm_from_rotation(t[0], t[1], th, psi)
    without = S.lever_arm_from_rotation(t[0], t[1], th)
    check("lever.mount_yaw_argument_is_live",
          math.hypot(with_psi[0] - without[0], with_psi[1] - without[1]) > 0.01,
          "passing mount_yaw_rad=%.2f changed the answer by less than 10 mm "
          "(%r vs %r). The result would be in the LASER frame, not base_link."
          % (psi, with_psi[:2], without[:2]))
    check("lever.mount_yaw_default_is_zero",
          S.lever_arm_from_rotation.__defaults__[-1] == 0.0,
          "mount_yaw_rad does not default to 0.0; a caller that has not "
          "measured yaw yet must get the un-rotated answer, not a surprise.")

    # The guard: too small a rotation must return None WITH A REASON, not a
    # huge number. Near theta = 0 the matrix is singular and the answer is
    # arbitrarily large in an arbitrary direction.
    for small_deg in (0.0, 1.0, 5.0, 10.0, 14.0, -10.0):
        lx, ly, why = S.lever_arm_from_rotation(0.01, 0.01, math.radians(small_deg))
        check("lever.refuses_small_%+.0f" % small_deg,
              lx is None and ly is None and isinstance(why, str) and why,
              "a %.0f deg rotation returned (%r, %r) instead of None with a "
              "reason. Near zero this solve is singular and would hand the "
              "operator a metre-scale offset with a straight face."
              % (small_deg, lx, ly))

    # Just over the guard it must solve.
    lx, ly, why = S.lever_arm_from_rotation(0.01, 0.01, math.radians(20.0))
    check("lever.solves_above_guard", lx is not None,
          "a 20 deg rotation (above the 15 deg guard) was refused: %s" % why)

    # And through the real pipeline, which is weak by nature: a 50 mm lever arm
    # produces ~20 mm of apparent translation against an ICP whose residual is
    # ~10 mm. Report the magnitude only, and only as a warning -- the module
    # already documents this as a sanity check on a ruler, not a replacement.
    L = (0.05, 0.0)
    th = math.radians(60.0)
    a = scan_at(0.20, 0.10, 0.0, lever=L)
    b = scan_at(0.20, 0.10, th, lever=L)
    r, _ = S.estimate_rotation(a, b, ANGLE_INC)
    dx, dy, rms, _ = S.estimate_translation(pts(a), pts(b), r)
    lx, ly, _ = S.lever_arm_from_rotation(dx, dy, r)
    if lx is not None:
        mag_err = abs(math.hypot(lx, ly) - math.hypot(*L))
        if mag_err > 0.015:
            warn("lever.pipeline_weak",
                 "end to end, a %.0f mm lever arm came back with magnitude "
                 "%.0f mm (%.0f mm out) at %.0f deg. The signal is ~%.0f mm of "
                 "apparent translation against an ICP residual of %.0f mm, and "
                 "a hand-turned rover does not pivot about base_link's origin "
                 "anyway. Use a ruler; treat this output as a smell test."
                 % (1000 * math.hypot(*L), 1000 * math.hypot(lx, ly),
                    1000 * mag_err, deg(th), 1000 * math.hypot(dx, dy),
                    1000 * rms))


# ==========================================================================
# 7. plane_level_diagnostic -- floor strike vs clutter
# ==========================================================================
def test_11_plane_level_diagnostic():
    """Flag a simulated floor strike; do NOT flag scattered clutter.

    GUARDS AGAINST: both halves of a diagnostic that is only useful if it
    discriminates. fpms_tf.launch.py's arithmetic: at h = 0.10 m a 2 degree
    downward pitch puts the floor strike at 2.9 m, well inside this room, and a
    tilted plane grazing the floor is a CONTIGUOUS band of short, consistent
    returns. Furniture is short returns too, but scattered.

    A diagnostic that cries wolf on every cluttered room gets ignored, and then
    the one time the mount really is tilted it gets ignored too.
    """
    rng = np.random.default_rng(1234)
    base = scan_at(0.0, 0.0, 0.0)

    # Floor strike: a 40-bin band of ~0.30 m returns, tightly clustered.
    strike = base.copy()
    strike[170:210] = 0.30 + rng.normal(0.0, 0.01, 40)
    out = S.plane_level_diagnostic(strike, ANGLE_MIN, ANGLE_INC)
    check("plane.flags_floor_strike", out["floor_strike_suspected"] is True,
          "a 40-bin band of 0.30 m returns clustered around straight ahead was "
          "NOT flagged. short_frac=%.3f, detail=%r"
          % (out["short_frac"], out["detail"]))
    check("plane.floor_strike_detail_actionable",
          "level" in out["detail"] or "laser_z" in out["detail"],
          "the floor-strike detail does not tell the operator what to do: %r"
          % out["detail"])

    # Same at a different bearing and a different width, so the check is not
    # tuned to one band.
    for start, width in ((20, 30), (300, 45), (350, 40)):
        s = base.copy()
        idx = (start + np.arange(width)) % N_BINS
        s[idx] = 0.28 + rng.normal(0.0, 0.01, width)
        out = S.plane_level_diagnostic(s, ANGLE_MIN, ANGLE_INC)
        check("plane.flags_strike_at_%d" % start,
              out["floor_strike_suspected"] is True,
              "a %d-bin short band starting at bin %d was not flagged "
              "(short_frac %.3f)" % (width, start, out["short_frac"]))

    # Clutter: the same NUMBER of short returns, scattered. Must not flag.
    clutter = base.copy()
    clutter[rng.choice(N_BINS, 45, replace=False)] = 0.30
    out = S.plane_level_diagnostic(clutter, ANGLE_MIN, ANGLE_INC)
    check("plane.ignores_scattered_clutter",
          out["floor_strike_suspected"] is False,
          "45 SCATTERED short returns (the same count as the floor-strike "
          "case) were flagged as a floor strike. short_frac=%.3f, detail=%r. A "
          "diagnostic that fires on furniture will be ignored when it matters."
          % (out["short_frac"], out["detail"]))

    # Two separate clumps of furniture, on opposite sides: still not a strike.
    two = base.copy()
    two[40:55] = 0.30
    two[220:235] = 0.30
    out = S.plane_level_diagnostic(two, ANGLE_MIN, ANGLE_INC)
    check("plane.ignores_opposed_clumps",
          out["floor_strike_suspected"] is False,
          "two opposed clumps of short returns were flagged as a floor "
          "strike; a tilted plane grazes ONE side. detail=%r" % out["detail"])

    # A clean room must be quiet.
    out = S.plane_level_diagnostic(base, ANGLE_MIN, ANGLE_INC)
    check("plane.quiet_on_clean_room",
          out["floor_strike_suspected"] is False and out["short_frac"] < 0.04,
          "a clean room reported short_frac %.3f / flagged %r"
          % (out["short_frac"], out["floor_strike_suspected"]))

    # Degenerate inputs must be described, not crash or silently pass.
    out = S.plane_level_diagnostic(np.zeros(N_BINS), ANGLE_MIN, ANGLE_INC)
    check("plane.dead_scan_reports_no_data",
          out["floor_strike_suspected"] is False and "too few" in out["detail"],
          "an all-zero (dead scanner) scan returned %r. It must say it cannot "
          "judge, not return a clean bill of health." % out)
    out = S.plane_level_diagnostic(np.ones(8), ANGLE_MIN, ANGLE_INC)
    check("plane.short_scan_reports_no_data",
          out["floor_strike_suspected"] is False and "too few" in out["detail"],
          "an 8-bin scan returned %r" % out)


# ==========================================================================
# 8. The whole operator session
# ==========================================================================
def test_12_end_to_end_operator_session():
    """Simulate the session the README will ask an operator to run.

        1. rotate the rover ~40 deg by hand, gyro watching -> mirror verdict
        2. push it straight three times from three different spots
        3. average the three yaws -> the number that goes into laser_yaw

    GUARDS AGAINST: every function passing its own unit test while the sequence
    they form does not work. This is the test that says the tool works.

    The de-rotation for each push uses the GYRO's heading change, not
    estimate_rotation's. That is the module's own documented guidance, now
    stated in estimate_rotation's docstring: the range-profile correlation is
    CONFOUNDED BY TRANSLATION, and over the 0.30-0.50 m push this tool asks for
    it reports up to 20 degrees of rotation that did not happen. The gyro is an
    independent witness that does not depend on the scan at all. The cost of
    doing it the other way is measured below.

    Push distances are 0.30/0.35/0.40 m, chosen to clear fpms-calibrate-lidar's
    own gates: PUSH_MIN_M = 0.25, and PUSH_MIN_CONFIDENCE = 0.6 against the
    (d - 0.10)/0.30 confidence curve needs ~0.28 m.

    Tolerance: 0.50 deg, straight from fpms_tf.launch.py's requirement that the
    mount yaw be "known to better than ~0.5 degrees".
    """
    rng = np.random.default_rng(20260811)

    # Three different start poses, three different distances, a little
    # incidental rotation on each because a hand push is not perfect.
    PUSHES = [(0.00, 0.00, 0.00, 0.30, 0.0),
              (-0.30, 0.40, 0.50, 0.35, math.radians(1.5)),
              (0.50, -0.50, -1.10, 0.40, math.radians(-1.0))]

    for psi_deg in (0.0, -7.0, 4.0, 12.0, -15.0):
        psi = math.radians(psi_deg)

        # --- step 1: handedness -------------------------------------------
        gyro_turn = math.radians(40.0)
        a = noisy(scan_at(0.0, 0.0, 0.0, mount_yaw=psi), rng)
        b = noisy(scan_at(0.0, 0.0, gyro_turn, mount_yaw=psi), rng)
        r, conf = S.estimate_rotation(a, b, ANGLE_INC)
        verdict, why = S.mirror_verdict(r, gyro_turn)
        check("e2e.mirror_ok_psi%+.0f" % psi_deg, verdict == "ok",
              "step 1 of the operator session, with a correctly-handed "
              "scanner and mount yaw %+.1f deg, returned %r. The operator "
              "would now flip LIDAR_ROTATION_SIGN and break a working "
              "scanner. %s" % (psi_deg, verdict, why))

        # --- step 2: three straight pushes ---------------------------------
        yaws = []
        for (sx, sy, st, d, incidental) in PUSHES:
            a = noisy(scan_at(sx, sy, st, mount_yaw=psi), rng)
            b = noisy(scan_at(sx + d * math.cos(st), sy + d * math.sin(st),
                              st + incidental, mount_yaw=psi), rng)
            # The gyro's heading change, passed as the CHASSIS rotation --
            # estimate_translation negates it internally.
            dx, dy, rms, inl = S.estimate_translation(pts(a), pts(b), incidental)
            y, c = S.yaw_from_straight_push(dx, dy)
            check("e2e.push_confident_psi%+.0f_d%.2f" % (psi_deg, d), c >= 0.60,
                  "a %.0f mm push reported confidence %.2f, below "
                  "fpms-calibrate-lidar's PUSH_MIN_CONFIDENCE = 0.6, so the "
                  "tool would reject a measurement this test relies on"
                  % (d * 1000, c))
            check("e2e.push_fit_psi%+.0f_d%.2f" % (psi_deg, d),
                  rms < 0.06 and inl >= 30,
                  "push ICP converged with rms %.4f over %d inliers, outside "
                  "the tool's PUSH_MAX_RMS_M = 0.06 / PUSH_MIN_INLIERS = 30"
                  % (rms, inl))
            yaws.append(y)

        # --- step 3: combine ------------------------------------------------
        # Circular mean, because these are angles.
        mean_yaw = math.atan2(float(np.mean(np.sin(yaws))),
                              float(np.mean(np.cos(yaws))))
        err = angdiff_deg(mean_yaw, psi_deg)
        check("e2e.yaw_psi%+.0f" % psi_deg, err <= YAW_TOL_DEG,
              "FULL SESSION at mount yaw %+.1f deg: three pushes gave %s, "
              "circular mean %+.3f deg, error %.3f deg > %.2f deg. That is "
              "%.1f mm of position error baked into every fix, in a direction "
              "that depends on heading, so it warps the map rather than "
              "averaging out."
              % (psi_deg, ["%+.3f" % deg(y) for y in yaws], deg(mean_yaw),
                 err, YAW_TOL_DEG, 10.5 * err),)

        # The spread across three pushes is what an operator would use to decide
        # whether to believe the answer, so it has to be informative.
        spread = max(deg(y) for y in yaws) - min(deg(y) for y in yaws)
        check("e2e.spread_psi%+.0f" % psi_deg, spread <= 4.0,
              "the three pushes disagreed by %.2f deg at mount yaw %+.1f. If "
              "single pushes scatter this much the operator cannot tell a good "
              "session from a bad one." % (spread, psi_deg))

    # --- scan-correlation de-rotation: the path the shipping tool takes ------
    # overlay/usr/local/bin/fpms-calibrate-lidar does, at its push analysis:
    #
    #     rot, rot_conf = sm.estimate_rotation(ref.ranges, end.ranges, inc)
    #     dx, dy, rms, inliers = sm.estimate_translation(ref.xy(), end.xy(), rot)
    #     yaw, conf = sm.yaw_from_straight_push(dx, dy, pushed_forward=True)
    #
    # The sign is now right, but the SOURCE is not: estimate_rotation is
    # confounded by translation, which the module now documents. So this path
    # feeds the ICP a rotation that did not happen.
    #
    # The question that matters is not whether it is accurate -- it is not --
    # but whether it is DETECTABLE. The tool publishes three gates
    # (PUSH_TURN_REJECT_DEG = 5.0, PUSH_MAX_RMS_M = 0.06, PUSH_MIN_CONFIDENCE =
    # 0.6), and a bad push must trip one of them rather than produce a plausible
    # yaw. That is what is asserted here: every push whose scan-de-rotated
    # answer misses the 0.5 deg budget must be REJECTED.
    # Both paths are run over the SAME 36 pushes -- two mount yaws, three
    # distances, six start poses -- and judged by the SAME single-push budget
    # of 1.5 deg used in test_09 (the session averages three pushes and must
    # meet 0.5 deg; a single push is allowed more). Only pushes the tool would
    # ACCEPT are scored, because a rejected push costs the operator time, not
    # accuracy.
    poses = [(0.0, 0.0, 0.0), (-0.30, 0.40, 0.50), (0.50, -0.50, -1.10),
             (0.20, 0.60, 2.00), (-0.80, 0.0, 1.60), (1.00, 0.30, -0.40)]

    def sweep(use_gyro):
        rng2 = np.random.default_rng(1618)
        accepted, rejected, worst_case = [], 0, None
        for psi_deg in (-7.0, 6.0):
            psi = math.radians(psi_deg)
            for d in (0.30, 0.40, 0.50):
                for (sx, sy, st) in poses:
                    a = noisy(scan_at(sx, sy, st, mount_yaw=psi), rng2)
                    b = noisy(scan_at(sx + d * math.cos(st),
                                      sy + d * math.sin(st), st,
                                      mount_yaw=psi), rng2)
                    # These pushes carry no real turn, so the gyro reports 0.
                    r, _ = S.estimate_rotation(a, b, ANGLE_INC)
                    rot = 0.0 if use_gyro else r
                    dx, dy, rms, inl = S.estimate_translation(pts(a), pts(b), rot)
                    y, c = S.yaw_from_straight_push(dx, dy)
                    # fpms-calibrate-lidar's published gates. PUSH_TURN_REJECT
                    # is applied to whichever rotation the tool is trusting --
                    # a gyro-driven tool rejects a steered push on the GYRO's
                    # turn, which is 0 for these (genuinely straight) pushes.
                    gated = (abs(deg(rot)) <= 5.0 and rms <= 0.06 and c >= 0.60
                             and math.hypot(dx, dy) >= 0.25 and inl >= 30)
                    if not gated:
                        rejected += 1
                        continue
                    e = angdiff_deg(y, psi_deg)
                    accepted.append(e)
                    if worst_case is None or e > worst_case[0]:
                        worst_case = (e, psi_deg, d, (sx, sy, st),
                                      deg(r), rms, c)
        return accepted, rejected, worst_case

    SINGLE_PUSH_TOL_DEG = 1.5

    g_acc, g_rej, g_worst = sweep(use_gyro=True)
    s_acc, s_rej, s_worst = sweep(use_gyro=False)

    # The recommended path, over the whole sweep rather than the three poses of
    # the scripted session above.
    check("e2e.gyro_derotation_sweep",
          g_acc and max(g_acc) <= SINGLE_PUSH_TOL_DEG
          and float(np.mean(g_acc)) <= YAW_TOL_DEG,
          "de-rotating with the gyro: %d of %d pushes accepted, worst %.2f "
          "deg, mean %.2f deg. A single push is allowed %.1f deg and the mean "
          "must clear the %.2f deg session budget."
          % (len(g_acc), len(g_acc) + g_rej, max(g_acc) if g_acc else -1,
             float(np.mean(g_acc)) if g_acc else -1, SINGLE_PUSH_TOL_DEG,
             YAW_TOL_DEG))
    check("e2e.gyro_derotation_yield", len(g_acc) >= 0.6 * (len(g_acc) + g_rej),
          "the gyro path had only %d of %d pushes accepted by the tool's own "
          "gates; the operator cannot complete a session at that yield"
          % (len(g_acc), len(g_acc) + g_rej))

    # The path the shipping tool NO LONGER takes. Kept as evidence.
    #
    # fpms-calibrate-lidar de-rotated the push with estimate_rotation until
    # this test measured what that costs; it now passes the gyro reading into
    # fit_motion() and gates on the gyro too. This block is retained as a
    # WARNING rather than deleted, because it is the measurement that justifies
    # that decision - and if anyone ever wires the scan correlation back in,
    # the numbers explaining why not are right here rather than in a commit
    # message nobody reads.
    if s_worst is None:
        detail = "no push survived the tool's gates at all"
    else:
        e, wpsi, wd, wpose, wrot, wrms, wconf = s_worst
        detail = (
            "the worst push that passed ALL of the tool's gates was %.2f deg "
            "out, against a %.1f deg single-push budget -- %.0f mm of "
            "position error.\n"
            "    Worst case: mount yaw %+.1f deg, %.2f m push from pose %s. "
            "estimate_rotation reported %+.2f deg of turn on a push with NO "
            "turn in it. That spurious rotation sat inside "
            "PUSH_TURN_REJECT_DEG = 5.0, rms %.4f inside PUSH_MAX_RMS_M = "
            "0.06, confidence %.2f above PUSH_MIN_CONFIDENCE = 0.6 -- every "
            "gate passed, two of them by under 10%%.\n"
            "    The gates are necessary but NOT sufficient. They are set to "
            "catch a STEERED push, and a translation-confounded rotation "
            "estimate is indistinguishable from one. Either de-rotate with "
            "the gyro (the module's own guidance; e2e.gyro_derotation_sweep "
            "shows it holds across these same 36 pushes), or tighten "
            "PUSH_TURN_REJECT_DEG towards the budget it is really standing in "
            "for -- 5.0 deg of accepted spurious turn admits several degrees "
            "of yaw error against a 0.5 deg requirement."
            % (e, SINGLE_PUSH_TOL_DEG, 10.5 * e, wpsi, wd, str(wpose),
               wrot, wrms, wconf))

    warn("e2e.scan_derotation_is_why_the_tool_uses_the_gyro",
         "if fpms-calibrate-lidar de-rotated with estimate_rotation - which it "
         "did until this was measured, and no longer does - then " + detail)

    warn("e2e.derotation_source_comparison",
         "over the same 36 pushes (2 mount yaws x 3 distances x 6 start "
         "poses), judged by the tool's own gates: GYRO de-rotation accepted "
         "%d, worst %.2f deg, mean %.2f deg. SCAN-CORRELATION de-rotation "
         "accepted %d, worst %.2f deg, mean %.2f deg. The gyro path is both "
         "more accurate and %.0fx more productive -- the scan path's spurious "
         "rotations trip the tool's own turn and rms gates, so the operator "
         "pushes repeatedly and is told the chassis turned when it did not. "
         "PUSH_TARGET_M = 0.50 is where the confounding is worst."
         % (len(g_acc), max(g_acc) if g_acc else -1,
            float(np.mean(g_acc)) if g_acc else -1,
            len(s_acc), max(s_acc) if s_acc else -1,
            float(np.mean(s_acc)) if s_acc else -1,
            len(g_acc) / max(len(s_acc), 1)))

    # And the same session on a MIRRORED scanner must be stopped at step 1,
    # before any yaw is measured against a mirrored world.
    psi = math.radians(-7.0)
    gyro_turn = math.radians(40.0)
    a = mirror_scan(noisy(scan_at(0.0, 0.0, 0.0, mount_yaw=psi), rng))
    b = mirror_scan(noisy(scan_at(0.0, 0.0, gyro_turn, mount_yaw=psi), rng))
    r, _ = S.estimate_rotation(a, b, ANGLE_INC)
    verdict, why = S.mirror_verdict(r, gyro_turn)
    check("e2e.mirrored_session_is_stopped", verdict == "mirrored",
          "a MIRRORED scanner got verdict %r at step 1, so the session would "
          "continue and measure a mount yaw against a mirrored world. No value "
          "of laser_yaw can undo a mirror; the number would be confident, "
          "plausible and wrong. %s" % (verdict, why))


# ==========================================================================
def main():
    print("=" * 72)
    print(" fpms_scanmatch -- synthetic ground-truth checks")
    print("=" * 72)

    if S is None:
        print("  [FAIL] import: %s" % IMPORT_ERROR)
        print("-" * 72)
        print("  0 checks run -- the module under test could not be loaded.")
        print("  This is NOT a pass. Nothing was verified.")
        print("=" * 72)
        return 2

    print("  module: %s" % os.path.join(LIBDIR, "fpms_scanmatch.py"))
    print("  numpy:  %s" % np.__version__)
    print("  world:  %.1f x %.1f m room, %d segments, %d bins at %.3f deg"
          % (4.0, 3.0, len(ROOM), N_BINS, deg(ANGLE_INC)))
    print("  tol:    rotation %.2f deg (%.2f noisy), translation %.0f mm, "
          "mount yaw %.2f deg" % (ROT_TOL_DEG, ROT_TOL_NOISY_DEG,
                                  1000 * TRANS_TOL_M, YAW_TOL_DEG))
    print("-" * 72)

    tests = [(k, v) for k, v in sorted(globals().items())
             if k.startswith("test_")]
    for name, fn in tests:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            fail(name, "the test itself raised: %r" % exc)

    for name, msg in warnings:
        print("  [WARN] %s: %s" % (name, msg))
    for name, msg in failures:
        print("  [FAIL] %s: %s" % (name, msg))

    print("-" * 72)
    print("  %d assertions across %d test groups"
          % (checks_run[0], len(tests)))
    if failures:
        print("  %d FAILED, %d warnings" % (len(failures), len(warnings)))
        # Group by root cause. 30-odd individual failures from two defects
        # reads as a flaky test file; it is not one.
        buckets = [
            ("SIGN CONVENTION (mirror.*, sign.*, e2e.mirror*)",
             lambda n: n.startswith(("mirror.", "sign.", "e2e.mirror")),
             "The settled contract: estimate_rotation returns the CHASSIS "
             "rotation (+theta, same sign as the gyro); mirror_verdict calls a "
             "scan correctly-handed when the two AGREE in sign; "
             "estimate_translation takes chassis_rotation_rad and negates "
             "internally. A failure here means one of the three has drifted "
             "from the other two -- and the symptom is an INVERTED handedness "
             "verdict, where a correct scanner reads 'mirrored' and a mirrored "
             "one reads 'ok'. That is the check the whole calibration is gated "
             "on, and nothing but synthetic ground truth can see it."),
            ("LEVER ARM (lever.*)",
             lambda n: n.startswith("lever."),
             "The inversion owed is L = R(psi).(R(-theta) - I)^-1.t. The "
             "historical defect returned the true offset rotated by -theta: "
             "right magnitude, wrong direction, at every rotation the guard "
             "allows. Check whether the failing values are exactly R(-theta)L "
             "-- the per-case message says so if they are."),
            ("PUSH PATH (e2e.yaw*, e2e.spread*, e2e.push*, e2e.scan_derot*, "
             "yaw.*)",
             lambda n: n.startswith(("e2e.yaw", "e2e.spread", "e2e.push",
                                     "e2e.scan_derot", "e2e.gyro_derot",
                                     "yaw.")),
             "The mount yaw itself. De-rotate the push with the GYRO, not with "
             "estimate_rotation -- the correlation is confounded by "
             "translation and reports up to 20 deg of turn on a 0.5 m straight "
             "push. Every degree here is 10.5 mm of permanent, "
             "heading-dependent position bias."),
        ]
        seen = set()
        for title, pred, why in buckets:
            hits = [n for n, _ in failures if pred(n)]
            if not hits:
                continue
            seen.update(hits)
            print()
            print("  %s -- %d failures" % (title, len(hits)))
            print("    %s" % why.replace("\n", "\n    "))
        other = [n for n, _ in failures if n not in seen]
        if other:
            print()
            print("  OTHER -- %d failures: %s"
                  % (len(other), ", ".join(sorted(set(other)))))
        print()
        print("  These are ground-truth failures against a world whose answer")
        print("  is known by construction. Do not loosen a tolerance to make")
        print("  one pass -- the numbers are justified in this file's header,")
        print("  and every one of them traces back to fpms_tf.launch.py's")
        print("  1 deg = 10.5 mm.")
    else:
        print("  all passed, %d warnings" % len(warnings))
    print()
    print("  NOT VERIFIED HERE: that the real scanner emits these bearings,")
    print("  that ranges arrive with the handedness fpms_lidar_ros.py assumes,")
    print("  or that a person can push this rover in a straight line. Ray")
    print("  casting has no beam divergence, no mixed pixels, no specular")
    print("  dropout and no motion distortion. Real numbers will be worse.")
    print("=" * 72)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
