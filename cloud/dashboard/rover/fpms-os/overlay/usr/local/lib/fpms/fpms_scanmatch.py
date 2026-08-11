"""fpms_scanmatch - 2D scan geometry for LiDAR mount calibration.

Pure numpy. No ROS, no hardware, no I/O. Everything here is a function from
arrays to numbers so it can be tested against synthetic scans on any machine,
which is the only way any of this gets verified before the board arrives.

WHY THIS EXISTS
===============
`base_link -> laser_frame` in fpms_tf.launch.py is six MEASURE ME placeholders,
and the unit launches it with all of them at zero. By that file's own
arithmetic **1 degree of mount yaw is about 10.5 mm of position error** - and
both the obstacle cone guard and the occupancy grid inherit it in the same
direction, so it accumulates over a mission rather than averaging out. Nav2 and
both SLAM units refuse to start because of it.

Separately, `LIDAR_ROTATION_SIGN = -1` in fpms_lidar_ros.py is marked
UNVERIFIED. If it is wrong the scan is MIRRORED, and a mirror is not a rigid
transform: no value of laser_yaw can undo it. That has to be settled first,
because measuring yaw against a mirrored world gives a confident wrong answer.

THE IDEA
========
The operator moves the rover BY HAND and the tool only watches - the same
contract as fpms_charact.py, which publishes to no actuation topic at all. That
buys three things:

  * no motion calibration is needed. Every estimate below depends on the
    DIRECTION of apparent motion, never its magnitude, so counts/mm being
    unmeasured does not matter;
  * no safety surface. Nothing here can move the rover;
  * the gyro is an independent witness. Heading comes from integrating
    angular_velocity.z, which does not depend on wheel calibration either.

WHAT IS OBSERVABLE FROM A 2D SCAN, AND WHAT IS NOT
==================================================
  mirror        YES  - rotate in place; a mirrored scan turns the wrong way
  yaw           YES  - push straight; static features stream past at an angle
                       that IS the mount yaw
  x, y          PARTLY - rotate in place; an off-centre scanner swings on a
                       lever arm. Weakly observed and reported with a caveat
  z             NO   - a level 2D scan plane carries no height information.
                       Use a ruler; it is one unambiguous measurement
  roll, pitch   NO   - only detectable as a floor strike or a wall whose range
                       varies with bearing. Diagnosed, never estimated
"""

from __future__ import annotations

import math

import numpy as np

# --------------------------------------------------------------------------
# scan -> points
# --------------------------------------------------------------------------


# Defaults taken from fpms_lidar_ros.py, NOT invented here. That driver uses
# RANGE_MIN_M = 0.12, RANGE_MAX_M = 6.0 and RANGE_CLAMP_M = 6.0, and it is
# what fills every scan this module will ever see. Wider defaults let two
# classes of junk through: returns in the 0.05-0.12 m band that the driver
# itself calls invalid, and CLAMPED 6.0 m returns - a "nothing out there"
# reading that would otherwise be matched as though it were a real surface
# at exactly 6 m, in every direction at once.
SCAN_RANGE_MIN_M = 0.12
SCAN_RANGE_MAX_M = 5.95   # just inside the clamp, so clamped returns drop

def scan_to_xy(ranges, angle_min, angle_increment,
               range_min=SCAN_RANGE_MIN_M, range_max=SCAN_RANGE_MAX_M):
    """Polar LaserScan ranges -> (N,2) Cartesian points in the LASER frame.

    Drops non-finite and out-of-band returns. That matters more than it looks:
    the rover's agent publishes ON A TIMER rather than on data, so a dead
    scanner still produces a full-length message whose ranges are all 0.0 -
    and 0.0 converts to `inf`, "no return", which reads as a completely clear
    360 degrees. Anything downstream that trusted such a scan would be
    confidently wrong rather than merely blind, so the filtering here is the
    same refusal the rest of the stack makes.
    """
    r = np.asarray(ranges, dtype=float)
    n = r.shape[0]
    if n == 0:
        return np.empty((0, 2), dtype=float)
    a = angle_min + angle_increment * np.arange(n, dtype=float)
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    r = r[ok]
    a = a[ok]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def wrap_pi(x):
    """Wrap to (-pi, pi]. Used everywhere an angle is differenced."""
    return (np.asarray(x, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


# --------------------------------------------------------------------------
# rotation, by angular correlation
# --------------------------------------------------------------------------


def estimate_rotation(ranges_a, ranges_b, angle_increment, max_shift_bins=None):
    """Apparent rotation from scan A to scan B, in radians.

    Correlates the two range profiles against each other as a function of
    angular shift, which for a full 360-degree scan is both cheap and very
    robust: it uses every bin, needs no correspondences, and does not care
    that some returns dropped out between the two frames.

    Deliberately NOT an ICP rotation. ICP needs an initial guess and this is
    the step that has to work when the scan might be MIRRORED - i.e. when any
    initial guess would be wrong by construction.

    Returns (radians, confidence 0..1). Confidence is the correlation peak's
    prominence over the background, so a featureless corridor - where rotation
    genuinely is not observable - reports low rather than inventing a number.

    IT IS CONFOUNDED BY TRANSLATION, and the caller must know this. Measured on
    synthetic scans: a 0.25-0.35 m straight push through a 4x3 m room reports
    up to 7.4 degrees of rotation that did not happen, because near features
    sweep past faster than far ones and the correlation cannot tell that from a
    turn. So for the straight-push estimate, DE-ROTATE WITH THE GYRO, not with
    this. The gyro is an independent witness that does not depend on the scan
    at all, which is the whole reason it is used to settle handedness.
    """
    a = np.asarray(ranges_a, dtype=float)
    b = np.asarray(ranges_b, dtype=float)
    n = min(a.shape[0], b.shape[0])
    if n < 16:
        return 0.0, 0.0
    a = a[:n].copy()
    b = b[:n].copy()

    # Non-returns become the median rather than inf/0, so they neither dominate
    # the correlation nor punch holes in it.
    for v in (a, b):
        bad = ~np.isfinite(v) | (v <= 0.0)
        if bad.all():
            return 0.0, 0.0
        v[bad] = np.median(v[~bad])

    a = a - a.mean()
    b = b - b.mean()
    denom = math.sqrt(float(np.dot(a, a)) * float(np.dot(b, b)))
    if denom <= 0.0:
        return 0.0, 0.0

    # Circular cross-correlation via FFT: O(n log n) and exact for a wrap-around
    # scan, which a sliding window is not.
    corr = np.real(np.fft.ifft(np.fft.fft(a) * np.conj(np.fft.fft(b)))) / denom

    if max_shift_bins is not None and max_shift_bins < n // 2:
        mask = np.zeros(n, dtype=bool)
        k = int(max_shift_bins)
        mask[:k + 1] = True
        mask[n - k:] = True
        corr = np.where(mask, corr, -np.inf)

    k = int(np.argmax(corr))
    peak = float(corr[k])

    # Sub-bin refinement by a parabola through the peak and its neighbours.
    # One bin is ~1 degree on this scanner, which is ~10 mm of position error
    # at the arithmetic quoted above - worth interpolating for.
    y0 = float(corr[(k - 1) % n])
    y2 = float(corr[(k + 1) % n])
    denom2 = (y0 - 2.0 * peak + y2)
    delta = 0.0 if denom2 == 0.0 else 0.5 * (y0 - y2) / denom2
    if not np.isfinite(delta) or abs(delta) > 1.0:
        delta = 0.0

    shift = k + delta
    if shift > n / 2.0:
        shift -= n

    # SIGN. Read this before touching it; it was wrong once and the failure was
    # not visible from either caller.
    #
    # The FFT form above, ifft(fft(a) * conj(fft(b))), peaks at the k where
    # b[m] == a[m + k]. For a chassis that turns +theta (CCW), a feature at
    # bearing beta in A reappears at beta - theta in B, so b[m] = a[m + theta/inc]
    # and the raw peak is at k = +theta/inc -- i.e. the CHASSIS rotation, the
    # same sign the gyro reports.
    #
    # Both consumers want the other one, the APPARENT rotation of the point
    # cloud, which is what "rotation from scan A to scan B" means and what this
    # docstring promises:
    #
    #   estimate_translation() de-rotates A by this value to bring it into B's
    #   orientation. The cloud turned by -theta, so it needs -theta. Fed +theta
    #   it reported ~310 mm of travel for a rover rotating in place.
    #
    #   mirror_verdict() calls a scan correctly-handed when this and the gyro
    #   have OPPOSITE signs. Returning the chassis rotation made them agree on
    #   correct data, so the verdict was exactly inverted: good scans read
    #   "mirrored" and mirrored scans read "ok". That is the one check the whole
    #   calibration is gated on.
    #
    # RESOLVED THE OTHER WAY: this function keeps returning the CHASSIS
    # rotation, because that is the more useful contract - it is directly
    # comparable to the gyro, which is the independent witness this whole
    # module leans on, and a caller reading "the rover turned 40 degrees" is
    # not surprised. The two consumers were changed instead:
    #
    #   mirror_verdict()      now calls a scan correctly-handed when this and
    #                         the gyro AGREE in sign.
    #   estimate_translation() now takes the chassis rotation and negates it
    #                         internally to de-rotate the cloud.
    #
    # Both are documented at their own definitions. Do not "fix" the sign here
    # without changing both of them in the same commit.
    radians = wrap_pi(shift * angle_increment)

    finite = corr[np.isfinite(corr)]
    background = float(np.mean(np.abs(finite))) if finite.size else 0.0
    confidence = 0.0 if peak <= 0 else max(0.0, min(1.0, peak - background))
    return float(radians), confidence


# --------------------------------------------------------------------------
# translation, by ICP after de-rotation
# --------------------------------------------------------------------------


def estimate_translation(pts_a, pts_b, chassis_rotation_rad, iterations=25,
                         trim_sigma=3.0):
    """Translation from A to B (metres, LASER frame), given the rotation.

    Point-to-point ICP with trimmed correspondences. The worst-matched pairs
    are dropped each iteration, which is what makes this survive the real
    failure here: between two scans a metre apart, a
    substantial fraction of returns have no counterpart at all because
    something came into or went out of view.

    The trim is ADAPTIVE (median + trim_sigma * MAD), not a fixed fraction.
    A fixed budget is wrong at both ends and both ends were measured: at full
    overlap, discarding a quarter of good correspondences costs accuracy for
    nothing, and at 30% occlusion a quarter is not enough to remove the
    non-overlapping tail, so the error jumps to ~15% of the push length with no
    outward sign. A robust spread adapts to whatever the scene actually gives.

    THE RMS IS THE TELL. A clean match sits near 0.008; the 30%-occluded case
    sat at 0.060. Callers should refuse on a high rms rather than accept a
    number, which is what makes this recoverable instead of quietly wrong.

    Returns (dx, dy, rms_residual_m, inlier_count).
    """
    A = np.asarray(pts_a, dtype=float)
    B = np.asarray(pts_b, dtype=float)
    if A.shape[0] < 8 or B.shape[0] < 8:
        return 0.0, 0.0, float("inf"), 0

    # NEGATED. The argument is the CHASSIS rotation (what estimate_rotation
    # returns, same sign as the gyro); the POINT CLOUD turned the other way, so
    # bringing A into B's orientation needs -theta. Passing +theta here
    # reported ~310 mm of travel for a rover rotating in place, with a
    # plausible residual and no error anywhere.
    th = -float(chassis_rotation_rad)
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    A = A @ R.T                     # de-rotate A into B's orientation

    t = np.zeros(2, dtype=float)
    rms = float("inf")
    keep = 0

    for _ in range(int(iterations)):
        P = A + t
        # Nearest neighbour by brute force. A 360-bin scan is a few hundred
        # points, so this is microseconds and needs no scipy - which is not
        # installed on the rover and which this project has been bitten by
        # adding before.
        d2 = ((P[:, None, :] - B[None, :, :]) ** 2).sum(axis=2)
        idx = np.argmin(d2, axis=1)
        dist = np.sqrt(d2[np.arange(P.shape[0]), idx])

        # ADAPTIVE TRIM, not a fixed fraction. A fixed reject_frac is wrong at
        # both ends: at full overlap it throws away good correspondences and
        # costs accuracy, and at 30% occlusion it cannot remove all the bad
        # ones and the error jumps to ~15% of the push length, silently. Both
        # were measured against synthetic scans with a known answer.
        #
        # A median-absolute-deviation cut adapts: with clean data the spread is
        # tiny so almost nothing is dropped, and with a large non-overlapping
        # tail that tail sits far outside the MAD and goes.
        med = float(np.median(dist))
        mad = float(np.median(np.abs(dist - med))) * 1.4826
        cut = med + trim_sigma * max(mad, 1e-4)
        m = dist <= cut
        if m.sum() < 8:
            break

        step = (B[idx][m] - P[m]).mean(axis=0)
        t = t + step
        rms = float(np.sqrt(np.mean(dist[m] ** 2)))
        keep = int(m.sum())
        if np.linalg.norm(step) < 1e-6:
            break

    return float(t[0]), float(t[1]), rms, keep


# --------------------------------------------------------------------------
# the three estimates the operator actually cares about
# --------------------------------------------------------------------------


def mirror_verdict(scan_rotation_rad, gyro_delta_rad, min_gyro_rad=0.26):
    """Is the scan mirrored? Compare its turn against the gyro's.

    estimate_rotation() reports the CHASSIS rotation - it has already undone
    the fact that the world appears to turn the other way. So on a
    correctly-handed scan it AGREES in sign with the gyro. If the two come back
    with OPPOSITE signs, the scan is mirrored and LIDAR_ROTATION_SIGN must flip.

    (This read the other way round until synthetic ground truth caught it, and
    the failure was invisible from either side: the verdict was exactly
    inverted, so a correct scan reported 'mirrored' and a mirrored one reported
    'ok'. It is the single check the rest of the calibration is gated on.)

    This has to be settled before anything else. A mirror is not a rigid
    transform - no value of laser_yaw can undo it - so measuring yaw against a
    mirrored world produces a confident, plausible, wrong number.

    min_gyro_rad (~15 degrees) exists because near zero the signs are noise.
    Returns (verdict, detail) where verdict is
    'ok' | 'mirrored' | 'inconclusive'.
    """
    g = float(gyro_delta_rad)
    s = float(scan_rotation_rad)
    if abs(g) < min_gyro_rad:
        return "inconclusive", (
            "the rover turned %.1f deg; turn it at least %.0f deg so the sign "
            "is not noise" % (math.degrees(g), math.degrees(min_gyro_rad)))
    if abs(s) < min_gyro_rad * 0.4:
        return "inconclusive", (
            "the gyro says %.1f deg but the scan barely moved (%.1f deg). "
            "Either the scan is stale or the scanner is not seeing structure."
            % (math.degrees(g), math.degrees(s)))

    ratio = abs(s) / abs(g)
    if not (0.55 < ratio < 1.8):
        return "inconclusive", (
            "scan turned %.1f deg while the gyro says %.1f deg (ratio %.2f). "
            "Too far apart to judge handedness - suspect gyro_scale, or the "
            "rover translated as well as turned."
            % (math.degrees(s), math.degrees(g), ratio))

    if s * g > 0.0:
        return "ok", ("scan turned %.1f deg with a gyro %.1f deg - same sign, "
                      "which is correct." % (math.degrees(s), math.degrees(g)))
    return "mirrored", (
        "scan turned %.1f deg while the gyro says %.1f deg - OPPOSITE signs. "
        "The scan is mirrored: flip LIDAR_ROTATION_SIGN in fpms_lidar_ros.py. "
        "Do this before measuring yaw - a mirror is not a rigid transform, so "
        "no value of laser_yaw can undo it and any yaw measured now would be "
        "confidently wrong." % (math.degrees(s), math.degrees(g)))


def yaw_from_straight_push(dx, dy, pushed_forward=True):
    """Mount yaw (radians) from the apparent motion of a straight push.

    Push the chassis along base_link +x by some distance d. In the LASER frame
    every static point appears to move by -d along the direction that base_link
    +x points in that frame. base_link +x expressed in the laser frame lies at
    angle -psi, where psi is the mount yaw. So the apparent motion sits at
    (-psi + pi), and

        psi = pi - atan2(dy, dx)          (wrapped)

    The MAGNITUDE of d never enters. That is the point: counts/mm on this rover
    is unmeasured and disputed, and this estimate does not care.

    Returns (yaw_rad, confidence 0..1) where confidence falls off as the push
    gets short - a 20 mm nudge has a direction dominated by scan noise.
    """
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return 0.0, 0.0
    heading = math.atan2(dy, dx)
    if not pushed_forward:
        heading = wrap_pi(heading + math.pi)
    yaw = float(wrap_pi(math.pi - heading))
    # A short push has a direction dominated by scan noise, and the operator is
    # told to push ~0.5 m. Full confidence therefore needs ~0.4 m; a 200 mm
    # nudge scores ~0.33, which is honest rather than encouraging. Measured:
    # short pushes were the largest contributor to spread between repeats.
    confidence = max(0.0, min(1.0, (d - 0.10) / 0.30))
    return yaw, float(confidence)


def lever_arm_from_rotation(dx, dy, rotation_rad, mount_yaw_rad=0.0):
    """Scanner offset (lx, ly) in base_link, from a pure rotation in place.

    GEOMETRY, stated carefully, because the first version of this was wrong in
    a way that passed every plausibility check an operator could apply: it
    returned the right MAGNITUDE rotated by R(-theta), so the number looked
    entirely reasonable and pointed somewhere else.

    Let the chassis turn by theta about base_link's origin, with the scanner at
    offset L in base_link and mount yaw psi. In the LASER frame the apparent
    translation of the static world is

        t = (R(-theta) - I) . R(-psi) . L

    so the inversion owed is

        L = R(psi) . (R(-theta) - I)^-1 . t

    Note (R(-theta) - I), not (R(theta) - I): the point cloud turns the other
    way from the chassis, the same sign trap as estimate_rotation(). And the
    R(psi) is not optional unless the mount yaw happens to be zero, which is
    the thing we are calibrating, so measure yaw FIRST and pass it in.

    The old form omitted both, and the error vanishes only as theta -> 0, which
    is exactly where the 15 degree guard below refuses to run - so there was no
    rotation at which it was both usable and correct.

    Singular at theta = 0 and ill-conditioned near it, hence the guard.

    WEAKLY OBSERVED, and reported as such. It also assumes the rotation really
    was about base_link's origin - a hand-turned rover pivots wherever it
    happens to pivot, which is exactly the assumption most likely to be wrong.
    Treat the output as a sanity check on a ruler, not a replacement for one.
    """
    th = float(rotation_rad)
    if abs(th) < 0.26:
        return None, None, "rotation too small (<15 deg) to solve a lever arm"

    # (R(-theta) - I)
    c, s = math.cos(-th), math.sin(-th)
    M = np.array([[c - 1.0, -s], [s, c - 1.0]])
    det = float(np.linalg.det(M))
    if abs(det) < 1e-9:
        return None, None, "geometry is singular at this rotation"

    t = np.array([float(dx), float(dy)], dtype=float)
    L_laser = np.linalg.solve(M, t)

    # Back into base_link: R(psi) . L_laser
    cp, sp = math.cos(float(mount_yaw_rad)), math.sin(float(mount_yaw_rad))
    L = np.array([[cp, -sp], [sp, cp]]) @ L_laser
    return float(L[0]), float(L[1]), None


def plane_level_diagnostic(ranges, angle_min, angle_increment,
                           range_min=SCAN_RANGE_MIN_M,
                           range_max=SCAN_RANGE_MAX_M):
    """Cheap roll/pitch smell test. Diagnoses; never estimates.

    A level scan plane sweeping a room of vertical surfaces produces ranges
    that vary smoothly with bearing. A TILTED plane strikes the floor on one
    side, which shows up as a band of suspiciously short, suspiciously
    consistent returns clustered in bearing.

    Returns a dict. It cannot separate roll from pitch, and it cannot give an
    angle - a 2D scanner has no height information. Measure z with a ruler and
    level the mount by eye; this only tells you whether to go and look.
    """
    r = np.asarray(ranges, dtype=float)
    n = r.shape[0]
    out = {"floor_strike_suspected": False, "detail": "", "short_frac": 0.0}
    if n < 32:
        out["detail"] = "too few bins to judge"
        return out

    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    if ok.sum() < 32:
        out["detail"] = "too few valid returns to judge"
        return out

    valid = r[ok]
    med = float(np.median(valid))
    short = ok & (r < 0.35 * med)
    frac = float(short.sum()) / float(ok.sum())
    out["short_frac"] = frac

    if frac < 0.04:
        out["detail"] = ("no floor strike apparent (%.1f%% of returns are much "
                         "shorter than the median %.2f m)" % (100.0 * frac, med))
        return out

    # Clustered in bearing => a surface the plane is grazing. Scattered => just
    # furniture, which is not a mount problem.
    idx = np.nonzero(short)[0]
    ang = angle_min + angle_increment * idx
    cx, cy = np.cos(ang).mean(), np.sin(ang).mean()
    concentration = float(math.hypot(cx, cy))
    if concentration > 0.6:
        out["floor_strike_suspected"] = True
        out["detail"] = (
            "%.1f%% of returns are under %.2f m and they cluster around bearing "
            "%.0f deg (concentration %.2f). That is what a tilted scan plane "
            "grazing the floor looks like. Check the mount is level and that "
            "laser_z is right; neither is solvable from a 2D scan."
            % (100.0 * frac, 0.35 * med, math.degrees(math.atan2(cy, cx)),
               concentration))
    else:
        out["detail"] = (
            "%.1f%% short returns but spread around the scan (concentration "
            "%.2f) - looks like clutter, not a floor strike."
            % (100.0 * frac, concentration))
    return out
