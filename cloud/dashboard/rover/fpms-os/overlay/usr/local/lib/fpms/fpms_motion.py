"""fpms_motion - continuous closed-loop motion math for FPMS firmware v3.

Pure functions and frozen state objects. NO ROS, NO serial, NO clock, NO
globals, NO file I/O. Time is always a parameter and every controller is a
`step(state, t, measurement) -> (new_state, command)` transform, so a whole
trajectory can be replayed offline against a synthetic plant. That is the only
way any of this gets verified before a rover is available, and it is the same
contract fpms_scanmatch.py already keeps in this directory.

WHY THIS EXISTS
===============
`fpms_missions.py` drives with burst -> STOP -> settle -> measure -> correct.
That is not a stylistic choice and it is not timidity. On the Yahboom FACTORY
firmware, `PWM_MOTOR_DEAD_ZONE` (200 of a 400-tick scale) is added as
feed-forward to every velocity command, so ANY non-zero cmd_vel becomes ~50 %
duty ~= 0.65 m/s. Amplitude is discarded. The smallest expressible move is
therefore ~227 mm and the smallest expressible non-zero speed is ~0.65 m/s:

    slow motion on factory firmware is not hard, it is UNREPRESENTABLE.

Given that, the ONLY remaining control variable is how long the wheels are on
and how often they are off, which is exactly what the burst loop manipulates.
Every constant around it - MIN_PULSE_S, MIN_MOVE_MM, DOCK_STEP_MM, STOP_SETTLE_S
- is a consequence of that one firmware defect, not a chassis property.

Firmware v3 (`firmware_v3/fpms_config.h`, section 1) removes the dead zone,
makes `/cmd_duty` the primary actuation path (`data[4]`, -100..100 PERCENT,
order [front-left, front-right, rear-left, rear-right]), publishes RAW per-wheel
ticks at 25 Hz, integrates `/odom_raw` with the CORRECT sign, and carries a real
ICM42670P gyro. On that firmware a duty of 8 % really is 8 %, so a speed can be
commanded, held, and trimmed - and a continuous profiled move becomes possible
for the first time on this rover.

    THIS MODULE THEREFORE TARGETS FIRMWARE v3 AND /cmd_duty ONLY.

It REFUSES to produce a command on factory firmware rather than emit motion it
cannot control (`require_firmware_v3`). A controller that "degrades" onto a
50 % dead zone does not degrade; it lurches, and every number it reports about
the lurch is a fiction.

WHAT THIS MODULE IS NOT
=======================
  * It is NOT a ROS node. It publishes nothing, subscribes to nothing, and
    cannot move a rover on its own. The node and the executor integration are
    deliberately somebody else's file.
  * It does NOT own safety. The obstacle cone, the arm/consent gate, the stop
    path, the deadman and HARD_MAX_DUTY at the wire all live upstream and
    downstream of this. A controller that believes it is the safety layer is
    the failure mode this project already survived once.
  * It does NOT replace burst-stop-settle everywhere. Below a floor set by
    `min_moving_duty` a move genuinely cannot be profiled (`_Trapezoid.is_burst`)
    and the honest answer is a bounded pulse plus a measurement, which is what
    the executor already does well.

THE FACTS THIS FILE IS BUILT AROUND (all recorded in the repo, all load-bearing)
===============================================================================
  1. TRUST POSE / TICKS, NEVER TWIST. `/odom_raw`'s `twist.linear.x` has an
     inverted sign relative to its own `pose.position` (measured wheels-off:
     commanded +0.012 -> twist -0.842, pose +1.395; docs/FAILURE_MODES.md C5).
     A guard comparing a commanded sign against reported twist aborts every
     CORRECT move - that is what killed the first deadband sweep and produced a
     retracted "290 mm backward lurch" that was a 290 mm forward move.
     ** No function in this module accepts a twist. There is nowhere to pass
     one. ** Linear feedback comes from `/wheel_ticks`; heading from the gyro.
  2. A HEALTHY GYRO READS EXACTLY 0.0 WHILE PARKED. There is a +/-0.01 rad/s
     deadband (docs/CALIBRATION.md). Zero is not a fault and it is not a stuck
     sensor. `GYRO_PARKED_DEADBAND_RADPS` exists so the settle check treats it
     as stillness instead of as evidence of anything. Relatedly: never command
     a turn to test whether the rover is alive. It destroys the heading the
     mission depends on and it proves nothing a passive pose check cannot.
  3. counts_per_mm ~= 5.5 is MEASURED (hand push forward 5.68, backward 5.47,
     agreeing within 4 %) but on a chassis whose front-left hub was working
     loose - that wheel later came off. THE 2.7x CORRECTION IS ROBUST; THE
     THIRD DIGIT IS NOT. It is a parameter here, never a literal. Note also
     that `fpms_duty_driver.py:114` still carries `TKMM = pi*70/1320`, which
     IMPLIES 6.00 counts/mm - 9 % away from the 5.5 that SPEC.md records as
     measured. Anything that trusts that header overshoots by 9 %. This module
     reads the number from a calibration profile or refuses.
  7. A MEASUREMENT IS ONLY VALID IN THE REGIME IT WAS TAKEN IN. The per-wheel
     trims {0.820, 0.839, 0.914, 1.000}, the kick-start (duty 140 for 220 ms)
     and the coast compensation (1270 fwd / 1448 rev) are widely quoted in this
     repo as golden B8B constants - including by GOLDEN_B8B_STUDY.md. THEY ARE
     NOT. A grep of all ten files in `golden_backup/` returns zero hits for any
     of them; they come from `firmware_linorobot/moves_main.cpp:28-42`, which
     drives at BASE_DUTY 70 with a kick to 140. B8B crawls at duty 26 with no
     kick and no trims. Friction, stiction and the motor curve are all
     nonlinear across a 3x span of duty, so a trim measured at 70 says nothing
     about a trim at 20. NONE of those numbers is hardcoded anywhere in this
     file, and none should be: trims arrive from a calibration profile or
     default to unity.
  4. gyro_scale is UNMEASURED. Turn accuracy rests entirely on it, so
     `TurnController` REFUSES without it. Heading HOLD does not need it (see
     `HeadingHold`: regulating an error to zero is invariant to the scale of
     the sensor), so a straight line can still be held straight.
  5. CMD_SCALE = 6.1 is a DEFECT ARTEFACT of a saturated dead-zone path, not a
     calibration. It appears nowhere in this file and must never be carried in.
  6. Four wheels disagreeing is a MECHANICAL fact before it is a numerical one.
     `travel_mm_from_ticks` takes the MEDIAN of the four wheel deltas and
     reports the spread, because averaging is exactly how three separate
     per-wheel anomalies were each explained away shortly before a wheel came
     off.

MISSING NUMBERS ARE REFUSALS, NOT DEFAULTS
==========================================
Every physical constant this controller needs is Optional and starts as None.
`Calibration.require()` raises `MotionRefusal` naming every missing key at
once. There is no `except: pass` in this file and no plausible substitute for
a measurement anywhere in it. A guessed calibration is silent and total: it
changes every distance and every heading, and nothing about the resulting
behaviour tells you the numbers were invented.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Optional, Sequence, Tuple

__all__ = [
    # refusals and configuration
    "MotionRefusal", "Calibration", "Limits", "Health", "BreakawayKick",
    "NO_KICK", "DEFAULT_LIMITS",
    # gates
    "require_firmware_v3", "require_gyro",
    # measurement
    "travel_mm_from_ticks", "TravelSample", "GyroState", "integrate_gyro",
    # profiles
    "TrapezoidalProfile", "AngularProfile", "ProfilePoint",
    # controllers
    "DistanceController", "DistanceState", "DistanceOutput",
    "HeadingHold", "HeadingState", "HeadingOutput",
    "TurnController", "TurnState", "TurnOutput",
    "StraightRun", "StraightRunState", "StraightRunOutput",
    "TurnInPlace", "TurnInPlaceState", "TurnInPlaceOutput",
    # actuation
    "mix_duty", "stop_command", "DutyCommand",
    # primitives
    "clamp", "wrap_pi", "integrate_bounded", "rate_limit",
    # constants worth asserting against
    "HARD_MAX_DUTY", "FW_V3_MIN_VERSION", "CONTROL_HZ", "CONTROL_DT",
    "GYRO_PARKED_DEADBAND_RADPS",
]


# ===========================================================================
# SECTION 0 -- CONSTANTS THAT ARE NOT CALIBRATION
#
# Everything here is either a wire fact (the firmware's own numbers) or a
# POLICY (a limit we impose). Policy limits may carry defaults; physical
# quantities may not. The distinction matters: choosing a conservative
# acceleration limit only costs time, whereas inventing a counts/mm silently
# rescales the arena.
# ===========================================================================

HARD_MAX_DUTY = 60
"""Last-line clamp on |duty| percent, mirroring fpms_duty_driver.HARD_MAX_DUTY.

Duplicated rather than imported ON PURPOSE: importing that module drags in the
serial transport and the framed-protocol path that `docs/` records as
unreachable on today's board, and this file must import on a Windows dev
machine with nothing installed. IF THAT CONSTANT CHANGES, CHANGE THIS ONE IN
THE SAME COMMIT. The firmware's own ceiling is FPMS_DUTY_LIMIT_PCT = 100; 60
sits above B8B's largest measured duty (TRN = 50) with margin and far below
anything that could be a runaway.
"""

FW_V3_MIN_VERSION = 30000
"""`/fpms_health` data[0] is FPMS_FW_VERSION; v3 ships 30001 (= 3.00.01).

Anything below 30000 is the factory or a pre-v3 build, i.e. a 50 % dead zone,
i.e. a rover this controller cannot control. Refuse, do not adapt.
"""

CONTROL_HZ = 25.0
"""Design control rate, in Hz.

Chosen to MATCH the measurement, not to exceed it: firmware v3 publishes
`/wheel_ticks` and `/wheel_duty` at FPMS_TICKS_HZ = 25. Running the loop faster
than the feedback arrives does not make it tighter, it makes it differentiate
stale data. The board's own control/deadman task runs at FPMS_CONTROL_HZ = 50,
so every command issued here is applied at least twice before it is refreshed.
"""

CONTROL_DT = 1.0 / CONTROL_HZ

DEADMAN_S = 0.30
"""The board zeroes all motors after FPMS_CMD_TIMEOUT_MS = 300 without a
command. At 25 Hz that is 7.5 missed ticks. Present here only so a caller can
size its watchdog against the same number; nothing in this file sleeps.
"""

GYRO_PARKED_DEADBAND_RADPS = 0.01
"""A HEALTHY parked gyro reads exactly 0.0 because of this deadband.

Used ONLY to decide "is the chassis still" in a settle test. It must never be
used to decide "is the sensor alive": a stationary rover reporting 0.0 rad/s is
the expected reading, and going looking for a broken IMU because of it has
already cost this project time once.
"""


# ===========================================================================
# SECTION 1 -- REFUSALS
#
# Named, comparable, and carrying the reason in the message. The executor's
# ABORT_* strings are the model: an operator should be able to read the string
# and know both what happened and what to do about it.
# ===========================================================================

class MotionRefusal(Exception):
    """This controller will not produce a command, and here is exactly why.

    A refusal is a SUCCESSFUL outcome of the safety design, not a bug. The
    alternative - emitting motion the controller cannot control, or closing a
    loop on a constant nobody measured - is how this project produced confident
    wrong numbers before.
    """

    def __init__(self, code: str, detail: str = ""):
        self.code = code
        self.detail = detail
        super().__init__(f"{code}: {detail}" if detail else code)


REFUSE_FIRMWARE = "firmware is not v3"
REFUSE_HEALTH_ABSENT = "no /fpms_health evidence"
REFUSE_HEALTH_STALE = "/fpms_health is stale"
REFUSE_ESTOP = "estop is latched on the board"
REFUSE_VEL_PATH = "the board is on the velocity path"
REFUSE_NO_GYRO = "the board reports the IMU did not initialise"
REFUSE_CALIBRATION = "required measured constants are missing"
REFUSE_CALIBRATION_FIRMWARE = "calibration was measured on other firmware"
REFUSE_GYRO_UNSCALED = "heading has no measured gyro_scale"
REFUSE_ARGUMENT = "the requested motion is not expressible"


# ===========================================================================
# SECTION 2 -- SMALL PURE HELPERS
# ===========================================================================

def clamp(v: float, lo: float, hi: float) -> float:
    """Bound `v` to [lo, hi]. Asserts the interval is real rather than
    silently swapping it - an inverted clamp is a programming error that
    otherwise produces a plausible constant."""
    if hi < lo:
        raise ValueError(f"clamp interval is inverted: [{lo}, {hi}]")
    return lo if v < lo else (hi if v > hi else v)


def wrap_pi(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _sign(v: float) -> float:
    return 1.0 if v >= 0.0 else -1.0


def _round_duty(x: float) -> int:
    """Round half AWAY FROM ZERO, so a symmetric pair of commands stays
    symmetric. Python's round() is half-to-even, which would round +0.5 and
    -0.5 both to 0 and quietly break a differential."""
    return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)


def _median4(values: Sequence[float]) -> float:
    """Median of exactly four numbers = mean of the middle two.

    MEDIAN AND NOT MEAN, DELIBERATELY. One wheel slipping, one hub working
    loose, or one encoder channel dropping counts moves a mean by a quarter of
    its error and moves this by nothing. On this chassis three per-wheel
    anomalies were each explained away separately and then a wheel came off.
    """
    s = sorted(values)
    return 0.5 * (s[1] + s[2])


# ===========================================================================
# SECTION 3 -- THE CALIBRATION PROFILE
#
# Mirrors `fpms_missions.load_calibration` / `_calib_float` deliberately: a
# profile is ADOPTED only when it carries `measured: true` AND each value is
# inside a sanity range, and every adoption or refusal appends a NOTE. Reading
# those notes is the only evidence that a number you set is a number being
# used.
#
# The difference from the executor's version, and it is the important one:
# a REFUSED or ABSENT value here becomes None and the controller REFUSES TO
# RUN. It does not fall back on a documented default, because there is no
# documented default for "how fast does this chassis go at 20 % duty" - nobody
# has measured it.
# ===========================================================================

_FIRMWARE_SENSITIVE = ("mps_per_duty", "radps_per_duty", "min_moving_duty")
"""Keys whose value is meaningless unless it was measured through firmware v3.

On factory firmware the dead zone makes every one of them an artefact of
saturation - the same trap that produced CMD_SCALE = 6.1. A profile that does
not say `"firmware": "v3"` gets these three refused with a note, while
counts_per_mm and the geometric keys survive (a hand push measures the same
number through any firmware, because the wheels are passive).
"""


@dataclass(frozen=True)
class Calibration:
    """Measured constants, each Optional, each with provenance in `notes`.

    NOTHING HERE HAS A PLAUSIBLE DEFAULT. Every field that a controller needs
    is None until an operator measured it, and `require()` names all the
    missing ones at once so a bring-up produces one shopping list rather than
    five successive refusals.
    """

    # -- geometry / measurement chain -------------------------------------
    counts_per_mm: Optional[float] = None
    """Encoder counts per millimetre of chassis travel, from RAW /wheel_ticks.

    MEASURED 2026-08-06 at ~5.5 by hand push (forward 5.68, backward 5.47; a
    powered run gave 6.56, inflated by tyre slip). The 2.7x correction over the
    old 14.8 is robust. The third digit is not - it was taken on a chassis with
    a loose front-left hub."""

    gyro_scale: Optional[float] = None
    """true_deg / integrated_gyro_deg. UNMEASURED as of this writing. Turn
    accuracy rests entirely on it, so `TurnController` refuses without it."""

    # -- actuation map (NEW KEYS -- see MOTION.md, nothing writes these yet) --
    mps_per_duty: Optional[float] = None
    """Chassis speed per 1 % of commanded duty, straight-line, on the arena
    surface, above the stiction floor. This is the ONE anchor that turns a
    velocity profile into a duty command."""

    radps_per_duty: Optional[float] = None
    """Yaw rate per 1 % of DIFFERENTIAL duty, turning in place.

    Measured directly rather than derived from track width on purpose:
    LR_WHEELS_DISTANCE is UNMEASURED (fpms_config.h says so explicitly - the
    operator measured 105 mm front-to-back, which is the wheelbase, not the
    track), and a skid-steer's effective track is not its geometric one anyway
    because the wheels scrub."""

    min_moving_duty: Optional[float] = None
    """Smallest duty percent at which the wheels actually turn under load.

    Below this the rover does not move and any integrator winds up against a
    stationary robot. A profile that dips below it is not a slower profile, it
    is a stopped one. NOTE: a profile carrying 0.0 is REFUSED - a chassis with
    no stiction floor does not exist, so 0.0 means the measurement was never
    taken."""

    # -- trims -------------------------------------------------------------
    lr_asymmetry: Optional[float] = None
    """left_travel / right_travel at equal duty. ~1.10 has been seen on this
    rover (and one rear wheel was slipping: 950 mm against 740-755)."""

    wheel_trims: Optional[Tuple[float, float, float, float]] = None
    """Per-wheel multipliers in /cmd_duty order [FL, FR, RL, RR].

    THE ONLY TRIM VALUES PUBLISHED FOR THIS ROVER ARE NOT USABLE HERE.
    {0.820, 0.839, 0.914, 1.000} is quoted in several places as a golden B8B
    constant and is not one: it comes from firmware_linorobot/moves_main.cpp,
    at BASE_DUTY 70 with a kick to 140. This controller cruises near the duty
    floor, perhaps a third of that, where the motor curve and the stiction are
    different. Adopting those four numbers here would apply a correction
    measured in one regime to motion in another, which is the same mistake as
    carrying CMD_SCALE across a firmware change. Trims must be re-measured at
    the duty this controller actually uses, or left absent (= unity)."""

    coast_mm: Optional[float] = None
    coast_deg: Optional[float] = None
    """How far the chassis carries after a command is CUT AT SPEED.

    PARSED FOR DIAGNOSTICS AND DELIBERATELY NOT APPLIED BY THIS CONTROLLER.
    Two independent reasons:

      * It is the wrong regime. `coast_mm` is measured by cutting from cruise
        (the burst executor's only option). This controller decelerates under
        control and cuts at the DUTY FLOOR, where the carried momentum is a
        different and much smaller number. Subtracting a cruise-cut coast from
        a floor-cut move would remove distance that was never carried.
      * A blanket coast correction is a diagnostic hazard. `DRIVE_COAST_FACTOR
        = 0.90` in fpms_missions.py has no golden ancestor - golden never
        coasted drives at all - and a constant 0.90 ratio is indistinguishable
        on a residual plot from a 10 % counts/mm error, which is precisely the
        distinction `_publish_residual`'s docstring teaches operators to read
        (constant OFFSET = coast, constant RATIO = scale). Applying either
        shape here would destroy that diagnostic.

    If a coast term is ever needed on this path it is a NEW measurement with a
    new name - "distance carried after a cut at the duty floor" - and it will
    be a few millimetres, not the tens that a cruise cut carries."""

    # -- polarity / mapping, both binary and both able to invert the rover --
    duty_forward_sign: Optional[int] = None
    """+1 if positive /cmd_duty drives the chassis FORWARD, -1 if backward.

    THIS IS A PROPERTY OF THE FLASHED BINARY, NOT OF THE REPO. Measured
    2026-08-06 on the board as it then stood: +60 % on all four drove EVERY
    wheel backward (ticks [-47952, -58868, -28919, -50845] over 15 s, confirmed
    by eye), and -60 % drove it forward 53 in. `firmware_v3/fpms_config.h` now
    carries the flipped MOTORn_INV values, so a board flashed with the CURRENT
    source should want +1 - but "should" is not a measurement and getting this
    wrong drives the rover backwards at the full commanded speed. There is no
    default."""

    side_map_confirmed: Optional[bool] = None
    """True only after an operator has WATCHED ONE SIDE TURN ON ITS OWN.

    The firmware's topic table declares /cmd_duty order [FL, FR, RL, RR]
    (MOTOR1..4 = board M3, M1, M4, M2). fpms_duty_driver.py carries
    SIDE_MAP_MEASURED = False over exactly this question, because
    fpms_missions.py records the opposite mapping after the 2026-08-01
    rewiring. Straight-line motion survives a swapped map; every turn and every
    heading correction inverts. Same guard, same reason."""

    notes: Tuple[str, ...] = ()
    """Adoptions, refusals and absences, in the order they were decided.
    Log these at startup. They are the only evidence of which numbers are
    live."""

    # -- construction ------------------------------------------------------
    @classmethod
    def from_profile(cls, profile: Optional[dict]) -> "Calibration":
        """Build from an already-parsed /etc/fpms/calibration.json dict.

        The CALLER does the file I/O; this module never opens anything. Passing
        None (no profile on the image, which is the shipped state) yields an
        all-None calibration and one note, and every controller then refuses
        with a list of what to measure.
        """
        notes: list = []

        if profile is None:
            notes.append("calibration: no profile supplied; every measured "
                         "constant is absent and motion will be REFUSED")
            return cls(notes=tuple(notes))

        if not isinstance(profile, dict):
            notes.append(f"calibration: profile is {type(profile).__name__}, "
                         "not a JSON object; IGNORED entirely")
            return cls(notes=tuple(notes))

        if not profile.get("measured"):
            notes.append("calibration: profile is not marked measured:true - "
                         "IGNORED entirely. Finish the characterisation run "
                         "before trusting it.")
            return cls(notes=tuple(notes))

        fw = str(profile.get("firmware", "")).strip().lower()
        fw_is_v3 = fw in ("v3", "3", "fpms_v3", "fpmsv3")
        if not fw_is_v3:
            notes.append(
                f"calibration: firmware={profile.get('firmware')!r} is not v3, "
                f"so {', '.join(_FIRMWARE_SENSITIVE)} are REFUSED. On factory "
                "firmware a 50 % dead zone makes every duty/speed number an "
                "artefact of saturation - that is what CMD_SCALE 6.1 was.")

        def num(key: str, lo: float, hi: float,
                firmware_sensitive: bool = False) -> Optional[float]:
            if firmware_sensitive and not fw_is_v3:
                return None
            raw = profile.get(key)
            if raw is None:
                notes.append(f"calibration {key} ABSENT - not defaulted")
                return None
            if isinstance(raw, bool):     # bool is an int in Python; catch it
                notes.append(f"calibration {key}={raw!r} is a boolean, not a "
                             "number; REFUSED")
                return None
            try:
                v = float(raw)
            except (TypeError, ValueError):
                notes.append(f"calibration {key}={raw!r} is not a number; "
                             "REFUSED")
                return None
            if not math.isfinite(v):
                notes.append(f"calibration {key}={v!r} is not finite; REFUSED")
                return None
            if not (lo <= v <= hi):
                # Far outside plausibility is far more likely to be a botched
                # run than a discovery. Refuse it; do not act on it.
                notes.append(f"calibration {key}={v:g} is outside "
                             f"[{lo:g}, {hi:g}] and was REFUSED")
                return None
            notes.append(f"calibration {key}={v:g} ADOPTED (measured "
                         f"{profile.get('measured_at', 'date unknown')})")
            return v

        counts_per_mm = num("counts_per_mm", 1.0, 30.0)
        gyro_scale = num("gyro_scale", 0.2, 5.0)
        # Range admits B8B's 26 % drive duty at anything from a crawl to
        # ~0.5 m/s, and rejects a decimal-point slip in either direction.
        mps_per_duty = num("mps_per_duty", 5e-4, 2e-2, firmware_sensitive=True)
        radps_per_duty = num("radps_per_duty", 1e-3, 1e-1,
                             firmware_sensitive=True)
        # LOWER BOUND 1.0 AND NOT 0.0, DELIBERATELY. calibration.json.example
        # ships min_moving_duty: 0.0 as a placeholder. A chassis with no
        # stiction floor does not exist, so 0.0 is not a measurement of zero -
        # it is the absence of a measurement, and adopting it would let the
        # profile command duties that cannot turn a wheel while the integrator
        # wound up against a stationary robot.
        min_moving_duty = num("min_moving_duty", 1.0, 40.0,
                              firmware_sensitive=True)
        if (min_moving_duty is None and fw_is_v3
                and _is_number(profile.get("min_moving_duty"))
                and float(profile["min_moving_duty"]) == 0.0):
            notes.append("calibration min_moving_duty=0 means the duty floor "
                         "was never measured, not that the chassis has none. "
                         "Run the floor sweep (MOTION.md) before driving.")
        lr_asymmetry = num("lr_asymmetry", 0.5, 2.0)
        coast_mm = num("coast_mm", 0.0, 500.0)
        coast_deg = num("coast_deg", 0.0, 90.0)

        # -- wheel trims: four numbers or nothing --------------------------
        trims: Optional[Tuple[float, float, float, float]] = None
        raw_trims = profile.get("wheel_trims")
        if raw_trims is None:
            notes.append("calibration wheel_trims ABSENT - wheels are treated "
                         "as identical, which lr_asymmetry only partly covers")
        elif (not isinstance(raw_trims, (list, tuple))
                or len(raw_trims) != 4):
            notes.append(f"calibration wheel_trims={raw_trims!r} is not four "
                         "numbers [FL, FR, RL, RR]; REFUSED")
        else:
            vals = []
            bad = False
            for i, r in enumerate(raw_trims):
                if isinstance(r, bool) or not _is_number(r):
                    notes.append(f"calibration wheel_trims[{i}]={r!r} is not a "
                                 "number; the whole set is REFUSED")
                    bad = True
                    break
                v = float(r)
                if not (0.5 <= v <= 1.5):
                    notes.append(f"calibration wheel_trims[{i}]={v:g} is "
                                 "outside [0.5, 1.5]; the whole set is "
                                 "REFUSED")
                    bad = True
                    break
                vals.append(v)
            if not bad:
                trims = (vals[0], vals[1], vals[2], vals[3])
                notes.append(f"calibration wheel_trims={trims} ADOPTED")

        # -- duty_forward_sign: exactly +1 or -1, never inferred -----------
        fsign: Optional[int] = None
        raw_sign = profile.get("duty_forward_sign")
        if raw_sign is None:
            notes.append("calibration duty_forward_sign ABSENT. It is a "
                         "property of the FLASHED BINARY: +60 % on all four "
                         "drove this rover BACKWARD on 2026-08-06. Confirm it "
                         "with a 1 s on-blocks pulse before any floor run.")
        elif raw_sign in (1, -1, 1.0, -1.0, "+1", "-1"):
            fsign = int(float(raw_sign))
            notes.append(f"calibration duty_forward_sign={fsign:+d} ADOPTED")
        else:
            notes.append(f"calibration duty_forward_sign={raw_sign!r} is not "
                         "+1 or -1; REFUSED")

        # -- side_map_confirmed: an operator's eyes, or nothing ------------
        side: Optional[bool] = None
        raw_side = profile.get("side_map_confirmed")
        if raw_side is None:
            notes.append("calibration side_map_confirmed ABSENT - no operator "
                         "has watched one side turn on its own. Turns and "
                         "heading holds are REFUSED (see the note in "
                         "fpms_duty_driver.SIDE_MAP_MEASURED).")
        elif raw_side is True:
            side = True
            notes.append("calibration side_map_confirmed=true ADOPTED")
        elif raw_side is False:
            side = False
            notes.append("calibration side_map_confirmed=false - the operator "
                         "explicitly has NOT confirmed it; turns REFUSED")
        else:
            notes.append(f"calibration side_map_confirmed={raw_side!r} is not "
                         "a boolean; REFUSED")

        return cls(counts_per_mm=counts_per_mm,
                   gyro_scale=gyro_scale,
                   mps_per_duty=mps_per_duty,
                   radps_per_duty=radps_per_duty,
                   min_moving_duty=min_moving_duty,
                   lr_asymmetry=lr_asymmetry,
                   wheel_trims=trims,
                   coast_mm=coast_mm,
                   coast_deg=coast_deg,
                   duty_forward_sign=fsign,
                   side_map_confirmed=side,
                   notes=tuple(notes))

    # -- requirements ------------------------------------------------------
    def missing(self, *names: str) -> Tuple[str, ...]:
        """Which of `names` are absent. `side_map_confirmed` counts as missing
        when it is False as well as when it is None: an explicit "no, nobody
        checked" is not consent."""
        out = []
        for n in names:
            v = getattr(self, n)
            if v is None or (n == "side_map_confirmed" and v is not True):
                out.append(n)
        return tuple(out)

    def require(self, *names: str) -> None:
        """Raise `MotionRefusal` listing EVERY missing constant at once.

        One refusal with a shopping list beats five successive refusals during
        a bring-up, which is when this will actually be read.
        """
        miss = self.missing(*names)
        if miss:
            raise MotionRefusal(
                REFUSE_CALIBRATION,
                "missing measured constants " + ", ".join(miss)
                + ". Nothing was substituted: a guessed calibration changes "
                "every distance and heading and says nothing about it. "
                "See docs/MOTION.md for how to measure each one.")

    # -- convenience -------------------------------------------------------
    #
    # THERE IS NO coast_mm_or_zero() AND THERE MUST NOT BE. See the `coast_mm`
    # field docstring: this controller decelerates under control and cuts at
    # the duty floor, so a coast measured by cutting from cruise is a
    # correction from a regime this path never enters. The residual is
    # REPORTED instead, which keeps the offset-vs-ratio diagnostic intact.

    def trims_or_unity(self) -> Tuple[float, float, float, float]:
        """Per-wheel trims, or unity. Unity = "treat the wheels as identical",
        which is exactly what having no trims means."""
        return self.wheel_trims if self.wheel_trims is not None else (1.0, 1.0, 1.0, 1.0)

    def asymmetry_or_unity(self) -> float:
        """left/right ratio, or 1.0. As above: 1.0 is the no-op, not a guess
        at the real imbalance (which has been seen near 1.10)."""
        return 1.0 if self.lr_asymmetry is None else self.lr_asymmetry


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


# ===========================================================================
# SECTION 4 -- POLICY LIMITS
#
# These MAY carry defaults, because they are limits we impose rather than
# properties we measured. Setting them too tight costs time; it cannot make
# the rover confidently wrong.
# ===========================================================================

@dataclass(frozen=True)
class Limits:
    """Ceilings applied to every command this module produces."""

    hard_max_duty: int = HARD_MAX_DUTY

    max_duty_step: float = 8.0
    """Largest change in one wheel's duty in one control tick.

    8 % per tick at 25 Hz = 200 %/s, so 0 -> HARD_MAX_DUTY takes 0.3 s. "A duty
    step of 60 in one tick is a lurch" is the requirement; this is the number
    that makes it impossible. `StraightRun` narrows it further, to whatever
    a_max implies through the measured mps_per_duty, so the rate limiter never
    silently becomes the real acceleration limit."""

    max_v_mps: float = 0.25
    """Matches fpms_missions.HARD_MAX_LIN_MPS. A last-line clamp in real units
    so a caller cannot request a speed the executor would have refused."""

    max_omega_radps: float = 0.90
    """Matches fpms_missions.HARD_MAX_ANG_RADPS."""

    a_max_mps2: float = 0.25
    """Linear acceleration limit. A POLICY, NOT A MEASUREMENT.

    At v_cruise = 0.18 m/s (the executor's configured cruise) this reaches
    cruise in 0.72 s over 65 mm - comparable to one MAX_LEG_MM leg, so a
    typical leg spends most of its length at speed. Too LOW only costs time and
    shows up as the rover leading its own profile; too HIGH shows up as the
    rover LAGGING it, which the bounded feedback absorbs and the residual
    reports. Neither direction can produce a confidently wrong distance."""

    alpha_max_radps2: float = 1.2
    """Angular acceleration limit, same argument. At TURN_RADPS = 0.45 this
    reaches turn speed in 0.375 s over 4.8 deg."""

    def __post_init__(self):
        if self.hard_max_duty <= 0 or self.hard_max_duty > 100:
            raise ValueError("hard_max_duty must be in (0, 100]")
        for name in ("max_duty_step", "max_v_mps", "max_omega_radps",
                     "a_max_mps2", "alpha_max_radps2"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")


DEFAULT_LIMITS = Limits()


# ===========================================================================
# SECTION 5 -- THE FIRMWARE GATE
#
# THE REFUSAL THAT DEFINES THIS MODULE.
# ===========================================================================

@dataclass(frozen=True)
class Health:
    """A parsed `/fpms_health` sample. Layout from firmware_v3 publishHealth().

    `age_s` is supplied by the CALLER, because this module has no clock. Pass
    (now - stamp_of_last_health_message).
    """

    fw_version: int
    flags: int
    active_path: int          # 0 stopped, 1 duty, 2 velocity
    ms_since_duty: int
    ms_since_vel: int
    duty_hz: float            # published as Hz x10
    control_hz: float         # published as Hz x10
    enc_moved_mask: int       # bit per wheel, changed in the last ~2 s
    battery_v: float
    imu_who: int
    free_heap_kb: int
    uptime_s: int
    age_s: float

    @property
    def estop_latched(self) -> bool:
        return bool(self.flags & (1 << 0))

    @property
    def vel_armed(self) -> bool:
        return bool(self.flags & (1 << 1))

    @property
    def deadman_firing(self) -> bool:
        """The board is zeroing the motors because no /cmd_duty arrived
        recently. THIS IS THE NORMAL STATE BEFORE A RUN STARTS and must never
        be a refusal - refusing on it would refuse every cold start."""
        return bool(self.flags & (1 << 2))

    @property
    def imu_ok(self) -> bool:
        return bool(self.flags & (1 << 3))

    @property
    def agent_connected(self) -> bool:
        return bool(self.flags & (1 << 4))

    @classmethod
    def from_array(cls, data: Sequence[int], age_s: float) -> "Health":
        """Parse `/fpms_health` data[12]. Raises on a short array.

        A SHORT ARRAY IS NOT A PARTIAL HEALTH REPORT. The firmware publishes
        exactly 12 fields; anything else is a different message, a different
        firmware, or a truncated read, and inferring "probably fine" from it is
        precisely the class of bug this file exists to avoid.
        """
        if data is None:
            raise MotionRefusal(REFUSE_HEALTH_ABSENT,
                                "health array is None")
        if len(data) < 12:
            raise MotionRefusal(
                REFUSE_HEALTH_ABSENT,
                f"/fpms_health carried {len(data)} fields, expected 12. This "
                "is not a v3 health message.")
        return cls(fw_version=int(data[0]),
                   flags=int(data[1]),
                   active_path=int(data[2]),
                   ms_since_duty=int(data[3]),
                   ms_since_vel=int(data[4]),
                   duty_hz=int(data[5]) / 10.0,
                   control_hz=int(data[6]) / 10.0,
                   enc_moved_mask=int(data[7]),
                   battery_v=int(data[8]) / 1000.0,
                   imu_who=int(data[9]),
                   free_heap_kb=int(data[10]),
                   uptime_s=int(data[11]),
                   age_s=float(age_s))


def require_firmware_v3(health: Optional[Health],
                        *, max_age_s: float = 2.0) -> None:
    """Refuse unless the board is provably running firmware v3 and is willing
    to accept duty commands. Raises `MotionRefusal`; returns None on success.

    WHY THIS IS A REFUSAL AND NOT A FALLBACK
    ----------------------------------------
    On factory firmware, PWM_MOTOR_DEAD_ZONE (200 of 400) is added as
    feed-forward to every velocity command. The smallest expressible non-zero
    output is ~0.65 m/s and the smallest expressible move is ~227 mm. A
    trapezoid whose cruise speed is 0.08 m/s, whose acceleration ramp is 65 mm
    long and whose terminal band is a few millimetres has NO REPRESENTATION AT
    ALL there. Running it anyway does not produce a degraded version of this
    controller; it produces a series of 0.65 m/s lurches with a feedback loop
    confidently reporting that it is correcting them.

    NO EVIDENCE IS NOT EVIDENCE OF v3. A missing or stale `/fpms_health` is
    refused for the same reason the DDS notes give: on this stack discovery can
    succeed while no data flows at all, so "I have not heard otherwise" is
    exactly the state a shared-memory transport failure produces.
    """
    if health is None:
        raise MotionRefusal(
            REFUSE_HEALTH_ABSENT,
            "no /fpms_health sample. The firmware version cannot be assumed "
            "from a silent topic - on this stack Fast DDS shared memory has "
            "let discovery succeed while no data flowed at all. Subscribe to "
            "/fpms_health (2 Hz) and prove v3 before commanding anything.")

    if health.age_s > max_age_s:
        raise MotionRefusal(
            REFUSE_HEALTH_STALE,
            f"newest /fpms_health is {health.age_s:.1f}s old (limit "
            f"{max_age_s:.1f}s, published at 2 Hz). A stale health field is "
            "worse than no health field.")

    if health.fw_version < FW_V3_MIN_VERSION:
        raise MotionRefusal(
            REFUSE_FIRMWARE,
            f"/fpms_health reports firmware {health.fw_version}, which is "
            f"below v3 ({FW_V3_MIN_VERSION}). That build adds a ~50 % dead "
            "zone to every velocity command: the smallest expressible speed is "
            "~0.65 m/s and the smallest expressible move ~227 mm, so a "
            "continuous profiled move is not slow there - it is "
            "UNREPRESENTABLE. This controller will not pretend otherwise. Use "
            "the burst/stop/settle executor on that firmware, or flash v3.")

    if health.estop_latched:
        raise MotionRefusal(
            REFUSE_ESTOP,
            "the board has estop latched. It stays latched until an explicit "
            "false is published; clearing it is an operator action, not a "
            "controller action.")

    if health.active_path == 2:
        raise MotionRefusal(
            REFUSE_VEL_PATH,
            "the board reports the VELOCITY path active. This controller owns "
            "/cmd_duty; two writers to one chassis is how a runaway hides. "
            "Disarm /cmd_enable first.")

    # Deliberately NOT refused here:
    #   * deadman_firing - the normal state before the first command; refusing
    #     on it would refuse every cold start.
    #   * enc_moved_mask - a parked rover has moved no wheels, by definition.
    #   * imu_ok - only heading needs it; see require_gyro(). A straight-line
    #     open-heading move is still meaningful without a gyro.


def require_gyro(health: Optional[Health]) -> None:
    """Additional gate for anything that closes on heading.

    Separate from `require_firmware_v3` because the two failures have different
    consequences: no gyro means no turn and no heading hold, but distance
    control on `/wheel_ticks` is untouched.
    """
    if health is None:
        raise MotionRefusal(REFUSE_HEALTH_ABSENT,
                            "no /fpms_health, so the IMU state is unknown")
    if not health.imu_ok:
        raise MotionRefusal(
            REFUSE_NO_GYRO,
            "/fpms_health flag bit3 says the ICM42670P did not initialise. "
            "Heading comes entirely from integrating its Z rate - /odom_raw "
            "carries an identity quaternion and there is no wheel-derived "
            "heading to fall back on. Note the opposite trap: a HEALTHY gyro "
            "reads exactly 0.0 while parked (+/-0.01 rad/s deadband), so a "
            "zero reading is not this fault.")


# ===========================================================================
# SECTION 6 -- MEASUREMENT HELPERS
#
# Pose and ticks only. There is no twist anywhere in this section, and that
# is not an oversight (see the module docstring, fact 1).
# ===========================================================================

@dataclass(frozen=True)
class TravelSample:
    """Chassis travel derived from raw per-wheel ticks."""

    mm: float
    """MEDIAN of the four wheel estimates. Robust to one wheel slipping, one
    hub loosening, or one encoder channel dropping counts."""

    spread_mm: float
    """max - min across the four wheels over this interval. A DIAGNOSTIC, not
    a correction: a spread that grows over a run is a mechanical fault
    announcing itself, and averaging it away is how three per-wheel anomalies
    got explained away separately shortly before a wheel came off."""

    per_wheel_mm: Tuple[float, float, float, float]


def travel_mm_from_ticks(prev_ticks: Sequence[int],
                         ticks: Sequence[int],
                         counts_per_mm: Optional[float]) -> TravelSample:
    """Convert a /wheel_ticks delta into chassis travel, in mm.

    `ticks` are RAW CUMULATIVE counts in /cmd_duty order [FL, FR, RL, RR], with
    MOTORn_ENCODER_INV already applied ON THE BOARD - a hand spin forward gives
    POSITIVE ticks on all four, which is the measured state of those four
    booleans. So the sign here is the CHASSIS's sense of forward and
    `duty_forward_sign` must NOT be applied to it: that constant describes
    which duty produces forward motion, not which way the counters run.

    Refuses without a measured counts_per_mm rather than picking one. Both
    contested values (5.5 and 14.8) are internally consistent and differ by
    2.7x - which is the entire 2.3 m overshoot.
    """
    if counts_per_mm is None:
        raise MotionRefusal(
            REFUSE_CALIBRATION,
            "counts_per_mm is not measured, so a tick count cannot be turned "
            "into a distance. The values on record disagree by up to 2.7x: 5.5 "
            "measured by hand push (SPEC.md), 6.00 implied by "
            "fpms_duty_driver.py:114's TKMM = pi*70/1320 (a 9 % overshoot on "
            "its own), and 14.8 from an earlier bare-metal build. That one "
            "factor explains the 2.3 m overshoot, the 744 mm plan that drove "
            "~2 m and the 300 mm move that went ~1 m. Pick it with a tape "
            "measure, not from a header.")
    if counts_per_mm <= 0:
        raise ValueError("counts_per_mm must be positive")
    if len(prev_ticks) != 4 or len(ticks) != 4:
        raise ValueError("/wheel_ticks carries exactly four wheels")

    per = tuple((float(ticks[i]) - float(prev_ticks[i])) / counts_per_mm
                for i in range(4))
    return TravelSample(mm=_median4(per),
                        spread_mm=max(per) - min(per),
                        per_wheel_mm=per)  # type: ignore[arg-type]


@dataclass(frozen=True)
class GyroState:
    """Integrated heading. `scaled` records whether a MEASURED gyro_scale was
    applied, so a consumer can refuse rather than inherit an unscaled angle."""

    yaw_rad: float = 0.0
    t: Optional[float] = None
    scaled: bool = False


def integrate_gyro(state: GyroState, t: float, wz_radps: float,
                   gyro_scale: Optional[float]) -> GyroState:
    """Rectangular integration of the gyro's Z rate. Pure; `t` is a parameter.

    Rectangular and not trapezoidal on purpose: the /imu stream is 20 Hz and
    genuinely sampled, so the extra half-sample of accuracy from trapezoidal
    integration is well below the unmeasured gyro_scale it would be multiplied
    by. Simplicity that a test can reproduce by hand is worth more here.

    A ZERO RATE IS NOT SUPPRESSED AND IS NOT A FAULT. The board applies a
    +/-0.01 rad/s deadband, so a parked rover integrates nothing, correctly.

    With `gyro_scale=None` the result is RAW integrated radians and `scaled`
    is False. `HeadingHold` accepts that (regulating an error to zero is
    invariant to a positive sensor scale); `TurnController` refuses it.
    """
    if state.t is None:
        return replace(state, t=t,
                       scaled=state.scaled or gyro_scale is not None)
    dt = t - state.t
    if dt <= 0.0:
        # Time did not advance, or went backwards. Integrating a negative dt
        # would UNWIND real rotation; extrapolating would invent it. Hold.
        return state
    k = 1.0 if gyro_scale is None else gyro_scale
    return GyroState(yaw_rad=state.yaw_rad + wz_radps * dt * k,
                     t=t,
                     scaled=gyro_scale is not None)


# ===========================================================================
# SECTION 7 -- THE TRAPEZOIDAL PROFILE
#
# One implementation, unit-agnostic, wrapped twice. The linear and angular
# profiles are the same arithmetic and keeping them one function means a bug
# fixed in one is fixed in both - the alternative has already produced a
# `NameError` that lived in _turn and not in _drive for months.
# ===========================================================================

PHASE_ACCEL = "accel"
PHASE_CRUISE = "cruise"
PHASE_DECEL = "decel"
PHASE_DONE = "done"
PHASE_BURST = "burst"


@dataclass(frozen=True)
class ProfilePoint:
    """The reference at one instant: where the profile says we should be, how
    fast it says we should be going, and which phase that is."""

    t_s: float
    s: float          # reference position along the move (mm, or rad)
    v: float          # reference speed (m/s, or rad/s)
    phase: str


class _Trapezoid:
    """Rest-to-rest trapezoidal profile over a distance. Immutable once built.

    UNITS ARE THE CALLER'S. Everything is (distance, speed, accel) in a
    consistent set; the wrappers below fix them to (mm, m/s, m/s^2) and
    (rad, rad/s, rad/s^2).

    THE THREE CASES, AND WHY THE THIRD ONE IS NAMED HONESTLY
    -------------------------------------------------------
    1. TRAPEZOID. The move is long enough to reach `v_cruise`:
           d_ramp = v_cruise^2 / (2a)      and    distance >= 2 * d_ramp
    2. TRIANGLE. It is not, so the peak is wherever accel and decel meet:
           v_peak = sqrt(a * distance)
    3. BURST. Even the triangle's peak is below `v_min` - the duty floor - and
       the ramps at `v_min` do not fit inside the distance:
           distance < v_min^2 / a
       There is NO profile here. The chassis cannot accelerate to a speed it
       can express and stop again inside the distance available, so the command
       degenerates to a square pulse of `v_min` and the rover arrives with
       kinetic energy it must coast off. `is_burst` says so, and a caller
       should hand these to the measure-and-correct executor rather than
       pretending the loop is closed. THIS IS THE EXACT BOUNDARY at which
       "continuous motion where the hardware permits it" stops permitting.
    """

    __slots__ = ("distance", "v_cruise", "a_max", "v_min", "v_peak",
                 "d_accel", "d_cruise", "d_decel",
                 "t_accel", "t_cruise", "t_decel", "total_time",
                 "is_triangular", "is_burst", "floor_limited", "lag_s")

    def __init__(self, distance: float, v_cruise: float, a_max: float,
                 v_min: float = 0.0, lag_s: float = 0.0):
        if distance < 0:
            raise ValueError("distance must be >= 0; give direction to the "
                             "controller, not to the profile")
        if v_cruise <= 0 or a_max <= 0:
            raise ValueError("v_cruise and a_max must be positive")
        if v_min < 0:
            raise ValueError("v_min must be >= 0")

        if lag_s < 0:
            raise ValueError("lag_s must be >= 0")
        self.distance = float(distance)
        self.a_max = float(a_max)
        self.v_min = float(v_min)
        self.lag_s = float(lag_s)

        # A floor above the requested cruise is not an error: it means the
        # slowest motion this chassis can express is faster than we asked for.
        # Cruise at the floor and SAY SO, rather than commanding a speed the
        # wheels will ignore.
        self.floor_limited = v_min >= v_cruise
        self.v_cruise = float(max(v_cruise, v_min))

        self.is_burst = False
        self.is_triangular = False

        d_ramp_full = self.v_cruise ** 2 / (2.0 * self.a_max)
        if self.distance >= 2.0 * d_ramp_full:
            self.v_peak = self.v_cruise
            self.d_accel = self.d_decel = d_ramp_full
        else:
            self.is_triangular = True
            self.v_peak = math.sqrt(self.a_max * self.distance)
            if self.v_peak < self.v_min:
                # Below the duty floor. Raise the peak to the floor and check
                # whether the ramps still fit.
                self.v_peak = self.v_min
                self.floor_limited = True
                if self.distance < self.v_min ** 2 / self.a_max:
                    self.is_burst = True
            self.d_accel = self.d_decel = min(0.5 * self.distance,
                                              self.v_peak ** 2 / (2.0 * self.a_max))

        if self.is_burst:
            # No room to ramp. Square pulse at the floor speed.
            self.d_accel = self.d_decel = 0.0
            self.d_cruise = self.distance
            self.t_accel = self.t_decel = 0.0
            self.t_cruise = (self.distance / self.v_peak) if self.v_peak > 0 else 0.0
        else:
            self.d_cruise = max(0.0, self.distance - self.d_accel - self.d_decel)
            self.t_accel = self.v_peak / self.a_max
            self.t_decel = self.t_accel
            self.t_cruise = (self.d_cruise / self.v_peak) if self.v_peak > 0 else 0.0
        self.total_time = self.t_accel + self.t_cruise + self.t_decel

    # -- native-unit implementations --------------------------------------
    #
    # THESE THREE ARE PRIVATE AND THEY CALL ONLY EACH OTHER, WHICH IS NOT
    # STYLE - IT IS THE FIX FOR A BUG THAT PASSED EVERY OTHER CHECK.
    # TrapezoidalProfile overrides the PUBLIC methods to convert mm/s to m/s at
    # the edge. When the public `command_v` was implemented in terms of
    # `self.v_envelope(...)`, Python dispatched that call to the SUBCLASS's
    # override, so a routine working internally in mm/s received a cap in m/s -
    # a factor of 1000 - and every commanded speed collapsed onto the duty
    # floor. The module imported cleanly, the loop converged, and 1000 mm moves
    # finished 4 mm short of target: the numbers were RIGHT and the rover was
    # crawling at a sixth of its cruise speed for 33 seconds. Only simulating
    # the trajectory and looking at the peak speed found it. Internal code
    # calls `_native` methods; overrides touch only the public ones.

    def _at_time(self, t: float) -> ProfilePoint:
        """The IDEAL reference at time `t` since the move started.

        This is the feedforward AND the position the feedback trims against. It
        is the ideal ramp: it is NOT floored at v_min, because the floor is an
        actuator property and belongs where the actuator is (`command_v`).
        Keeping the reference ideal means a test can check it against
        s = v^2/2a by hand.
        """
        if t <= 0.0:
            return ProfilePoint(t_s=max(0.0, t), s=0.0,
                                v=(self.v_peak if self.is_burst else 0.0),
                                phase=PHASE_BURST if self.is_burst else PHASE_ACCEL)
        if t >= self.total_time:
            return ProfilePoint(t_s=t, s=self.distance, v=0.0, phase=PHASE_DONE)

        if self.is_burst:
            return ProfilePoint(t_s=t, s=self.v_peak * t, v=self.v_peak,
                                phase=PHASE_BURST)

        if t < self.t_accel:
            return ProfilePoint(t_s=t, s=0.5 * self.a_max * t * t,
                                v=self.a_max * t, phase=PHASE_ACCEL)

        t2 = t - self.t_accel
        if t2 < self.t_cruise:
            return ProfilePoint(t_s=t, s=self.d_accel + self.v_peak * t2,
                                v=self.v_peak, phase=PHASE_CRUISE)

        t3 = t2 - self.t_cruise                       # into the decel ramp
        v = max(0.0, self.v_peak - self.a_max * t3)
        s = self.d_accel + self.d_cruise + (self.v_peak * t3
                                            - 0.5 * self.a_max * t3 * t3)
        return ProfilePoint(t_s=t, s=min(s, self.distance), v=v,
                            phase=PHASE_DECEL)

    # -- the distance-parameterised braking envelope ----------------------
    def _v_envelope(self, s_travelled: float) -> float:
        """The fastest speed from which the move can still stop at `distance`.

        THIS IS WHAT MAKES ARRIVAL NON-OVERSHOOTING EVEN WHEN THE TIME
        REFERENCE IS WRONG. If the rover lags its profile (a low battery, a
        carpet, an a_max we guessed too high), `at_time` keeps advancing and
        the position trim keeps asking for more speed - and this cap silently
        removes exactly as much of it as the remaining distance cannot absorb.
        It is a function of MEASURED position only, so it degrades with the
        measurement rather than with the model.

        LAG COMPENSATION, AND WHY IT IS NOT OPTIONAL IN PRACTICE. The textbook
        envelope sqrt(2*a*rem) assumes the chassis decelerates the instant the
        command drops. It does not: the drivetrain has a first-order lag, so
        the real stopping distance is the ramp PLUS what is carried during the
        lag,

            rem = v^2 / (2a) + v * lag        =>
            v   = -a*lag + sqrt((a*lag)^2 + 2*a*rem)

        which is what this returns when `lag_s > 0` and reduces exactly to the
        textbook form when it is 0. WITHOUT THIS TERM the turn simulation
        arrived a consistent +1.3 deg PAST every target, in the same direction
        every time, at every angle from 15 to 360 deg - which is a BIAS, the
        one error a retrace cannot cancel because it reappears identically on
        the way back. It is not a tuning problem and no gain fixes it: the
        envelope was solving the wrong equation.
        """
        rem = max(0.0, self.distance - s_travelled)
        if self.lag_s > 0.0:
            al = self.a_max * self.lag_s
            v = -al + math.sqrt(al * al + 2.0 * self.a_max * rem)
        else:
            v = math.sqrt(2.0 * self.a_max * rem)
        return min(self.v_peak, v)

    # -- the floor --------------------------------------------------------
    def _command_v(self, v_desired: float, s_travelled: float,
                   stop_band: float, v_meas: Optional[float] = None) -> float:
        """Turn a desired speed into a COMMANDABLE one.

        Three rules, in this order:
          1. Inside the stop band of the target, command exactly zero. This is
             the anti-hunting rule and it is structural, not a tuning hope: the
             band is at least as wide as the distance one tick at the duty
             floor produces, so there is no residual the controller could chase
             but not express.
          2. Never exceed the braking envelope.
          3. Never command 0 < v < v_min. Below the floor the wheels do not
             turn, the rover does not move, and an integrator winds up against
             a stationary robot - which is the same failure the dead zone
             produced, arrived at from the other direction.

        THE BAND IS WIDENED BY THE MEASURED SPEED, AND THAT IS THE WHOLE FIX
        FOR A BIAS THAT NO GAIN REMOVES.
        ================================================================
        A FIXED band decides to cut using POSITION only, while the distance
        actually carried after the cut is set by the SPEED AT THAT MOMENT.
        Those are not the same thing, because the chassis lags its own command
        by `lag_s` for the entire deceleration ramp: while the envelope is
        walking the COMMAND down toward the floor, the REAL speed is still
        above it. So the move cuts while genuinely travelling near cruise and
        coasts `v_actual * lag_s` past the target.

        Measured, on a plant with lag_s = 0.15 s: every turn from 15 to 180 deg
        landed 3.0-4.7 deg PAST target, in the same direction every time,
        ~= w_peak * lag_s = 0.45 * 0.15 = 3.9 deg. Same direction at every
        angle is the signature of a BIAS, and a bias is the one error a retrace
        cannot cancel - it reappears identically on the way back. The envelope's
        own lag term does not catch it: that term makes the COMMAND stoppable,
        and the command was never the problem.

        With `v_meas` supplied the band becomes the distance this speed will
        actually carry. After the command goes to zero the drivetrain decays
        with the same first-order lag, so the carry is the integral of that
        decay, `v * lag_s` exactly - not `v^2/(2a)`, which is the distance
        under ACTIVE braking and does not apply once the command is zero.

        `v_meas` stays optional so a caller with no rate measurement keeps the
        old fixed-band behaviour rather than getting a silently different one.
        """
        rem = self.distance - s_travelled
        band = stop_band
        if v_meas is not None and self.lag_s > 0.0:
            vm = abs(float(v_meas))
            if vm > 0.0:
                band = max(band, vm * self.lag_s)
        if rem <= band:
            return 0.0
        v = min(max(0.0, v_desired), self._v_envelope(s_travelled))
        if 0.0 < v < self.v_min:
            v = self.v_min
        return v

    # -- public, native units (AngularProfile uses these unchanged) --------
    def at_time(self, t: float) -> ProfilePoint:
        """The ideal reference at time `t` since the move started."""
        return self._at_time(t)

    def v_envelope(self, s_travelled: float) -> float:
        """Fastest speed from which the move can still stop at `distance`."""
        return self._v_envelope(s_travelled)

    def command_v(self, v_desired: float, s_travelled: float,
                  stop_band: float, v_meas: Optional[float] = None) -> float:
        """Turn a desired speed into a commandable one."""
        return self._command_v(v_desired, s_travelled, stop_band, v_meas)


class TrapezoidalProfile(_Trapezoid):
    """Linear trapezoid. Units: distance mm, speed m/s, accel m/s^2.

    NOTE THE MIXED UNITS AND THAT THEY ARE INTENTIONAL: distances are in mm
    everywhere in this project (arena, residuals, tolerances) and speeds are in
    m/s everywhere (cmd_vel, CRUISE_MPS, the executor's clamps). Converting one
    of them at the boundary of this class would put a factor of 1000 somewhere
    a reader has to remember; converting it here, once, in the constructor,
    puts it somewhere a reader can see.
    """

    def __init__(self, distance_mm: float, v_cruise_mps: float,
                 a_max_mps2: float, v_min_mps: float = 0.0,
                 lag_s: float = 0.0):
        super().__init__(distance=float(distance_mm),
                         v_cruise=v_cruise_mps * 1000.0,
                         a_max=a_max_mps2 * 1000.0,
                         v_min=v_min_mps * 1000.0,
                         lag_s=lag_s)

    # Everything internal is mm and mm/s; expose m/s at the edges. These
    # override ONLY the public methods and delegate to the `_native` ones, so
    # no internal computation can pick up a converted value. See the comment
    # on `_Trapezoid._at_time`.
    def at_time(self, t: float) -> ProfilePoint:
        p = self._at_time(t)
        return ProfilePoint(t_s=p.t_s, s=p.s, v=p.v / 1000.0, phase=p.phase)

    def v_envelope(self, s_travelled_mm: float) -> float:
        return self._v_envelope(s_travelled_mm) / 1000.0

    def command_v(self, v_desired_mps: float, s_travelled_mm: float,
                  stop_band_mm: float,
                  v_meas_mps: Optional[float] = None) -> float:
        # v_meas crosses the same m/s -> mm/s boundary as v_desired. Passing it
        # through unconverted would compare metres against millimetres and
        # widen the band by a factor of 1000, which stops the move on the first
        # tick - a failure that looks like "the rover will not start".
        return self._command_v(
            v_desired_mps * 1000.0, s_travelled_mm, stop_band_mm,
            None if v_meas_mps is None else v_meas_mps * 1000.0) / 1000.0

    @property
    def v_peak_mps(self) -> float:
        return self.v_peak / 1000.0


class AngularProfile(_Trapezoid):
    """Angular trapezoid for a turn in place. Units: rad, rad/s, rad/s^2."""

    def __init__(self, angle_rad: float, w_cruise_radps: float,
                 alpha_max_radps2: float, w_min_radps: float = 0.0,
                 lag_s: float = 0.0):
        super().__init__(distance=abs(float(angle_rad)),
                         v_cruise=w_cruise_radps,
                         a_max=alpha_max_radps2,
                         v_min=w_min_radps,
                         lag_s=lag_s)


# ===========================================================================
# SECTION 8 -- ANTI-WINDUP AND RATE LIMITING
#
# Every integrator in this file goes through `integrate_bounded` and every
# actuator step through `rate_limit`. There are no other integrators and no
# other duty steps.
# ===========================================================================

def integrate_bounded(i_prev: float, err: float, dt: float, ki: float,
                      i_limit: float, *, saturated_sign: float = 0.0,
                      frozen: bool = False) -> float:
    """One conditional-integration step with a hard clamp.

    THREE PROTECTIONS, ALL NECESSARY:
      * `i_limit` clamps the accumulated term, so a stuck rover cannot store an
        unbounded correction and then release it as a lurch when it frees.
      * `saturated_sign` freezes integration IN THE DIRECTION OF SATURATION.
        When the actuator is already at HARD_MAX_DUTY, more integral buys no
        more output and only has to be unwound later - which is the classic
        overshoot after a slow start. Integration AWAY from saturation is
        still allowed, so recovery is immediate.
      * `frozen` stops it entirely. The distance controller uses this during
        the acceleration phase, where the tracking error is dominated by
        unmodelled motor lag rather than by any persistent disturbance;
        integrating that produces a step of extra speed exactly at the moment
        the profile reaches cruise.
    """
    if frozen or dt <= 0.0 or ki == 0.0:
        return clamp(i_prev, -i_limit, i_limit)
    if saturated_sign > 0.0 and err > 0.0:
        return clamp(i_prev, -i_limit, i_limit)
    if saturated_sign < 0.0 and err < 0.0:
        return clamp(i_prev, -i_limit, i_limit)
    return clamp(i_prev + ki * err * dt, -i_limit, i_limit)


def rate_limit(prev: float, target: float, max_step: float) -> float:
    """Move `prev` toward `target` by at most `max_step`."""
    if max_step <= 0.0:
        raise ValueError("max_step must be positive")
    d = target - prev
    if d > max_step:
        return prev + max_step
    if d < -max_step:
        return prev - max_step
    return target


# ===========================================================================
# SECTION 9 -- DUTY MIXING
# ===========================================================================

@dataclass(frozen=True)
class DutyCommand:
    """Four integer duties plus everything the caller needs to log or to feed
    back into the next control step.

    `saturated` is not decoration: it is the input to the next step's
    anti-windup. A mixer that clamps silently and a controller that keeps
    integrating are the two halves of one bug.
    """

    duty: Tuple[int, int, int, int]        # [FL, FR, RL, RR], /cmd_duty order
    v_mps: float                           # what was actually mixed, post-clamp
    omega_radps: float
    saturated: bool
    saturated_sign: float                  # +1 / -1 / 0, on the LINEAR axis
    rate_limited: bool
    wheels_below_floor: Tuple[bool, bool, bool, bool]
    """Wheels commanded non-zero but below `min_moving_duty`.

    NOT silently raised to the floor: on a differential command a genuinely
    slower side may sit below it, and forcing it up would corrupt the very
    differential that steers. Reported so the caller can log it, because "the
    inside wheel is not turning" is a real explanation for a curve that came
    out wrong."""

    notes: Tuple[str, ...] = ()


def mix_duty(v_mps: float, omega_radps: float, *,
             cal: Calibration,
             limits: Limits = DEFAULT_LIMITS,
             prev_duty: Tuple[int, int, int, int] = (0, 0, 0, 0),
             max_duty_step: Optional[float] = None) -> DutyCommand:
    """(v, omega) -> four integer wheel duties in /cmd_duty order.

    THE MAP, AND WHY IT IS TWO MEASURED PAIRS AND NOT GEOMETRY
    ---------------------------------------------------------
        common     = v / mps_per_duty                (duty percent)
        differential = omega / radps_per_duty        (duty percent)

    The angular half deliberately does NOT go through track width. On this
    chassis LR_WHEELS_DISTANCE is UNMEASURED - the operator measured 105 mm
    front-to-back, which is the wheelbase - and a skid steer's effective track
    is not its geometric one in any case, because four non-steered wheels must
    scrub to rotate. Measuring rad/s per unit differential duty directly gives
    the number that geometry was only ever a proxy for.

    SIGN CONVENTION (REP-103): +v is forward, +omega is counter-clockwise seen
    from above, so a positive omega slows the LEFT side and speeds the RIGHT:

        left  = common - differential
        right = common + differential

    `duty_forward_sign` is applied LAST, to all four wheels together, because
    it describes the flashed binary's polarity, not the geometry.

    HEADROOM POLICY: WHEN IT WILL NOT ALL FIT, HEADING WINS
    ------------------------------------------------------
    If |common| + |differential| exceeds the ceiling, the COMMON term is scaled
    down first and the differential is preserved. Losing speed costs time;
    losing the differential costs the heading, and a heading error integrates
    into a position error that grows for the rest of the leg. The executor's
    own bearing-tolerance note makes the same trade: up to 47 mm of lateral
    error over a 300 mm segment is what an uncorrected 9 deg of heading buys.
    """
    cal.require("mps_per_duty", "radps_per_duty", "min_moving_duty",
                "duty_forward_sign", "side_map_confirmed")

    notes: list = []
    v = clamp(float(v_mps), -limits.max_v_mps, limits.max_v_mps)
    w = clamp(float(omega_radps), -limits.max_omega_radps,
              limits.max_omega_radps)
    if v != v_mps or w != omega_radps:
        notes.append(f"request ({v_mps:+.3f} m/s, {omega_radps:+.3f} rad/s) "
                     f"clamped to ({v:+.3f}, {w:+.3f}) by Limits")

    common = v / cal.mps_per_duty          # type: ignore[operator]
    differential = w / cal.radps_per_duty  # type: ignore[operator]

    ceiling = float(limits.hard_max_duty)
    saturated = False

    # Differential first: it is the term we protect.
    if abs(differential) > ceiling:
        differential = _sign(differential) * ceiling
        common = 0.0
        saturated = True
        notes.append("differential alone exceeded the duty ceiling; the "
                     "linear term was dropped to preserve the turn")
    elif abs(common) + abs(differential) > ceiling:
        room = ceiling - abs(differential)
        common = _sign(common) * room
        saturated = True
        notes.append("linear term scaled to fit the duty ceiling; the "
                     "differential was preserved (heading wins)")

    left = common - differential
    right = common + differential

    # LEFT/RIGHT ASYMMETRY, APPLIED AS A RATIO CORRECTION.
    # `lr_asymmetry` is left_travel / right_travel at equal duty. Scaling the
    # left side by 1/sqrt(k) and the right by sqrt(k) corrects the RATIO by k
    # while leaving the geometric mean duty - and therefore the commanded
    # speed - unchanged. Putting the whole correction on one side would
    # silently change the speed as well as the balance, and the distance loop
    # would then spend its authority undoing a trim.
    k = cal.asymmetry_or_unity()
    if k <= 0:
        raise ValueError("lr_asymmetry must be positive")
    root = math.sqrt(k)
    left /= root
    right *= root

    tfl, tfr, trl, trr = cal.trims_or_unity()
    raw = [left * tfl, right * tfr, left * trl, right * trr]

    fsign = float(cal.duty_forward_sign)   # type: ignore[arg-type]
    raw = [d * fsign for d in raw]

    # SCALE THE GROUP RATHER THAN CLIPPING ONE WHEEL. The trims and the
    # asymmetry correction can push a wheel over the ceiling even after the
    # headroom policy above (a 10 % imbalance at 60 % duty is 63 %). Clipping
    # only the offender would destroy exactly the left/right ratio that was
    # just corrected, and a duty imbalance is a STEER: the rover would curve
    # away at full speed while every commanded number still looked right.
    # Scaling all four preserves both the ratio and the differential.
    peak = max(abs(d) for d in raw)
    if peak > ceiling:
        raw = [d * ceiling / peak for d in raw]
        saturated = True
        notes.append(f"per-wheel duty peaked at {peak:.1f} after trims; all "
                     "four scaled together to preserve the ratio")

    floor = float(cal.min_moving_duty)     # type: ignore[arg-type]
    below = tuple(0.0 < abs(d) < floor for d in raw)

    step = limits.max_duty_step if max_duty_step is None else max_duty_step
    rate_limited = False
    out = []
    for i, d in enumerate(raw):
        limited = rate_limit(float(prev_duty[i]), d, step)
        if abs(limited - d) > 1e-9:
            rate_limited = True
        limited = clamp(limited, -ceiling, ceiling)
        if abs(d) > ceiling + 1e-9:
            saturated = True
        out.append(_round_duty(limited))

    # The saturation SIGN is reported on the linear axis in the chassis's own
    # sense of forward, so an anti-windup upstream can freeze the right half of
    # its integrator without knowing anything about wiring polarity.
    sat_sign = 0.0
    if saturated:
        sat_sign = _sign(v) if v != 0.0 else 0.0

    return DutyCommand(duty=(out[0], out[1], out[2], out[3]),
                       v_mps=v, omega_radps=w,
                       saturated=saturated, saturated_sign=sat_sign,
                       rate_limited=rate_limited,
                       wheels_below_floor=below,   # type: ignore[arg-type]
                       notes=tuple(notes))


def stop_command() -> DutyCommand:
    """All four wheels zero. The only command that is always safe to send in
    any state, on any firmware, with any calibration - so it needs none."""
    return DutyCommand(duty=(0, 0, 0, 0), v_mps=0.0, omega_radps=0.0,
                       saturated=False, saturated_sign=0.0,
                       rate_limited=False,
                       wheels_below_floor=(False, False, False, False))


# ===========================================================================
# SECTION 10 -- THE DISTANCE CONTROLLER
# ===========================================================================

@dataclass(frozen=True)
class DistanceState:
    """Loop state for one straight move. Immutable; `step` returns a new one."""

    t0: Optional[float] = None       # when the move started (caller's clock)
    t: Optional[float] = None        # time of the last step
    s_mm: float = 0.0                # measured travel so far, signed +forward
    integ: float = 0.0               # integral term, m/s
    v_cmd: float = 0.0               # last commanded speed, m/s
    done: bool = False
    reason: str = ""


@dataclass(frozen=True)
class DistanceOutput:
    v_mps: float
    reference: ProfilePoint
    error_mm: float
    done: bool
    reason: str
    residual_mm: float


class DistanceController:
    """Profile feedforward plus a BOUNDED trim, closing on encoder travel.

    THE LAW
    -------
        s_ref(t), v_ref(t) = profile.at_time(t - t0)        feedforward
        e            = s_ref(t) - s_meas                     mm
        trim         = clamp(Kp*e + I, +/- trim_max)         m/s
        I           <- bounded, frozen while accelerating and while saturated
        v_cmd        = profile.command_v(v_ref + trim, s_meas, stop_band)

    FEEDFORWARD CARRIES THE MOTION, FEEDBACK ONLY TRIMS IT. That split is the
    whole design. A pure PID on remaining distance would fight the profile:
    early in the move the error is large by construction (the profile has not
    got there yet either), and a PID reads that as something to fix, commanding
    exactly the step the ramp exists to avoid. So the trim is bounded to a
    fraction of cruise and can never become the primary signal.

    THE GAINS, AND WHERE THEY COME FROM
    -----------------------------------
    `kp_per_s` is expressed as a RECIPROCAL TIME CONSTANT, so it reads as a
    decision instead of as a number: kp = 1/T means a standing error is closed
    with time constant T. T = 1.0 s is chosen to be SLOWER than the profile's
    own acceleration time (0.72 s at a_max = 0.25, v = 0.18), so the trim
    cannot fight the ramp it is riding on. Faster than the profile and the two
    loops argue; much slower and a real disturbance is never corrected.

    `ki_per_s2` = 0.2 with the integral clamped to 20 mm/s. The integral exists
    for ONE disturbance: a persistent duty-to-speed error, i.e. `mps_per_duty`
    measured on a fuller battery than the one in the rover. It is frozen during
    acceleration (where the error is model lag, not disturbance) and frozen
    into saturation. At the clamp it can contribute at most ~11 % of cruise.
    """

    def __init__(self, profile: TrapezoidalProfile, *,
                 direction: float = 1.0,
                 kp_per_s: float = 1.0,
                 ki_per_s2: float = 0.2,
                 trim_max_frac: float = 0.30,
                 integ_max_mps: float = 0.02,
                 arrive_tol_mm: float = 5.0,
                 stop_band_mm: Optional[float] = None,
                 dt_nominal: float = CONTROL_DT):
        if profile.distance < 0:
            raise ValueError("profile distance must be >= 0")
        self.profile = profile
        self.direction = 1.0 if direction >= 0 else -1.0
        self.kp = float(kp_per_s) / 1000.0     # mm of error -> m/s of trim
        self.ki = float(ki_per_s2) / 1000.0
        self.trim_max = trim_max_frac * profile.v_peak_mps
        self.integ_max = float(integ_max_mps)
        self.dt_nominal = float(dt_nominal)

        # THE ANTI-HUNTING BAND, DERIVED AND NOT TYPED. Two physical floors:
        #   v_min * dt   - one control tick at the duty floor. A target band
        #                  narrower than this is one the controller can chase
        #                  forever and never land on, because every correction
        #                  it can express overshoots the band it aims at.
        #   v_min * lag  - what is carried after the command is cut at the
        #                  floor speed. Cutting inside this guarantees an
        #                  overshoot no gain can prevent.
        # The band is the widest of these and the caller's tolerance, and
        # whatever is left over is REPORTED as a residual, never hunted.
        unclosable = profile.v_min * max(self.dt_nominal, profile.lag_s)
        self.stop_band = (max(float(arrive_tol_mm), unclosable)
                          if stop_band_mm is None else float(stop_band_mm))

    def start(self, t: float) -> DistanceState:
        return DistanceState(t0=t, t=t)

    def step(self, state: DistanceState, t: float, travel_mm: float,
             *, saturated_sign: float = 0.0) -> Tuple[DistanceState, DistanceOutput]:
        """One control tick.

        `travel_mm` is CUMULATIVE SIGNED travel since the move started, in the
        chassis's forward sense, from `travel_mm_from_ticks`. It is never a
        twist and never a velocity.
        """
        if state.t0 is None:
            raise ValueError("call start() before step()")

        s_along = self.direction * float(travel_mm)   # progress along the move
        dt = 0.0 if state.t is None else (t - state.t)
        if dt < 0.0:
            # Time went backwards. Hold the last command rather than integrate
            # a negative interval; a clock that jumped is not a measurement.
            return state, DistanceOutput(
                v_mps=state.v_cmd,
                reference=self.profile.at_time(max(0.0, (state.t or t) - state.t0)),
                error_mm=0.0, done=state.done,
                reason=state.reason or "time went backwards; holding",
                residual_mm=self.profile.distance - s_along)

        ref = self.profile.at_time(t - state.t0)
        err = ref.s - s_along
        remaining = self.profile.distance - s_along

        if state.done or remaining <= self.stop_band:
            new = replace(state, t=t, s_mm=s_along, v_cmd=0.0, done=True,
                          reason=state.reason or "arrived")
            return new, DistanceOutput(v_mps=0.0, reference=ref,
                                       error_mm=err, done=True,
                                       reason=new.reason,
                                       residual_mm=remaining)

        integ = integrate_bounded(
            state.integ, err, dt, self.ki, self.integ_max,
            saturated_sign=saturated_sign,
            # Frozen while the profile is still ramping up: the error there is
            # dominated by motor lag we did not model, and integrating it
            # delivers a step of surplus speed exactly at cruise onset.
            frozen=(ref.phase == PHASE_ACCEL))

        trim = clamp(self.kp * err + integ, -self.trim_max, self.trim_max)

        # MEASURED speed, by finite difference on the same cumulative travel
        # the rest of this function trusts. Not from /odom_raw's twist, whose
        # linear.x sign is inverted relative to its own pose - a guard built on
        # that twist aborts every CORRECT move, which is how the first deadband
        # sweep was lost.
        #
        # It is a one-tick difference and therefore noisy, which is exactly why
        # it is used only to WIDEN the cut band (via max() inside command_v)
        # and never to narrow it: a noise spike can then only stop the move
        # slightly early, never carry it past the target. A zero or negative
        # difference - a stalled wheel, a dropped frame - simply leaves the
        # fixed band in force.
        v_meas_mps = 0.0
        if dt > 0.0:
            v_meas_mps = (s_along - float(state.s_mm)) / dt / 1000.0

        v = self.profile.command_v(ref.v + trim, s_along, self.stop_band,
                                   v_meas_mps=abs(v_meas_mps))

        new = replace(state, t=t, s_mm=s_along, integ=integ, v_cmd=v)
        return new, DistanceOutput(v_mps=v, reference=ref, error_mm=err,
                                   done=False, reason="", residual_mm=remaining)


# ===========================================================================
# SECTION 11 -- HEADING HOLD (straight-line runs)
# ===========================================================================

@dataclass(frozen=True)
class HeadingState:
    yaw0: Optional[float] = None
    t: Optional[float] = None
    omega_cmd: float = 0.0


@dataclass(frozen=True)
class HeadingOutput:
    omega_radps: float
    error_rad: float
    differential_duty: Optional[float]
    """The bias this correction becomes, in duty percent, when the actuation
    map is known. Reported so an operator can compare it against B8B's measured
    +/-6 counts of +/-23 % differential - the number that is known to have held
    a line on this chassis."""


class HeadingHold:
    """Hold the heading a straight run started with. P + rate damping.

    THE LAW
    -------
        e     = wrap_pi(yaw0 - yaw)                   rad
        omega = clamp(Kp*e - Kd*omega_meas, +/- omega_max)

    WHY THERE IS NO INTEGRAL HERE, DELIBERATELY
    -------------------------------------------
    An integrator on heading would integrate GYRO BIAS into a permanent steer
    over a long leg, and `gyro_scale` is unmeasured, so the size of that steer
    is unknown too. A straight-line hold has no persistent disturbance that
    needs an integral in the first place - a crooked chassis shows up as
    `lr_asymmetry` in the mixer, which is a measurement, not a wind-up. P-only
    also cannot wind up, which matters because this is the term that keeps
    running while the linear axis is saturated.

    WHY THE DAMPING TERM IS RATE FEEDBACK AND NOT A DERIVATIVE
    ----------------------------------------------------------
    `-Kd * omega_meas` uses the gyro's OWN rate output. It never differentiates
    a position signal, so there is no numerical differentiation of a noisy
    integral and no filter to tune. It is also scale-honest: both terms scale
    together if `gyro_scale` turns out to be off, which is exactly why this
    controller can run without it (below).

    WHY THIS RUNS WITHOUT A MEASURED gyro_scale
    -------------------------------------------
    It regulates an error to ZERO. Multiply the sensor by any positive
    constant and the equilibrium is unchanged; only the effective loop gain
    moves, and the clamp bounds the consequences of that. `TurnController`, by
    contrast, must arrive at a SPECIFIC angle, so it refuses without the scale.
    THE ONE THING THE SCALE CANNOT EXCUSE IS A SIGN: an inverted heading hold
    is POSITIVE FEEDBACK and the rover spirals while every logged number stays
    plausible. That is why the mixer refuses without `side_map_confirmed`.

    GAINS: Kp = 1.2 rad/s per rad and the clamp 0.25 rad/s are carried over
    unchanged from `fpms_missions.HEADING_KP` / `HEADING_CORR_MAX_RADPS`, which
    are themselves the shape of B8B's `_KPH = 10` clamped to +/-6 duty counts.
    They are the only heading numbers in this project with any history behind
    them. 1/Kp = 0.83 s: on the ideal velocity-commanded model that is a
    first-order response, which cannot overshoot at all; Kd = 0.15 s exists
    only to damp the actuator lag the ideal model omits, and contributes at
    most 0.14 rad/s at the maximum plausible yaw rate.
    """

    def __init__(self, *, kp: float = 1.2, kd: float = 0.15,
                 omega_max_radps: float = 0.25):
        if kp <= 0 or kd < 0 or omega_max_radps <= 0:
            raise ValueError("kp > 0, kd >= 0, omega_max > 0")
        self.kp = float(kp)
        self.kd = float(kd)
        self.omega_max = float(omega_max_radps)

    def start(self, t: float, yaw_rad: float) -> HeadingState:
        return HeadingState(yaw0=float(yaw_rad), t=t)

    def step(self, state: HeadingState, t: float, yaw_rad: float,
             omega_meas_radps: float = 0.0,
             cal: Optional[Calibration] = None
             ) -> Tuple[HeadingState, HeadingOutput]:
        if state.yaw0 is None:
            raise ValueError("call start() before step()")
        err = wrap_pi(state.yaw0 - float(yaw_rad))
        w = clamp(self.kp * err - self.kd * float(omega_meas_radps),
                  -self.omega_max, self.omega_max)
        bias = None
        if cal is not None and cal.radps_per_duty:
            bias = w / cal.radps_per_duty
        return (replace(state, t=t, omega_cmd=w),
                HeadingOutput(omega_radps=w, error_rad=err,
                              differential_duty=bias))


# ===========================================================================
# SECTION 12 -- TURN IN PLACE
# ===========================================================================

@dataclass(frozen=True)
class TurnState:
    t0: Optional[float] = None
    t: Optional[float] = None
    yaw0: Optional[float] = None
    turned_rad: float = 0.0
    omega_cmd: float = 0.0
    settling_since: Optional[float] = None
    latched: bool = False
    """Set the first time the turn enters the terminal band, and NEVER
    cleared. This is the anti-hunting guarantee: once inside, the controller
    commits to settling and reports the residual rather than re-opening a
    correction it may not be able to express. See `TurnController`."""
    done: bool = False
    reason: str = ""


@dataclass(frozen=True)
class TurnOutput:
    omega_radps: float
    turned_rad: float
    error_rad: float
    reference: ProfilePoint
    done: bool
    reason: str
    residual_rad: float

    @property
    def residual_deg(self) -> float:
        return math.degrees(self.residual_rad)


class TurnController:
    """Profiled turn in place, closing on gyro-integrated heading.

    THE LAW
    -------
        theta_ref(t), w_ref(t) = profile.at_time(t - t0)
        e     = theta_target_effective - |turned|
        w_cmd = sign * clamp_env( w_ref + Kp*e - Kd*|w_meas| )

    "PERFECT TURNING" = ARRIVING WITHOUT OVERSHOOT AND WITHOUT HUNTING, AND
    THOSE ARE TWO DIFFERENT PROBLEMS WITH TWO DIFFERENT ANSWERS.

    NO OVERSHOOT -> DAMPING, DELIBERATELY OVERDAMPED (zeta = 1.2).
    Model the plant near the target as an integrator behind a first-order
    actuator lag tau: theta'' * tau + theta' = w_cmd. With w_cmd = Kp*e - Kd*theta'
    the closed-loop characteristic polynomial is

        tau*s^2 + (1 + Kd)*s + Kp = 0
        w_n  = sqrt(Kp / tau)
        zeta = (1 + Kd) / (2 * sqrt(Kp * tau))
        =>   Kd = 2*zeta*sqrt(Kp*tau) - 1

    With Kp = 2.0 1/s (a 0.5 s residual time constant, i.e. the trim settles
    well inside the profile's own decel ramp), tau = 0.15 s and zeta = 1.2:

        Kd = 2*1.2*sqrt(2.0*0.15) - 1 = 2.4*0.548 - 1 = 0.315 s

    WHY OVERDAMPED AND NOT CRITICAL: an overshoot is a BIAS. It appears in the
    same direction every time, so a retrace cannot cancel it - it reappears
    identically on the way back. That is the exact reasoning behind B8B's
    `_COAST = 0.93`, and it is worth more than the fraction of a second that
    zeta = 1.2 costs over zeta = 1.0. WHY THE UNMEASURED tau IS SAFE: zeta
    varies as 1/sqrt(tau), so if the true actuator is FASTER than the assumed
    0.15 s the loop becomes MORE damped, not less. Assuming a slow actuator is
    the safe direction to be wrong in, and it is the direction assumed here.

    NO HUNTING -> A STRUCTURAL TERMINAL BAND, NOT A TUNING HOPE.
    The smallest angle one control tick can produce is
    `radps_per_duty * min_moving_duty * dt`. Inside that band there is no
    correction the hardware can express that does not overshoot the band it is
    aiming at, so the controller commands ZERO and REPORTS the residual. It
    does not try. That is what makes "no hunting" a property of the design
    rather than of the gains.

    NO COAST CUT, AND THAT IS THE POINT. The coast trick is what you do when
    you cannot decelerate. This profile can, so cutting early would only trade
    a repeatable overshoot for a repeatable undershoot. `coast_deg` is parsed
    for diagnostics and not applied - see `Calibration.coast_deg`.

    SETTLE. Done requires BOTH |e| <= tol AND |w_meas| <= still, held for
    `settle_hold_s`. The stillness threshold is 0.02 rad/s, above the board's
    +/-0.01 rad/s deadband: A HEALTHY PARKED GYRO READS EXACTLY 0.0, so zero
    means still, not broken. And nothing here may be used the other way round:
    never command a turn to find out whether the rover is alive - it destroys
    the heading the mission depends on and proves nothing that watching the
    pose does not.
    """

    def __init__(self, angle_rad: float, cal: Calibration, *,
                 limits: Limits = DEFAULT_LIMITS,
                 w_cruise_radps: float = 0.45,
                 kp: float = 2.0,
                 tau_actuator_s: float = 0.15,
                 zeta: float = 1.2,
                 tol_rad: float = math.radians(1.0),
                 settle_hold_s: float = 0.20,
                 dt_nominal: float = CONTROL_DT,
                 timeout_s: Optional[float] = None):
        # A turn closes entirely on the gyro, so the scale is load-bearing.
        cal.require("gyro_scale", "radps_per_duty", "min_moving_duty",
                    "duty_forward_sign", "side_map_confirmed")
        self.cal = cal
        self.limits = limits
        self.sign = 1.0 if angle_rad >= 0 else -1.0

        # NO COAST CUT. The command is the full target.
        #
        # B8B's `_COAST = 0.93` cut turns at 93 % and let momentum finish,
        # because at TRN = 50 duty with no way to command anything slower there
        # was no other way to avoid a repeatable overshoot. This controller
        # decelerates under control: the angular profile's braking envelope
        # brings the commanded rate to the duty floor at the target, so what
        # remains to be carried is one tick at the floor rate, which the
        # terminal band already covers. Cutting early ON TOP of that would
        # produce a repeatable UNDERSHOOT - the same bias, mirrored, and a bias
        # is the one error a retrace cannot cancel.
        self.target_rad = abs(float(angle_rad))
        self.commanded_rad = self.target_rad

        w_min = cal.radps_per_duty * cal.min_moving_duty  # type: ignore[operator]
        self.profile = AngularProfile(
            self.commanded_rad,
            min(w_cruise_radps, limits.max_omega_radps),
            limits.alpha_max_radps2,
            w_min_radps=w_min,
            lag_s=float(tau_actuator_s))

        self.kp = float(kp)
        self.kd = 2.0 * float(zeta) * math.sqrt(self.kp * float(tau_actuator_s)) - 1.0
        if self.kd < 0.0:
            # A very fast actuator or a very low Kp can make the derived Kd
            # negative, which would be POSITIVE rate feedback. Clamp to zero
            # and note it rather than inverting the damping term.
            self.kd = 0.0
        self.zeta = float(zeta)
        self.tau = float(tau_actuator_s)

        # THE TERMINAL BAND, DERIVED FROM THREE PHYSICAL FLOORS AND NOT TYPED.
        #   err_floor  = w_min * dt   - the smallest angle one control tick at
        #                the duty floor can produce. Nothing smaller is
        #                expressible, so nothing smaller may be chased.
        #   coast_floor = w_min * tau - the angle carried after the command is
        #                cut at the floor rate, from the same first-order lag
        #                the damping is designed against. THIS IS WHAT REPLACES
        #                A COAST CONSTANT: computed from the rate we actually
        #                cut at, not measured by cutting from cruise.
        # The tolerance is the widest of these and the caller's request, so a
        # minimum-size correction can never overshoot the band it aims at.
        self.err_floor_rad = w_min * float(dt_nominal)
        self.coast_floor_rad = w_min * float(tau_actuator_s)
        self.tol_rad = max(float(tol_rad), self.err_floor_rad,
                           self.coast_floor_rad)
        # Bound on a correction issued after an overshoot. It is a trim, not a
        # move: a quarter of turn speed is enough to walk back a degree or two
        # and far too little to become a second overshoot.
        self.w_correct_max = max(w_min, 0.25 * self.profile.v_peak)
        self.settle_hold_s = float(settle_hold_s)
        self.still_radps = max(0.02, 2.0 * GYRO_PARKED_DEADBAND_RADPS)
        self.timeout_s = timeout_s

    def start(self, t: float, yaw_rad: float) -> TurnState:
        return TurnState(t0=t, t=t, yaw0=float(yaw_rad))

    def step(self, state: TurnState, t: float, yaw_rad: float,
             omega_meas_radps: float = 0.0,
             gyro_scaled: bool = True) -> Tuple[TurnState, TurnOutput]:
        """One control tick.

        `gyro_scaled` must be True: pass `GyroState.scaled` straight through.
        An unscaled heading integral is a plausible number in the wrong units,
        which is the failure mode this whole file is written against.
        """
        if state.t0 is None or state.yaw0 is None:
            raise ValueError("call start() before step()")
        if not gyro_scaled:
            raise MotionRefusal(
                REFUSE_GYRO_UNSCALED,
                "the heading integral was produced without a measured "
                "gyro_scale. Heading hold tolerates that (it regulates to "
                "zero); arriving at a SPECIFIC angle does not.")

        # SIGNED, not abs(). With abs(), a turn driven the WRONG WAY reaches
        # the target magnitude just as happily as a correct one and records the
        # opposite angle - and it looks like a success in every log. This is
        # the executor's own hard-won check, kept.
        #
        # AND NOT wrap_pi() EITHER. `yaw_rad` is a CUMULATIVE GYRO INTEGRAL,
        # not a compass heading: `integrate_gyro` never wraps it, precisely so
        # that a turn larger than pi is representable. Wrapping here made a
        # 180 deg turn integrate as -180 and drove the rover 539 deg before it
        # "settled" - caught in simulation, and it would have looked like a
        # spin with a plausible log.
        turned = self.sign * (float(yaw_rad) - state.yaw0)
        err = self.commanded_rad - turned
        residual = self.target_rad - turned
        ref = self.profile.at_time(t - state.t0)

        if state.done:
            return replace(state, t=t, turned_rad=turned, omega_cmd=0.0), \
                TurnOutput(0.0, turned, err, ref, True, state.reason, residual)

        if self.timeout_s is not None and (t - state.t0) > self.timeout_s:
            new = replace(state, t=t, turned_rad=turned, omega_cmd=0.0,
                          done=True, reason="turn timeout")
            return new, TurnOutput(0.0, turned, err, ref, True, new.reason,
                                   residual)

        # -- the terminal band, and the LATCH that makes it hunt-free -------
        # `latched` is set the first time the band is entered and never
        # cleared. Once inside, the controller commands zero, waits for
        # stillness, and REPORTS whatever residual is left. It does not
        # re-open a correction. That is what makes "no hunting" a property of
        # the structure: there is no loop left that could oscillate.
        within = abs(err) <= self.tol_rad
        latched = state.latched or within
        still = abs(float(omega_meas_radps)) <= self.still_radps
        if latched:
            since = state.settling_since if state.settling_since is not None else t
            if still and (t - since) >= self.settle_hold_s:
                new = replace(state, t=t, turned_rad=turned, omega_cmd=0.0,
                              settling_since=since, latched=True, done=True,
                              reason="settled")
                return new, TurnOutput(0.0, turned, err, ref, True,
                                       "settled", residual)
            new = replace(state, t=t, turned_rad=turned, omega_cmd=0.0,
                          settling_since=since, latched=True)
            return new, TurnOutput(0.0, turned, err, ref, False,
                                   "in band, settling", residual)

        if abs(err) < self.err_floor_rad:
            # Smaller than one tick at the duty floor can produce. Any command
            # that moves at all overshoots this. Stop and report.
            new = replace(state, t=t, turned_rad=turned, omega_cmd=0.0,
                          done=True,
                          reason="residual is below the duty floor; reported, "
                                 "not chased")
            return new, TurnOutput(0.0, turned, err, ref, True, new.reason,
                                   residual)

        if err > 0.0:
            # -- approaching: feedforward + damped trim, capped by the
            #    braking envelope so it arrives at rest rather than at speed.
            w_des = (ref.v + self.kp * err
                     - self.kd * abs(float(omega_meas_radps)))
            # The MEASURED rate decides when to cut, not the commanded one.
            # See _command_v: the chassis trails its command by tau for the
            # whole deceleration ramp, so cutting on position alone cuts while
            # still near cruise and carries w*tau past the target - the same
            # sign at every angle, which is a bias a retrace cannot cancel.
            w_mag = self.profile.command_v(w_des, max(0.0, turned),
                                           stop_band=self.tol_rad,
                                           v_meas=abs(float(omega_meas_radps)))
            w = self.sign * w_mag
        else:
            # -- OVERSHOT WITHOUT EVER ENTERING THE BAND. Only a much slower
            #    actuator than the damping assumed can produce this, and when
            #    it does, refusing to correct would leave a heading error the
            #    whole mission carries. So: a BOUNDED, floored, opposite-sense
            #    trim - not a second profiled move. It runs at most until the
            #    band is entered, and the latch above then closes the loop for
            #    good, so this cannot become an oscillation.
            w_mag = clamp(self.kp * abs(err), self.profile.v_min,
                          self.w_correct_max)
            w = -self.sign * w_mag
        w = clamp(w, -self.limits.max_omega_radps, self.limits.max_omega_radps)

        new = replace(state, t=t, turned_rad=turned, omega_cmd=w,
                      settling_since=None, latched=latched)
        return new, TurnOutput(w, turned, err, ref, False, "", residual)


# ===========================================================================
# SECTION 13 -- COMPOSITION
#
# The two things a caller actually wants: drive this far in a straight line,
# and turn this far on the spot. Each one threads its own saturation back into
# its own anti-windup, which is the coupling that must not be left to the
# caller to remember.
# ===========================================================================

@dataclass(frozen=True)
class BreakawayKick:
    """An optional brief over-duty pulse to break static friction at start.

    OFF BY DEFAULT, AND IT IS A HYPOTHESIS RATHER THAN A MEASUREMENT.

    THE ARGUMENT FOR IT. Static friction exceeds kinetic friction. A profile
    that starts at exactly the duty floor may fail to break away at all, sit
    still while the integrator does nothing useful, and then lurch when
    something finally gives.

    THE ARGUMENT AGAINST IT, WHICH IS STRONGER THAN IT LOOKS. The floor this
    controller starts at is `min_moving_duty`, and if that number was measured
    the way it should be - the duty at which a STATIONARY loaded rover first
    breaks away - then it ALREADY INCLUDES the breakaway margin and a kick is
    redundant. A kick is only justified when the measured floor is a SUSTAINED
    floor (the lowest duty that keeps an already-rolling rover rolling), which
    is a different and lower number.

    SO THE REAL FIX IS A MEASUREMENT, NOT A CONSTANT: record which of the two
    floors was measured. If only one number exists, treat it as the breakaway
    floor and leave this disabled. Enable this only after watching the rover
    fail to start from a floor-duty command.

    WHY 140 / 220 ms IS NOT THE DEFAULT HERE. Those values are quoted around
    this repo as golden B8B constants and are not: they come from
    firmware_linorobot/moves_main.cpp, where BASE_DUTY is 70. A kick to exactly
    2x the base duty under a profile whose base is near the floor is a
    completely different impulse - 140 would be more than twice HARD_MAX_DUTY.
    Both fields below are UNMEASURED and both are derived from THIS
    controller's own floor, not carried across from that one.
    """

    enabled: bool = False

    duty: Optional[float] = None
    """Kick amplitude in duty percent. None derives `kick_ratio * floor`.
    UNMEASURED."""

    kick_ratio: float = 2.0
    """Derived amplitude as a multiple of `min_moving_duty`. UNMEASURED - 2x
    is chosen because it is the smallest multiple that is unambiguously more
    than the floor while staying small in absolute terms (a floor of 8 gives a
    kick of 16, not 140)."""

    duration_s: float = 0.12
    """UNMEASURED. Three ticks at 25 Hz - long enough for the board's 50 Hz
    control task to apply it six times, short enough that if it is wrong the
    rover has travelled a few millimetres, not a few hundred. NOT the 220 ms
    from the base-70 regime."""

    def amplitude(self, cal: "Calibration", limits: Limits) -> float:
        """Kick duty, bounded by HARD_MAX_DUTY. Requires a measured floor."""
        cal.require("min_moving_duty")
        base = (self.kick_ratio * float(cal.min_moving_duty)  # type: ignore[arg-type]
                if self.duty is None else float(self.duty))
        return clamp(base, 0.0, float(limits.hard_max_duty))

    def active(self, elapsed_s: float) -> bool:
        return self.enabled and 0.0 <= elapsed_s < self.duration_s


NO_KICK = BreakawayKick()


@dataclass(frozen=True)
class StraightRunState:
    distance: DistanceState = field(default_factory=DistanceState)
    heading: HeadingState = field(default_factory=HeadingState)
    duty: Tuple[int, int, int, int] = (0, 0, 0, 0)
    saturated_sign: float = 0.0


@dataclass(frozen=True)
class StraightRunOutput:
    command: DutyCommand
    distance: DistanceOutput
    heading: HeadingOutput
    done: bool


class StraightRun:
    """A profiled straight leg: distance closed on ticks, heading on the gyro.

    `distance_mm` is SIGNED - negative drives backwards, and the heading hold
    still holds the SAME heading (a reversing rover does not turn round).
    """

    def __init__(self, distance_mm: float, cal: Calibration, *,
                 limits: Limits = DEFAULT_LIMITS,
                 v_cruise_mps: float = 0.18,
                 hold_heading: bool = True,
                 kick: BreakawayKick = NO_KICK,
                 tau_actuator_s: float = 0.15,
                 dt_nominal: float = CONTROL_DT,
                 **distance_kwargs):
        needed = ["counts_per_mm", "mps_per_duty", "min_moving_duty",
                  "duty_forward_sign", "side_map_confirmed"]
        if hold_heading:
            needed.append("radps_per_duty")
        cal.require(*needed)
        self.cal = cal
        self.limits = limits
        self.direction = 1.0 if distance_mm >= 0 else -1.0

        # NO COAST CUT. See TurnController for the argument and
        # Calibration.coast_mm for why the recorded number is from the wrong
        # regime anyway. The command is the full distance.
        self.target_mm = abs(float(distance_mm))

        v_min = cal.mps_per_duty * cal.min_moving_duty  # type: ignore[operator]
        profile = TrapezoidalProfile(
            self.target_mm,
            min(v_cruise_mps, limits.max_v_mps),
            limits.a_max_mps2,
            v_min_mps=v_min,
            lag_s=float(tau_actuator_s))
        self.profile = profile
        self.distance = DistanceController(profile, direction=1.0,
                                           dt_nominal=dt_nominal,
                                           **distance_kwargs)
        self.heading = HeadingHold() if hold_heading else None

        # TIE THE DUTY RATE LIMIT TO a_max SO THERE IS ONE ACCELERATION LIMIT
        # AND NOT TWO. Without this the rate limiter quietly becomes the real
        # acceleration limit whenever it is the tighter of the pair, and the
        # profile a test verified would not be the profile the rover ran.
        implied = limits.a_max_mps2 * dt_nominal / cal.mps_per_duty  # type: ignore[operator]
        self.max_duty_step = min(limits.max_duty_step, max(1.0, implied))

        self.kick = kick
        self.kick_duty = kick.amplitude(cal, limits) if kick.enabled else 0.0
        self.kick_v_mps = self.kick_duty * cal.mps_per_duty  # type: ignore[operator]

    def start(self, t: float, yaw_rad: float = 0.0) -> StraightRunState:
        return StraightRunState(
            distance=self.distance.start(t),
            heading=(self.heading.start(t, yaw_rad) if self.heading
                     else HeadingState()))

    def step(self, state: StraightRunState, t: float, travel_mm: float,
             yaw_rad: float = 0.0, omega_meas_radps: float = 0.0
             ) -> Tuple[StraightRunState, StraightRunOutput]:
        """One control tick.

        `travel_mm` is signed cumulative travel since `start`, in the chassis's
        forward sense (from `travel_mm_from_ticks`). NOT a twist, NOT a speed.
        """
        dstate, dout = self.distance.step(
            state.distance, t, self.direction * float(travel_mm),
            saturated_sign=state.saturated_sign)

        if self.heading is not None:
            hstate, hout = self.heading.step(state.heading, t, yaw_rad,
                                             omega_meas_radps, self.cal)
        else:
            hstate, hout = state.heading, HeadingOutput(0.0, 0.0, None)

        if dout.done:
            cmd = stop_command()
        else:
            v = dout.v_mps
            step = self.max_duty_step
            elapsed = t - (state.distance.t0 if state.distance.t0 is not None else t)
            if self.kick.active(elapsed):
                # THE KICK BYPASSES THE RATE LIMITER, WHICH IS THE ONLY WAY IT
                # CAN MEAN ANYTHING - a breakaway impulse that ramps over eight
                # ticks is not a breakaway impulse. What bounds the lurch is
                # the AMPLITUDE, which is a small multiple of the duty floor
                # (typically well under 20 %), not the ramp. This is the one
                # place in this file where a duty step is not rate limited, and
                # it is disabled by default.
                v = max(v, self.kick_v_mps)
                step = max(step, self.kick_duty)
            cmd = mix_duty(self.direction * v, hout.omega_radps,
                           cal=self.cal, limits=self.limits,
                           prev_duty=state.duty,
                           max_duty_step=step)

        new = StraightRunState(distance=dstate, heading=hstate,
                               duty=cmd.duty,
                               saturated_sign=cmd.saturated_sign * self.direction)
        return new, StraightRunOutput(command=cmd, distance=dout,
                                      heading=hout, done=dout.done)


@dataclass(frozen=True)
class TurnInPlaceState:
    turn: TurnState = field(default_factory=TurnState)
    duty: Tuple[int, int, int, int] = (0, 0, 0, 0)


@dataclass(frozen=True)
class TurnInPlaceOutput:
    command: DutyCommand
    turn: TurnOutput
    done: bool


class TurnInPlace:
    """`TurnController` plus the mixer, with the duty rate limit tied to
    alpha_max for the same reason `StraightRun` ties it to a_max."""

    def __init__(self, angle_rad: float, cal: Calibration, *,
                 limits: Limits = DEFAULT_LIMITS,
                 kick: BreakawayKick = NO_KICK,
                 dt_nominal: float = CONTROL_DT, **turn_kwargs):
        self.cal = cal
        self.limits = limits
        self.turn = TurnController(angle_rad, cal, limits=limits,
                                   dt_nominal=dt_nominal, **turn_kwargs)
        implied = limits.alpha_max_radps2 * dt_nominal / cal.radps_per_duty  # type: ignore[operator]
        self.max_duty_step = min(limits.max_duty_step, max(1.0, implied))
        self.kick = kick
        self.kick_duty = kick.amplitude(cal, limits) if kick.enabled else 0.0
        self.kick_radps = self.kick_duty * cal.radps_per_duty  # type: ignore[operator]

    def start(self, t: float, yaw_rad: float) -> TurnInPlaceState:
        return TurnInPlaceState(turn=self.turn.start(t, yaw_rad))

    def step(self, state: TurnInPlaceState, t: float, yaw_rad: float,
             omega_meas_radps: float = 0.0, gyro_scaled: bool = True
             ) -> Tuple[TurnInPlaceState, TurnInPlaceOutput]:
        tstate, tout = self.turn.step(state.turn, t, yaw_rad,
                                      omega_meas_radps, gyro_scaled)
        if tout.omega_radps == 0.0:
            cmd = stop_command()
        else:
            w = tout.omega_radps
            step = self.max_duty_step
            elapsed = t - (state.turn.t0 if state.turn.t0 is not None else t)
            if self.kick.active(elapsed):
                # See StraightRun.step. Same hypothesis, same bounded
                # amplitude, same default of OFF. A turn breaks away against
                # four scrubbing wheels, so if a kick is ever justified
                # anywhere on this chassis it is here - which is an argument
                # for MEASURING it, not for assuming it.
                w = _sign(w) * max(abs(w), self.kick_radps)
                step = max(step, self.kick_duty)
            cmd = mix_duty(0.0, w, cal=self.cal,
                           limits=self.limits, prev_duty=state.duty,
                           max_duty_step=step)
        new = TurnInPlaceState(turn=tstate, duty=cmd.duty)
        return new, TurnInPlaceOutput(command=cmd, turn=tout, done=tout.done)
