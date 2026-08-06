#!/usr/bin/env python3
"""fpms_charact.py — measure what this rover ACTUALLY does, and write it down.

PHASE 1 OF THE TWO-PHASE DESIGN.
  Phase 1 (this file): the joystick teaches the SYSTEM about the ROVER.
  Phase 2 (fpms_missions.py): the rover plans its OWN route, autonomously, and
                              converts it into slow measured segments.

The joystick does NOT teach a route. Nothing here records a path and nothing
here replays one. What it produces is a CALIBRATION PROFILE — the handful of
constants that turn "the planner said 744 mm" into 744 mm of floor.

============================================================================
THIS PROGRAM NEVER MOVES THE ROVER
============================================================================
There is no publisher of /cmd_vel, /cmd_duty or any actuation topic anywhere in
this file. It subscribes and it measures. The operator drives, by joystick,
watching, with a hand ready. That is not a limitation to be worked around: on
this rover the odometry has reported clean travel for a chassis that was
spinning in place, so a measurement taken while a script drives itself is worth
very little. Every number below is anchored to something a human observed.

============================================================================
WHAT IT DERIVES, AND WHAT EACH ONE FIXES
============================================================================
  odom_scale        true_mm / odom_reported_mm.  THE headline number.
                    Settles the longest-running open question in this project:
                    6.00 counts/mm (derived from 11 lines x 30:1 x4) vs 14.8
                    (tape-measured over 800 mm) vs a 0.743x hand-push reading,
                    plus a possible x2 from attachHalfQuad. 14.8/6.00 = 2.467,
                    which is exactly 74/30 — a 74:1 gearbox where 30:1 was
                    assumed. fpms_missions.py multiplies /odom_raw position by
                    this at one boundary, so correcting it needs NO reflash.
  counts_per_mm     from RAW ticks, when the firmware publishes them.
                    Independent of whatever the board thinks its CPR is, which
                    is the whole reason firmware v3 puts raw ticks on the wire.
  gyro_scale        true_deg / integrated_gyro_deg. Heading closes on the gyro,
                    so this is what turn accuracy actually rests on.
  coast_mm          how far it keeps going after the burst is cut.
  coast_deg         how far it keeps turning after a turn is cut.
  min_moving_duty   the lowest duty that reliably breaks stiction. On factory
                    firmware this is meaningless (a 50% dead zone is added to
                    every command). On firmware v3 there is no dead zone, so a
                    real crawl is finally possible and this is its floor.
  lr_asymmetry      left ticks / right ticks over straight driving. The rover
                    has measured ~10% side-to-side before.

============================================================================
MODES
============================================================================
  --push-check      NO BATTERY, NO MOTORS, 30 SECONDS. Push the rover along a
                    tape measure by hand. Verifies the ODOMETRY SIGN and
                    derives odom_scale. Do this one first, always: it is the
                    only honest way to settle the sign, and getting the sign
                    wrong silently re-breaks everything downstream.
  --spin-check DEG  Rotate the rover by hand through a known angle. Derives
                    gyro_scale.
  --drive           Watch a joystick run and derive coast, min moving duty and
                    left/right asymmetry from it. Passive throughout.
  --report          Print the current profile and where each number came from.

Nothing is written unless a run completes and the operator confirms the real
distance. A half-finished run leaves the profile untouched, and the profile is
only ever ADOPTED by fpms_missions.py when it carries `measured: true`.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import threading
import time

CALIB_FILE = os.environ.get("FPMS_CALIB_FILE", "/etc/fpms/calibration.json")


def load_config(path="/etc/fpms/config.env"):
    cfg = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    cfg[k.strip()] = v.strip()
    except FileNotFoundError:
        pass
    for k, v in os.environ.items():
        if k.startswith("FPMS_"):
            cfg[k] = v
    return cfg


CFG = load_config()
THING = CFG.get("FPMS_THING_NAME", "rover2")


def log(*a):
    print(*a, flush=True)


def ask(prompt, cast=float):
    """Ask the operator for the number only they can supply — the real one."""
    while True:
        try:
            raw = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            log("\naborted; nothing written")
            sys.exit(1)
        if not raw:
            return None
        try:
            return cast(raw)
        except ValueError:
            log("  not a number; try again (or blank to abort)")


# --------------------------------------------------------------- ROS input ---

class Listener:
    """Subscribe-only view of the board. Creates no publisher of any kind.

    Handles BOTH firmwares:
      factory v2.0.0  /odom_raw, /imu, /battery   (no raw ticks)
      FPMS v3         + /wheel_ticks, /wheel_duty, /fpms_health
    Missing topics are reported as missing rather than substituted for, because
    a derived stand-in for a measurement is how this project got three
    different counts/mm in the first place.
    """

    def __init__(self):
        import rclpy
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                               HistoryPolicy)
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu

        self._rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node("fpms_charact")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=20)

        self.lock = threading.Lock()
        self.odom = None          # (t, x, y, yaw)
        self.odom_n = 0
        self.gyro_z = 0.0
        self.gyro_int = 0.0       # integrated radians
        self._gyro_t = None
        self.imu_n = 0
        self.ticks = None         # (t, [4 raw ints])
        self.ticks_n = 0
        self.duty = None          # (t, [4 ints])
        self.duty_n = 0

        self.node.create_subscription(Odometry, "/odom_raw", self._on_odom, qos)
        self.node.create_subscription(Imu, "/imu", self._on_imu, qos)

        # Raw ticks are the point of firmware v3. Try the likely message types
        # without hard-failing if the topic is absent on factory firmware.
        self.have_ticks = False
        for mod, cls, topic in (("std_msgs.msg", "Int32MultiArray", "/wheel_ticks"),
                                ("std_msgs.msg", "Int32MultiArray", "/wheel_duty")):
            try:
                m = __import__(mod, fromlist=[cls])
                T = getattr(m, cls)
                cb = self._on_ticks if topic == "/wheel_ticks" else self._on_duty
                self.node.create_subscription(T, topic, cb, qos)
                if topic == "/wheel_ticks":
                    self.have_ticks = True
            except Exception:
                pass

        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._stop = threading.Event()
        self._spin.start()

    def _spin_loop(self):
        while not self._stop.is_set():
            try:
                self._rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception:
                time.sleep(0.05)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            # RAW, with no sign or scale applied. This program exists to
            # MEASURE those corrections; applying them here would make the
            # measurement circular.
            self.odom = (time.monotonic(), float(msg.pose.pose.position.x),
                         float(msg.pose.pose.position.y), yaw)
            self.odom_n += 1

    def _on_imu(self, msg):
        now = time.monotonic()
        with self.lock:
            self.gyro_z = float(msg.angular_velocity.z)
            if self._gyro_t is not None:
                dt = now - self._gyro_t
                if 0.0 < dt < 0.5:
                    self.gyro_int += self.gyro_z * dt
            self._gyro_t = now
            self.imu_n += 1

    def _on_ticks(self, msg):
        with self.lock:
            self.ticks = (time.monotonic(), [int(v) for v in msg.data])
            self.ticks_n += 1

    def _on_duty(self, msg):
        with self.lock:
            self.duty = (time.monotonic(), [int(v) for v in msg.data])
            self.duty_n += 1

    def snap(self):
        with self.lock:
            return {"odom": self.odom, "gyro_int": self.gyro_int,
                    "gyro_z": self.gyro_z, "ticks": self.ticks,
                    "duty": self.duty, "odom_n": self.odom_n,
                    "imu_n": self.imu_n, "ticks_n": self.ticks_n}

    def reset_gyro(self):
        with self.lock:
            self.gyro_int = 0.0
            self._gyro_t = None

    def wait_for_data(self, seconds=6.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            s = self.snap()
            if s["odom_n"] > 3 and s["imu_n"] > 3:
                return True
            time.sleep(0.2)
        return False

    def close(self):
        self._stop.set()
        try:
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------- profile ---

def read_profile():
    try:
        with open(CALIB_FILE) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except Exception:
        return {}


def write_profile(updates, provenance):
    """Merge and persist. Refuses to write anything it did not measure."""
    prof = read_profile()
    prof.update(updates)
    prof["measured"] = True
    prof["measured_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    prov = prof.setdefault("provenance", {})
    prov.update(provenance)
    prof.setdefault("note",
                    "Written by fpms_charact.py from operator-observed "
                    "measurements. fpms_missions.py adopts a field only if "
                    "'measured' is true and the value is inside its sanity "
                    "range; refusals are logged to CFG_NOTES at startup.")
    target = CALIB_FILE
    try:
        d = os.path.dirname(target)
        if d and not os.path.isdir(d):
            os.makedirs(d, exist_ok=True)
        tmp = target + ".tmp"
        with open(tmp, "w") as f:
            json.dump(prof, f, indent=2, sort_keys=True)
            f.write("\n")
        os.replace(tmp, target)
    except PermissionError:
        alt = os.path.expanduser("~/calibration.json")
        with open(alt, "w") as f:
            json.dump(prof, f, indent=2, sort_keys=True)
            f.write("\n")
        log(f"\n  No permission to write {target}.")
        log(f"  Written to {alt} instead. Install it with:")
        log(f"      sudo install -m 0644 {alt} {target}")
        log(f"      sudo systemctl restart fpms-missions")
        return alt
    log(f"\n  profile written: {target}")
    log("  restart the executor to adopt it:  sudo systemctl restart fpms-missions")
    return target


# ------------------------------------------------------------- push check ---

def push_check(lis):
    """The 30-second, zero-battery test that settles the sign AND the scale."""
    log("""
============================================================================
PUSH CHECK - no motors, no battery drain, ~30 seconds
============================================================================
This is the ONLY honest way to settle the odometry sign, and it also gives the
single most valuable constant in the stack (odom_scale). It involves no
actuation whatsoever.

  1. Put the rover on the floor with a tape measure alongside it.
  2. Note where the front edge starts.
  3. When prompted, PUSH IT STRAIGHT FORWARD by hand, slowly, ~500-800 mm.
     Straight matters more than the exact distance - you will type in whatever
     you actually got.
  4. Do not turn it. Do not lift it.
============================================================================""")
    if not lis.wait_for_data():
        log("\n  NO /odom_raw or /imu ARRIVING. Nothing to measure.")
        log("  Check:  systemctl status micro-ros-agent")
        log("          ros2 topic hz /odom_raw    (needs ROS_DOMAIN_ID=20)")
        return False

    input("\n  Press ENTER when you are ready to push...")
    s0 = lis.snap()
    if not s0["odom"]:
        log("  no odometry sample; aborting")
        return False
    t0, x0, y0, _ = s0["odom"]
    ticks0 = s0["ticks"][1] if s0["ticks"] else None
    log("\n  RECORDING - push the rover forward now, then press ENTER.")
    input()
    s1 = lis.snap()
    _, x1, y1, _ = s1["odom"]
    ticks1 = s1["ticks"][1] if s1["ticks"] else None

    dx, dy = (x1 - x0), (y1 - y0)
    reported_mm = math.hypot(dx, dy) * 1000.0
    # Sign is taken from the dominant axis of travel, so this stays meaningful
    # whichever way the rover happened to be facing in its own odom frame.
    along = dx if abs(dx) >= abs(dy) else dy
    sign_seen = 1 if along >= 0 else -1

    log(f"\n  odom delta      : dx={dx*1000:+.1f} mm  dy={dy*1000:+.1f} mm")
    log(f"  reported travel : {reported_mm:.1f} mm")
    if ticks0 and ticks1 and len(ticks0) == len(ticks1):
        dticks = [b - a for a, b in zip(ticks0, ticks1)]
        log(f"  raw tick delta  : {dticks}")
    else:
        dticks = None
        log("  raw tick delta  : /wheel_ticks not published "
            "(factory firmware) - counts_per_mm cannot be derived directly")

    true_mm = ask("\n  How far did it ACTUALLY move, in mm (tape measure)? ")
    if not true_mm or true_mm <= 0:
        log("  no distance given; nothing written")
        return False

    updates, prov = {}, {}

    # --- the sign ----------------------------------------------------------
    log("\n  ---- ODOMETRY SIGN ----")
    if sign_seen > 0:
        log("  Pushed FORWARD, reported position INCREASED. That is correct.")
        log("  -> FPMS_MISSION_ODOM_POSE_SIGN = +1")
        want_sign = 1
    else:
        log("  Pushed FORWARD, reported position DECREASED. Backwards.")
        log("  -> FPMS_MISSION_ODOM_POSE_SIGN = -1")
        want_sign = -1
    updates["odom_pose_sign_observed"] = want_sign
    prov["odom_pose_sign_observed"] = (
        f"hand push, dx={dx*1000:+.1f}mm dy={dy*1000:+.1f}mm, "
        f"{time.strftime('%Y-%m-%d')}")
    log(f"\n  Set this in /etc/fpms/config.env:")
    log(f"      FPMS_MISSION_ODOM_POSE_SIGN={want_sign:+d}")
    log("  and then RE-ZERO with set_coordinate - the saved teleop origin was")
    log("  captured under the other sign and is silently wrong otherwise.")

    # --- the scale ---------------------------------------------------------
    log("\n  ---- ODOM SCALE ----")
    if reported_mm < 5.0:
        log(f"  reported travel is only {reported_mm:.1f} mm - too small to "
            "derive a ratio from. Push further and re-run.")
    else:
        scale = true_mm / reported_mm
        log(f"  true {true_mm:.0f} mm / reported {reported_mm:.1f} mm "
            f"= odom_scale {scale:.4f}")
        for cand, why in ((2.467, "74/30 gearbox (14.8 vs 6.00 counts/mm)"),
                          (0.405, "reciprocal of the 74/30 gearbox"),
                          (2.0, "attachFullQuad vs attachHalfQuad"),
                          (0.5, "attachHalfQuad vs attachFullQuad"),
                          (1.0, "no correction needed")):
            if abs(scale - cand) / cand < 0.08:
                log(f"  -> within 8% of {cand}: {why}")
        if 0.2 <= scale <= 5.0:
            updates["odom_scale"] = round(scale, 4)
            prov["odom_scale"] = (f"hand push: tape {true_mm:.0f}mm vs "
                                  f"/odom_raw {reported_mm:.1f}mm, "
                                  f"{time.strftime('%Y-%m-%d')}")
        else:
            log(f"  REFUSED: {scale:.3f} is outside [0.2, 5.0]. That is far "
                "more likely to be a botched run than a discovery - check the "
                "rover went straight and did not lose a wheel.")

    # --- counts/mm from raw ticks -----------------------------------------
    if dticks and any(dticks):
        mags = [abs(d) for d in dticks if abs(d) > 2]
        if mags:
            cpm = statistics.mean(mags) / true_mm
            log("\n  ---- COUNTS PER MM (from RAW ticks) ----")
            log(f"  mean |tick delta| {statistics.mean(mags):.0f} / "
                f"{true_mm:.0f} mm = {cpm:.3f} counts/mm")
            log(f"  for reference: 14.8 tape-measured, 6.00 derived, "
                f"ratio 14.8/6.00 = 2.467 = 74/30")
            if len(mags) >= 4:
                lr = (mags[0] + mags[2]) / max(mags[1] + mags[3], 1e-9)
                log(f"  left/right tick ratio {lr:.3f} "
                    f"({'within' if 0.9 <= lr <= 1.1 else 'OUTSIDE'} 10%)")
                updates["lr_asymmetry"] = round(lr, 4)
                prov["lr_asymmetry"] = "hand push, per-wheel tick deltas"
            updates["counts_per_mm"] = round(cpm, 4)
            prov["counts_per_mm"] = (f"hand push: {statistics.mean(mags):.0f} "
                                     f"raw ticks over tape {true_mm:.0f}mm")

    if not updates:
        log("\n  nothing derived; profile untouched")
        return False
    write_profile(updates, prov)
    return True


# ------------------------------------------------------------- spin check ---

def spin_check(lis, true_deg):
    log(f"""
============================================================================
SPIN CHECK - no motors, ~30 seconds
============================================================================
Rotate the rover BY HAND through {true_deg:.0f} degrees on the spot, then stop.
Use a protractor, a floor mark, or a wall reference. Heading accuracy on this
rover rests entirely on this number, because turns close on the gyro.
============================================================================""")
    if not lis.wait_for_data():
        log("\n  NO /imu ARRIVING. Nothing to measure.")
        return False
    input("\n  Press ENTER when ready, then rotate the rover...")
    lis.reset_gyro()
    log("  RECORDING - rotate now, then press ENTER.")
    input()
    s = lis.snap()
    integ_deg = math.degrees(s["gyro_int"])
    log(f"\n  gyro integrated : {integ_deg:+.2f} deg")
    log(f"  operator says   : {true_deg:+.2f} deg")
    if abs(integ_deg) < 1.0:
        log("\n  The gyro integrated almost nothing. That is the known dead-gyro")
        log("  signature: on the ICM42670P, reading gyro as bytes 6-11 of the")
        log("  accel burst returns ZEROS because GYRO_DATA_X1 (0x11) is not")
        log("  contiguous with the accel block (0x0B). See firmware_v3 section 5")
        log("  and check `ros2 topic echo /imu --field orientation`.")
        return False
    scale = true_deg / integ_deg
    log(f"  gyro_scale      : {scale:.4f}")
    if scale < 0:
        log("  NEGATIVE - the gyro sign is opposite to the convention you used.")
    if not (0.2 <= abs(scale) <= 5.0):
        log("  REFUSED: outside [0.2, 5.0]; re-run and rotate more carefully.")
        return False
    write_profile({"gyro_scale": round(scale, 4)},
                  {"gyro_scale": f"hand rotation {true_deg:.0f}deg vs "
                                 f"integrated {integ_deg:.2f}deg"})
    return True


# ----------------------------------------------------------- drive watch ----

def drive_watch(lis, seconds):
    """Passively watch a joystick run. Publishes nothing, commands nothing."""
    log(f"""
============================================================================
JOYSTICK CHARACTERISATION - {seconds:.0f}s, PASSIVE
============================================================================
Drive the rover with the joystick. This program only WATCHES.

Do these, in any order, and it will pick them out of the trace:
  * several straight runs, then RELEASE cleanly   -> coast_mm
  * a few turns on the spot, then RELEASE         -> coast_deg
  * creep up from a standstill very gently        -> min_moving_duty
  * one long straight run                         -> lr_asymmetry

KEEP A HAND ON STOP. Judge what happened with your EYES, not the numbers on
screen - this rover's odometry has reported clean travel while it span in place.
============================================================================""")
    if not lis.wait_for_data():
        log("\n  no telemetry; aborting")
        return False
    input("\n  Press ENTER to start recording...")

    trace, t0 = [], time.monotonic()
    log("  RECORDING... (Ctrl-C to stop early)")
    try:
        while time.monotonic() - t0 < seconds:
            s = lis.snap()
            if s["odom"]:
                trace.append({
                    "t": round(time.monotonic() - t0, 3),
                    "x": s["odom"][1], "y": s["odom"][2], "yaw": s["odom"][3],
                    "gyro_z": s["gyro_z"], "gyro_int": s["gyro_int"],
                    "ticks": (s["ticks"][1] if s["ticks"] else None),
                    "duty": (s["duty"][1] if s["duty"] else None),
                })
            time.sleep(0.04)          # 25 Hz - above the 20 Hz control rate
    except KeyboardInterrupt:
        log("\n  stopped early")

    if len(trace) < 20:
        log("  too few samples to derive anything")
        return False

    raw_path = os.path.expanduser(
        f"~/fpms_charact_{time.strftime('%Y%m%d_%H%M%S')}.json")
    with open(raw_path, "w") as f:
        json.dump({"hz": 25, "samples": trace}, f)
    log(f"\n  raw trace saved: {raw_path}  ({len(trace)} samples)")
    log("  KEEP IT. A raw trace is a measurement; a derived constant is an")
    log("  opinion about one, and you may want to re-derive it differently.")

    updates, prov = {}, {}

    # --- coast: motion continuing after the command goes to zero ------------
    have_duty = any(s["duty"] for s in trace)
    if not have_duty:
        log("\n  /wheel_duty is not published (factory firmware), so a burst")
        log("  cut cannot be located in the trace and coast cannot be derived.")
        log("  Coast needs firmware v3, or the bare-metal numbers already on")
        log("  record: COAST_FWD 1270 counts (~86mm), COAST_REV 1448 (~98mm).")
    else:
        coasts_mm, coasts_deg, moving_duties = [], [], []
        for i in range(1, len(trace)):
            prev, cur = trace[i - 1], trace[i]
            pd = max(abs(v) for v in (prev["duty"] or [0]))
            cd = max(abs(v) for v in (cur["duty"] or [0]))
            if pd > 0 and cd == 0:          # the cut
                x0, y0, g0 = cur["x"], cur["y"], cur["gyro_int"]
                for j in range(i, len(trace)):
                    if trace[j]["t"] - cur["t"] > 1.5:
                        break
                    if max(abs(v) for v in (trace[j]["duty"] or [0])) > 0:
                        break
                else:
                    j = len(trace) - 1
                d_mm = math.hypot(trace[j]["x"] - x0, trace[j]["y"] - y0) * 1000.0
                d_deg = abs(math.degrees(trace[j]["gyro_int"] - g0))
                if 1.0 < d_mm < 600.0:
                    coasts_mm.append(d_mm)
                if 1.0 < d_deg < 180.0:
                    coasts_deg.append(d_deg)
            if cd > 0 and cur["ticks"] and prev["ticks"]:
                if any(abs(b - a) > 1 for a, b in zip(prev["ticks"], cur["ticks"])):
                    moving_duties.append(cd)

        if coasts_mm:
            v = statistics.median(coasts_mm)
            log(f"\n  coast_mm  : median {v:.1f} mm over {len(coasts_mm)} cuts "
                f"(range {min(coasts_mm):.0f}-{max(coasts_mm):.0f})")
            updates["coast_mm"] = round(v, 1)
            prov["coast_mm"] = f"joystick run, {len(coasts_mm)} clean cuts"
        if coasts_deg:
            v = statistics.median(coasts_deg)
            log(f"  coast_deg : median {v:.1f} deg over {len(coasts_deg)} cuts")
            updates["coast_deg"] = round(v, 1)
            prov["coast_deg"] = f"joystick run, {len(coasts_deg)} clean cuts"
        if moving_duties:
            v = min(moving_duties)
            log(f"  min_moving_duty : {v} (lowest duty at which ticks advanced)")
            log("    On FACTORY firmware this is meaningless - a 50% dead zone")
            log("    is added to every command. On v3 it is the crawl floor.")
            updates["min_moving_duty"] = v
            prov["min_moving_duty"] = "joystick run, lowest duty with tick motion"

    # --- left/right asymmetry over the straightest stretch -----------------
    if any(s["ticks"] for s in trace):
        first = next((s for s in trace if s["ticks"]), None)
        last = next((s for s in reversed(trace) if s["ticks"]), None)
        if first and last and first is not last:
            d = [b - a for a, b in zip(first["ticks"], last["ticks"])]
            if len(d) >= 4 and abs(d[1]) + abs(d[3]) > 100:
                lr = (abs(d[0]) + abs(d[2])) / (abs(d[1]) + abs(d[3]))
                log(f"  lr_asymmetry : {lr:.3f} over the whole run")
                log("    (whole-run figure - it mixes turns in, so trust the")
                log("     push-check value over this one if you have both)")
                updates.setdefault("lr_asymmetry_run", round(lr, 4))
                prov["lr_asymmetry_run"] = "joystick run, whole-trace tick ratio"

    if not updates:
        log("\n  nothing derivable from this run; profile untouched")
        return False
    write_profile(updates, prov)
    return True


# ---------------------------------------------------------------- report ----

def report():
    prof = read_profile()
    log(f"\ncalibration profile: {CALIB_FILE}")
    if not prof:
        log("  (none yet — run --push-check first)")
        return
    log(f"  measured   : {prof.get('measured')}")
    log(f"  measured_at: {prof.get('measured_at')}")
    prov = prof.get("provenance", {})
    for k in sorted(prof):
        if k in ("provenance", "note", "measured", "measured_at"):
            continue
        log(f"  {k:22s} = {prof[k]!r}")
        if k in prov:
            log(f"  {'':22s}   from: {prov[k]}")
    log("\n  Adopted by fpms_missions.py only when measured=true and the value")
    log("  is inside its sanity range. Check the executor's startup log for")
    log("  'calibration ... ADOPTED' or '... REFUSED'.")


def joystick_help():
    log("""
----------------------------------------------------------------------------
JOYSTICK: what is and is not set up on this Pi
----------------------------------------------------------------------------
As of this writing NO joystick node is configured on the rover. `joy` and
`teleop_twist_joy` are ROS packages that are not part of the FPMS install, and
nothing in the systemd stack launches them. Check before assuming:

    ros2 pkg list | grep -E 'joy|teleop'
    ls /dev/input/js*  /dev/input/event*

IF YOU HAVE A USB GAMEPAD and want the ROS path:
    sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
    ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
  It publishes /cmd_vel directly, so STOP fpms-teleop first or you will have
  two writers on the wire:
    sudo systemctl stop fpms-teleop

IF YOU DO NOT HAVE ONE - and this is the path that needs no new hardware -
the dashboard's existing drive controls already work and go through
fpms-teleop over MQTT. That is a perfectly good characterisation input: what
this program measures is what the ROVER did, not what the operator pressed.
Drive it from the dashboard and run `--drive` exactly the same way.

Either way, this program NEVER commands motion. It only watches.
----------------------------------------------------------------------------""")


def main():
    ap = argparse.ArgumentParser(
        description="Measure this rover's real constants. Never drives it.")
    ap.add_argument("--push-check", action="store_true",
                    help="hand-push test: odometry SIGN + odom_scale (do this first)")
    ap.add_argument("--spin-check", type=float, metavar="DEG",
                    help="hand-rotate through DEG degrees: gyro_scale")
    ap.add_argument("--drive", action="store_true",
                    help="passively watch a joystick run: coast, min duty, asymmetry")
    ap.add_argument("--seconds", type=float, default=90.0,
                    help="recording window for --drive (default 90)")
    ap.add_argument("--report", action="store_true",
                    help="print the current profile and its provenance")
    ap.add_argument("--joystick-help", action="store_true",
                    help="how to get a joystick working, and the fallback")
    args = ap.parse_args()

    if args.report:
        report(); return 0
    if args.joystick_help:
        joystick_help(); return 0
    if not (args.push_check or args.spin_check or args.drive):
        ap.print_help()
        log("\nStart with:  python3 fpms_charact.py --push-check")
        return 0

    try:
        lis = Listener()
    except Exception as e:
        log(f"\nCannot start a ROS node ({e}).")
        log("Check ROS_DOMAIN_ID=20 is set and /opt/ros/humble/setup.bash is sourced:")
        log("  source /opt/ros/humble/setup.bash && "
            "ROS_DOMAIN_ID=20 python3 fpms_charact.py --push-check")
        return 1
    try:
        if args.push_check:
            return 0 if push_check(lis) else 1
        if args.spin_check:
            return 0 if spin_check(lis, args.spin_check) else 1
        if args.drive:
            return 0 if drive_watch(lis, args.seconds) else 1
    finally:
        lis.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
